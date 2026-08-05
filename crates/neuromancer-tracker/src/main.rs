//! `neuromancer-tracker` — Nreal Light head tracker (Phase 1: 3-DoF).
//!
//! Data flow (spec §1.6): USB IMU → Mahony AHRS → axis mapping →
//! Opentrack UDP (≥ 60 Hz, rate-gated) + 2 Hz HUD + optional logs.
//! Single-threaded event loop, no async runtime (spec §2.7).

use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use neuromancer_ahrs::{quat_to_ypr, Mahony};
use neuromancer_fusion::{Eskf, EskfConfig};
use neuromancer_tracker::axis::AxisMap;
use neuromancer_tracker::calib::{BiasRefresher, GyroCalibrator};
use neuromancer_tracker::cli::{self, Config, ParseOutcome};
use neuromancer_tracker::imu::{ArDriversSource, ImuError, ImuSource, ReplaySource};
use neuromancer_tracker::log;
use neuromancer_tracker::output::{Frame, HudSink, ImuLogSink, PoseLogSink, Sink, UdpSink};
use neuromancer_tracker::visual::{ReplayVisualSource, SlamCameraSource, VisualRecorder, VisualSource};
use neuromancer_tracker::{log_error, log_info, log_warn};
use neuromancer_vo::pipeline::VoPipeline;
use neuromancer_vo::stereo::StereoMatcher;
use neuromancer_vo::{UnitQuaternion, Vector3};

/// Ctrl-C press counter: 1 = graceful shutdown requested, ≥2 = force exit.
static CTRL_C: AtomicUsize = AtomicUsize::new(0);

/// Cap on the filter `dt` (seconds): protects the gyro integration from a
/// huge jump after a USB stall or replay gap (spec §3.3: robust to jitter).
const MAX_DT: f64 = 0.1;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match cli::parse(args) {
        Ok(ParseOutcome::Run(c)) => *c,
        Ok(ParseOutcome::Help) => {
            print!("{}", cli::usage());
            return ExitCode::SUCCESS;
        }
        Ok(ParseOutcome::Version) => {
            println!("neuromancer-tracker {}", env!("CARGO_PKG_VERSION"));
            return ExitCode::SUCCESS;
        }
        Err(e) => {
            eprintln!("error: {e}\n\n{}", cli::usage());
            return ExitCode::from(2);
        }
    };
    run(cfg)
}

fn run(cfg: Config) -> ExitCode {
    // Apply the requested log verbosity first — everything below is gated.
    log::set_level(cfg.log_level);

    // P2 input switch: `visual` (6-DoF, IMU fully off) runs the stereo VO
    // pipeline; `imu+visual` (P2b) runs the ESKF fusion loop.
    if cfg.input_source == cli::InputSource::Visual {
        return run_visual(cfg);
    }
    if cfg.input_source == cli::InputSource::Fusion {
        return run_fusion(cfg);
    }

    // --- Step 2/3: open the IMU source (exit 1 on failure) ---------------
    let mut source: Box<dyn ImuSource> = match &cfg.replay {
        Some(path) => match ReplaySource::open(path, true) {
            Ok(s) => Box::new(s),
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::from(1);
            }
        },
        None => match ArDriversSource::open() {
            Ok(s) => Box::new(s),
            Err(e) => {
                log_error!("error: {e}");
                return ExitCode::from(1);
            }
        },
    };

    // --- Step 4: filter + axis mapping ------------------------------------
    let mut mahony = Mahony::with_gains(cfg.kp, cfg.ki);
    let axis_map = AxisMap {
        invert_yaw: cfg.invert_yaw,
        invert_pitch: cfg.invert_pitch,
        invert_roll: cfg.invert_roll,
        sensitivity: cfg.sensitivity,
    };

    // --- Step 5: sinks (UDP bind failure: warn + continue, spec §3.2) -----
    let (mut sinks, mut imu_log) = build_sinks(&cfg);

    // --- Signal handling (spec §3.5) ---------------------------------------
    install_signal_handler();

    // --- Startup confirmation line (spec §3.2) -----------------------------
    let udp_out = if cfg.no_udp {
        "disabled".to_string()
    } else {
        format!("{}:{}", cfg.host, cfg.port)
    };
    let calib_out = if cfg.gyro_calib > 0.0 {
        format!("{}s", cfg.gyro_calib)
    } else {
        "off".to_string()
    };
    println!(
        "device={} kp={} ki={} out={} protocol={} units={} udp_rate={}Hz hud={} calib={}",
        source.name(),
        cfg.kp,
        cfg.ki,
        udp_out,
        match cfg.protocol {
            cli::Protocol::Classic => "classic",
            cli::Protocol::Extended => "extended",
        },
        match cfg.units {
            cli::Units::Deg => "deg",
            cli::Units::Rad => "rad",
        },
        cfg.udp_rate,
        if cfg.hud { "on" } else { "off" },
        calib_out,
    );

    // --- Startup gyro-bias calibration (see calib.rs) -----------------------
    // A residual turn-on gyro bias integrates into linear yaw drift with no
    // magnetometer reference (measured ~15°/60 s ≈ 0.25°/s on real glasses).
    let mut calib = (cfg.gyro_calib > 0.0).then(|| GyroCalibrator::new(cfg.gyro_calib));
    let mut calib_notice = true;
    // After startup, keep refreshing the bias whenever the device rests still
    // for a window — tracks in-run thermal drift of the gyro.
    let mut refresher = BiasRefresher::new(cfg.gyro_calib, [0.0; 3]);

    // --- Main loop (spec §3.3) ---------------------------------------------
    let mut prev_t: Option<f64> = None;
    let mut t_first: Option<f64> = None;
    let mut n_samples: u64 = 0;
    let mut rate_reported = false;

    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            log_info!("SIGINT received — shutting down cleanly");
            break;
        }

        let sample = match source.next_sample() {
            Ok(s) => s,
            Err(ImuError::Eof) => {
                log_info!("replay input exhausted — shutting down cleanly");
                break;
            }
            Err(ImuError::Io(e)) => {
                // Malformed/unreadable replay file — a bad input, not a
                // hardware failure (exit code 1, not 3).
                log_error!("error: replay input failed: {e}");
                return ExitCode::from(1);
            }
            Err(e) => {
                // A Ctrl-C that lands while blocked in the USB read makes
                // hidapi report "unplugged?" — that's the signal interrupting
                // the read, not a device failure. Check the flag before
                // declaring the device gone (spike finding 2026-08-04).
                if CTRL_C.load(Ordering::SeqCst) >= 1 {
                    log_info!("SIGINT received — shutting down cleanly");
                    break;
                }
                log_error!("error: IMU input failed: {e}");
                log_error!(
                    "(device unplugged — restart the tracker to reconnect; exit code 3)"
                );
                return ExitCode::from(3);
            }
        };

        // Raw samples are logged regardless of calibration phase, so
        // --log-imu always records the untouched sensor stream.
        if let Some(log) = &mut imu_log {
            log.write_sample(&sample);
        }

        // --- Calibration phase: consume samples until the bias is known -----
        let mut calib_done: Option<neuromancer_tracker::calib::CalibResult> = None;
        if let Some(c) = calib.as_mut() {
            if calib_notice {
                log_info!(
                    "gyro bias calibration: keep the device still for {:.1}s ...",
                    cfg.gyro_calib
                );
                calib_notice = false;
            }
            calib_done = c.push(&sample);
        }
        if let Some(res) = calib_done {
            if res.complete {
                log_info!(
                    "gyro bias calibrated: gx={:.6} gy={:.6} gz={:.6} rad/s ({} still samples, std [{:.4}, {:.4}, {:.4}])",
                    res.bias[0], res.bias[1], res.bias[2], res.samples,
                    res.std[0], res.std[1], res.std[2]
                );
            } else {
                log_warn!(
                    "gyro calibration incomplete — device was moving? using {} samples, std [{:.4}, {:.4}, {:.4}]",
                    res.samples, res.std[0], res.std[1], res.std[2]
                );
            }
            // Start orientation fresh from here (yaw reference = calibration end).
            mahony.reset();
            prev_t = Some(sample.t);
            n_samples = 0;
            t_first = None;
            rate_reported = false;
            refresher.set_bias(res.bias);
            calib = None;
        }
        if calib.is_some() {
            continue; // still calibrating: no filter/output yet
        }

        // --- In-run bias refresh on stillness (tracks thermal drift) -------
        if let Some(update) = refresher.feed(&sample) {
            if update.complete {
                log_info!(
                    "in-run gyro bias refreshed: gx={:.6} gy={:.6} gz={:.6} rad/s (was [{:.4}, {:.4}, {:.4}], {} still samples)",
                    update.new[0], update.new[1], update.new[2],
                    update.old[0], update.old[1], update.old[2],
                    update.samples
                );
            } else {
                log_warn!(
                    "in-run gyro calibration incomplete — device moving? keeping bias [{:.4}, {:.4}, {:.4}]",
                    update.old[0], update.old[1], update.old[2]
                );
            }
        }

        // dt from sample arrival timestamps (monotonic), clamped (spec §3.3).
        let dt = match prev_t {
            Some(pt) => (sample.t - pt).clamp(0.0, MAX_DT),
            None => 0.0,
        };
        prev_t = Some(sample.t);

        // Filter consumes every sample (no input downsampling); the measured
        // gyro bias is subtracted first (calib.rs).
        let bias = refresher.bias();
        let q = mahony.update(
            [sample.ax, sample.ay, sample.az],
            [sample.gx - bias[0], sample.gy - bias[1], sample.gz - bias[2]],
            dt,
        );
        let mut ypr_deg = quat_to_ypr(q).map(f64::to_degrees);
        axis_map.apply(&mut ypr_deg);

        let frame = Frame::new_3dof(sample.t, ypr_deg);
        for sink in sinks.iter_mut() {
            sink.write(&frame);
        }

        // Measured IMU rate (OQ3): report once after 30 samples.
        n_samples += 1;
        if n_samples == 1 {
            t_first = Some(sample.t);
        }
        if !rate_reported && n_samples == 30 {
            let rate = 29.0 / (sample.t - t_first.unwrap_or(sample.t)).max(1e-9);
            log_info!("measured imu_rate={rate:.0} Hz");
            rate_reported = true;
        }
    }

    // --- Clean shutdown (flush logs via Drop, exit 0) ----------------------
    drop(imu_log);
    drop(sinks);
    log_info!("tracker exited");
    ExitCode::SUCCESS
}

/// Build the output sinks from the config (shared by imu and visual modes).
fn build_sinks(cfg: &Config) -> (Vec<Box<dyn Sink>>, Option<ImuLogSink>) {
    let mut sinks: Vec<Box<dyn Sink>> = Vec::new();
    if !cfg.no_udp {
        match UdpSink::bind(&cfg.host, cfg.port, cfg.protocol, cfg.units, cfg.udp_rate) {
            Ok(s) => sinks.push(Box::new(s)),
            Err(e) => log_warn!(
                "UDP output disabled — cannot bind sender to {}:{}: {e}",
                cfg.host, cfg.port
            ),
        }
    }
    if cfg.hud {
        sinks.push(Box::new(HudSink::<std::io::Stdout>::stdout(cfg.hud_rate)));
    }
    if let Some(path) = &cfg.log_pose {
        match PoseLogSink::open(path, cfg.units) {
            Ok(s) => sinks.push(Box::new(s)),
            Err(e) => log_warn!("pose log disabled — cannot open {}: {e}", path.display()),
        }
    }
    let imu_log = match &cfg.log_imu {
        Some(path) => match ImuLogSink::open(path) {
            Ok(s) => Some(s),
            Err(e) => {
                log_warn!("IMU log disabled — cannot open {}: {e}", path.display());
                None
            }
        },
        None => None,
    };
    (sinks, imu_log)
}

/// SIGINT handler — must be async-signal-safe: lock-free atomic RMW and
/// `_exit` only (no allocation, no locks, no logging).
///
/// The flag is set **in signal context**, so a Ctrl-C that interrupts the
/// blocking USB read is already visible to the main thread when the read
/// returns its error — unlike `ctrlc`'s handler, which only posts a
/// semaphore and increments the flag from a separate waiter thread that may
/// not be scheduled before the main thread checks it (the race that made
/// Ctrl-C during an IMU read exit 3 with a bogus "unplugged" error).
extern "C" fn on_sigint(_: libc::c_int) {
    let n = CTRL_C.fetch_add(1, Ordering::SeqCst) + 1;
    if n >= 2 {
        // Second Ctrl-C: immediate exit 1. `_exit` (not `process::exit`,
        // which runs atexit handlers) is the async-signal-safe choice;
        // buffered log lines are lost, accepted trade-off.
        unsafe { libc::_exit(1) }
    }
}

/// Install the Ctrl-C handler (first press = graceful, second = force exit).
fn install_signal_handler() {
    // SAFETY: standard sigaction setup; the handler only touches the
    // lock-free atomic above. SA_RESTART is deliberately NOT set so that a
    // Ctrl-C interrupts the blocking USB read (EINTR) instead of silently
    // restarting it, letting the loop observe the request promptly.
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigint as *const () as libc::sighandler_t;
        libc::sigemptyset(&mut sa.sa_mask);
        sa.sa_flags = 0;
        // Match ctrlc's default surface (SIGINT/SIGTERM/SIGHUP) so `kill`
        // and terminal close keep the documented clean-shutdown behavior.
        for sig in [libc::SIGINT, libc::SIGTERM, libc::SIGHUP] {
            if libc::sigaction(sig, &sa, std::ptr::null_mut()) != 0 {
                eprintln!("warning: cannot install signal handler for {sig}");
            }
        }
    }
}

/// 6-DoF visual mode (`--input visual`): stereo frames → VO pipeline → pose
/// → sinks. The IMU is never opened (spec Appendix D — the "IMU off" switch).
fn run_visual(cfg: Config) -> ExitCode {
    // --- Visual source (hardware or replay) --------------------------------
    let mut source: Box<dyn VisualSource> = match &cfg.replay_visual {
        Some(dir) => match ReplayVisualSource::open(dir, 640, 480) {
            Ok(s) => Box::new(s),
            Err(e) => {
                log_error!("error: {e}");
                return ExitCode::from(1);
            }
        },
        None => match SlamCameraSource::open() {
            Ok(s) => Box::new(s),
            Err(e) => {
                log_error!("error: {e}");
                return ExitCode::from(1);
            }
        },
    };

    let (mut sinks, _) = build_sinks(&cfg);
    install_signal_handler();

    // Optional raw-frame recording (P2 --record-visual).
    let mut recorder = match &cfg.record_visual {
        Some(dir) => match VisualRecorder::open(dir) {
            Ok(r) => Some(r),
            Err(e) => {
                log_warn!("visual recording disabled: {e}");
                None
            }
        },
        None => None,
    };
    let mut recorded = 0usize;
    let mut record_notice = true;

    // M8: build the rectified rig from the real camera calibration — the
    // on-device CameraDescriptor when the glasses are present, otherwise
    // this unit's captured constants (replay). The rectifier remaps each raw
    // (distorted, un-rectified) frame into the canonical rectified pair the
    // VO pipeline expects (same intrinsics, X-only baseline, aligned rows).
    let (rectifier, imu_to_cam_left) =
        neuromancer_tracker::cam_calib::build_rectifier(cfg.replay_visual.is_none());
    let rig = rectifier.rig.clone();
    let mut vo = VoPipeline::new(rig, StereoMatcher::new(5, 0.5, 10.0));

    let udp_out = if cfg.no_udp {
        "disabled".to_string()
    } else {
        format!("{}:{}", cfg.host, cfg.port)
    };
    println!(
        "device={} input=visual out={} protocol={} units={} udp_rate={}Hz hud={}",
        source.name(),
        udp_out,
        match cfg.protocol {
            cli::Protocol::Classic => "classic",
            cli::Protocol::Extended => "extended",
        },
        match cfg.units {
            cli::Units::Deg => "deg",
            cli::Units::Rad => "rad",
        },
        cfg.udp_rate,
        if cfg.hud { "on" } else { "off" },
    );

    let mut frames: u64 = 0;
    let mut rate_reported = false;
    let mut t_first: Option<f64> = None;
    let run_start = Instant::now();
    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            log_info!("SIGINT received — shutting down cleanly");
            break;
        }
        let frame = match source.next_frame() {
            Ok(f) => f,
            Err(e) if e == "end of visual replay" => {
                log_info!("visual replay input exhausted — shutting down cleanly");
                break;
            }
            Err(e) => {
                log_error!("error: visual input failed: {e}");
                return ExitCode::from(3);
            }
        };
        // Record the raw stereo frames before processing (P2).
        if let Some(r) = &mut recorder {
            r.record(&frame);
            if record_notice {
                log_info!("recording visual frames to {}", cfg.record_visual.as_ref().unwrap().display());
                record_notice = false;
            }
            recorded = r.frames();
        }
        // M8: rectify the raw (distorted, un-rectified) pair into the
        // canonical rectified rig the VO pipeline expects.
        let (left_r, right_r) = rectifier.apply(&frame.left, &frame.right);
        if let Some(pose) = vo.process(&left_r, &right_r) {
            // M8: express the visual pose in the head/IMU frame.
            // `imu_to_cam_left` maps IMU-frame points into the left camera
            // frame (`p_cam = imu_to_cam_left * p_imu`), so the camera pose
            // becomes a head pose via `world_T_head = world_T_cam * cam_T_imu`
            // with `cam_T_imu = imu_to_cam_left`. (Cross-checked against the
            // IMU YPR in M9.)
            let pose_head = match &imu_to_cam_left {
                Some(cam_t_imu) => pose * cam_t_imu,
                None => pose,
            };
            let q = pose_head.rotation.quaternion();
            let ypr_deg = neuromancer_ahrs::quat_to_ypr(neuromancer_ahrs::Quat::new(q.w, q.i, q.j, q.k))
                .map(f64::to_degrees);
            let out = Frame {
                t: frame.t,
                ypr_deg,
                position: [pose_head.translation.x, pose_head.translation.y, pose_head.translation.z],
            };
            for sink in sinks.iter_mut() {
                sink.write(&out);
            }
        }
        frames += 1;
        if frames == 1 {
            t_first = Some(frame.t);
        }
        if frames == 30 && !rate_reported {
            let fps = 29.0 / (frame.t - t_first.unwrap_or(frame.t)).max(1e-9);
            log_info!("visual pipeline running — measured frame rate {fps:.0} fps");
            rate_reported = true;
        }
    }
    if recorder.is_some() {
        log_info!("recorded {recorded} visual frames");
    }
    let run_secs = run_start.elapsed().as_secs_f64();
    if frames > 0 {
        log_info!(
            "visual pipeline sustained rate: {:.1} fps ({frames} frames in {run_secs:.1} s)",
            frames as f64 / run_secs.max(1e-9)
        );
    }
    drop(sinks);
    log_info!("tracker exited");
    ExitCode::SUCCESS
}

/// 6-DoF fused mode (`--input imu+visual`, P2b): two threads + shared clock.
///
/// - **IMU thread** (this thread): reads the live IMU at ~1 kHz and runs the
///   ESKF prediction, publishing the fused pose to the sinks at IMU rate.
/// - **Camera thread**: reads stereo frames at ~30 fps, runs the VO pipeline
///   (rectify → features → KLT → stereo → RANSAC, ~7–17 fps effective) and
///   applies each successful 6-DoF pose as an ESKF correction.
///
/// Both threads share the ESKF behind a mutex; timestamps come from a shared
/// wall-clock epoch so the fused output is on one timeline. Fusion is
/// live-first (P2b): replay/record modes are rejected by the CLI.
fn run_fusion(cfg: Config) -> ExitCode {
    use std::sync::{Arc, Mutex};
    use std::thread;

    // --- Both live sources (CLI cross-validation rejects replay/record) ----
    let mut imu = match ArDriversSource::open() {
        Ok(s) => s,
        Err(e) => {
            log_error!("error: {e}");
            return ExitCode::from(1);
        }
    };
    let mut cam = match SlamCameraSource::open() {
        Ok(s) => s,
        Err(e) => {
            log_error!("error: {e}");
            return ExitCode::from(1);
        }
    };

    // Shared clock epoch — both threads stamp their samples from this so the
    // fused timeline (IMU predictions + VO corrections) is consistent.
    // Created AFTER calibration so t=0 is the fused run start.
    let epoch = Instant::now();

    // --- Rectifier + head-frame transform (same wiring as run_visual) ------
    let (rectifier, imu_to_cam_left) = neuromancer_tracker::cam_calib::build_rectifier(true);
    let rig = rectifier.rig.clone();

    // --- ESKF (shared) ------------------------------------------------------
    let eskf = Arc::new(Mutex::new(Eskf::new(
        EskfConfig::real_scene(),
        Vector3::zeros(),
        UnitQuaternion::identity(),
        [0.1, 0.1, 0.1, 0.01, 0.01], // p, v, θ, bg, ba initial std-dev
    )));

    // --- Startup gyro-bias calibration (same as IMU mode, spec C.1) --------
    // The residual turn-on gyro bias (~0.25°/s measured) integrates into
    // orientation drift, which corrupts gravity compensation and blows up the
    // fused position. Calibrate BEFORE the camera thread starts (and before
    // any output) so the ESKF starts with a known bias; it continues refining
    // via VO corrections. This also drains the USB buffer, so the first rate
    // measurement reflects the real IMU rate.
    // Accumulate the mean specific force during the still window: it is
    // the gravity vector in the BODY frame (the accelerometer reads
    // specific force = −gravity at rest). Defaults to (0, 9.81, 0) when
    // gyro calibration is disabled.
    let mut acc_sum = [0.0f64; 3];
    let mut acc_n = 0u64;
    if cfg.gyro_calib > 0.0 {
        let mut calib = GyroCalibrator::new(cfg.gyro_calib);
        log_info!("gyro bias calibration: keep the device still for {:.1}s ...", cfg.gyro_calib);
        loop {
            let sample = match imu.next_sample() {
                Ok(s) => s,
                Err(e) => {
                    log_error!("error: IMU input failed: {e}");
                    return ExitCode::from(3);
                }
            };
            acc_sum[0] += sample.ax;
            acc_sum[1] += sample.ay;
            acc_sum[2] += sample.az;
            acc_n += 1;
            if let Some(res) = calib.push(&sample) {
                if res.complete {
                    log_info!(
                        "gyro bias calibrated: gx={:.6} gy={:.6} gz={:.6} rad/s ({} still samples)",
                        res.bias[0], res.bias[1], res.bias[2], res.samples
                    );
                } else {
                    log_warn!(
                        "gyro calibration incomplete — device moving? using {} samples",
                        res.samples
                    );
                }
                eskf.lock().unwrap().set_gyro_bias(Vector3::new(res.bias[0], res.bias[1], res.bias[2]));
                break;
            }
        }
    // Gravity in the filter world frame. The VO world is anchored to the
    // FIRST camera frame (identity world_T_cam), so the first head pose
    // equals `imu_to_cam_left` — the camera sits ~14.5° off the IMU frame
    // on the glasses, so the VO world is tilted relative to gravity.
    // At rest the accelerometer reads specific force f = a − g in the
    // body frame; with q rotating world→body, f = −q·g_w, so the gravity
    // in the VO world is g_w = −q_head0⁻¹·f (NOT −q_head0·f — the
    // missing inverse left ~2·g·sin(14.5°) of spurious acceleration).
    // This is only a coarse startup guess — the camera thread re-anchors
    // gravity after the first VO correction using the ESKF's own
    // converged orientation.
    let a_mean = if acc_n > 0 {
        Vector3::new(acc_sum[0] / acc_n as f64, acc_sum[1] / acc_n as f64, acc_sum[2] / acc_n as f64)
    } else {
        Vector3::new(0.0, 9.81, 0.0)
    };
    let g_w = match &imu_to_cam_left {
        Some(q_head0) => -(q_head0.rotation.inverse() * a_mean),
        None => Vector3::new(0.0, -9.81, 0.0),
    };
    log_info!(
        "gravity in VO world (startup guess): ({:.4}, {:.4}, {:.4}) m/s² (accel mean ({:.4}, {:.4}, {:.4}))",
        g_w.x, g_w.y, g_w.z, a_mean.x, a_mean.y, a_mean.z
    );
    eskf.lock().unwrap().set_gravity(g_w);
    if CTRL_C.load(Ordering::SeqCst) >= 1 {
        log_info!("SIGINT received — shutting down cleanly");
        return ExitCode::SUCCESS;
    }
}

    let device = format!("{} + {}", imu.name(), cam.name());
    let udp_out = if cfg.no_udp { "disabled".to_string() } else { format!("{}:{}", cfg.host, cfg.port) };
    let protocol = match cfg.protocol { cli::Protocol::Classic => "classic", cli::Protocol::Extended => "extended" };
    let units = match cfg.units { cli::Units::Deg => "deg", cli::Units::Rad => "rad" };
    println!(
        "device={device} input=imu+visual out={udp_out} protocol={protocol} units={units} udp_rate={}Hz hud={}",
        cfg.udp_rate,
        if cfg.hud { "on" } else { "off" },
    );

    // --- Camera/VO thread: frames → VO pose → ESKF correction --------------
    let eskf_cam = Arc::clone(&eskf);
    // Refined after the first correction (below) using the ESKF's own
    // converged orientation, which removes imu_to_cam_left inaccuracy.
    let a_mean = Vector3::new(acc_sum[0] / acc_n.max(1) as f64, acc_sum[1] / acc_n.max(1) as f64, acc_sum[2] / acc_n.max(1) as f64);
    let cam_handle = thread::spawn(move || {
        let mut vo = VoPipeline::new(rig, StereoMatcher::new(5, 0.5, 10.0));
        let mut frames: u64 = 0;
        let mut corr_frames: u64 = 0;
        let mut gravity_anchored = false;
        let mut t_first: Option<f64> = None;
        let mut rate_reported = false;
        let run_start = Instant::now();
        loop {
            if CTRL_C.load(Ordering::SeqCst) >= 1 {
                break;
            }
            let frame = match cam.next_frame() {
                Ok(f) => f,
                Err(e) => {
                    log_error!("error: visual input failed: {e}");
                    return 3;
                }
            };
            let (left_r, right_r) = rectifier.apply(&frame.left, &frame.right);
            if let Some(pose) = vo.process(&left_r, &right_r) {
                // world_T_head = world_T_cam · cam_T_imu — the ESKF body
                // frame is the IMU/head frame (same as M8 visual wiring).
                let pose_head = match &imu_to_cam_left {
                    Some(cam_t_imu) => pose * cam_t_imu,
                    None => pose,
                };
                if let Ok(mut e) = eskf_cam.lock() {
                    let vo_q = pose_head.rotation.quaternion();
                    let vo_ypr = quat_to_ypr(neuromancer_ahrs::Quat::new(vo_q.w, vo_q.i, vo_q.j, vo_q.k)).map(f64::to_degrees);
                    let es_q = e.orientation().quaternion();
                    let es_ypr = quat_to_ypr(neuromancer_ahrs::Quat::new(es_q.w, es_q.i, es_q.j, es_q.k)).map(f64::to_degrees);
                    // Raw innovation before the correction (position + body-frame angle).
                    let dtheta = (e.orientation().conjugate() * pose_head.rotation).angle().to_degrees();
                    let dp = (e.position() - pose_head.translation.vector).norm();
                    e.correct(&pose_head.translation.vector, &pose_head.rotation);
                    corr_frames += 1;
                    if corr_frames <= 6 || corr_frames.is_multiple_of(20) {
                        log_info!(
                            "[fusion] corr#{corr_frames} vo_ypr=({:.1},{:.1},{:.1}) es_ypr=({:.1},{:.1},{:.1}) |dz_theta|={dtheta:.1}° |dz_p|={dp:.3}m bg=({:.4},{:.4},{:.4}) ba=({:.3},{:.3},{:.3})",
                            vo_ypr[0], vo_ypr[1], vo_ypr[2],
                            es_ypr[0], es_ypr[1], es_ypr[2],
                            e.gyro_bias().x, e.gyro_bias().y, e.gyro_bias().z,
                            e.accel_bias().x, e.accel_bias().y, e.accel_bias().z,
                        );
                    }
                    // After the FIRST correction we know the head orientation
                    // in the VO world (= pose_head.rotation = q_head0), and
                    // the calibration window gave the body-frame specific
                    // force at rest. Anchor gravity in the filter world:
                    // at rest, specific force f = −q_head0·g_w (q maps
                    // world→body), so g_w = −q_head0⁻¹·f. Anchor from the VO
                    // MEASUREMENT, NOT the filter state: the first
                    // correction only partially converges (Kalman gain < 1),
                    // so e.orientation() would give a wrong q and freeze
                    // that error into g_w forever.
                    if !gravity_anchored {
                        let g_w = -(pose_head.rotation.inverse() * a_mean);
                        e.set_gravity(g_w);
                        log_info!(
                            "gravity anchored: ({:.4}, {:.4}, {:.4}) m/s² after first correction",
                            g_w.x, g_w.y, g_w.z
                        );
                        gravity_anchored = true;
                    }
                }
            }
            frames += 1;
            if frames == 1 {
                t_first = Some(frame.t);
            }
            if frames == 30 && !rate_reported {
                let fps = 29.0 / (frame.t - t_first.unwrap_or(frame.t)).max(1e-9);
                log_info!("visual pipeline running — measured frame rate {fps:.0} fps");
                rate_reported = true;
            }
        }
        let run_secs = run_start.elapsed().as_secs_f64();
        log_info!(
            "fusion camera thread: {frames} frames in {run_secs:.1}s ({:.0} fps), {corr_frames} corrections",
            frames as f64 / run_secs.max(1e-9)
        );
        0
    });

    // --- IMU thread (this thread): samples → ESKF predict → sinks ----------
    install_signal_handler();
    let (mut sinks, _imu_log) = build_sinks(&cfg);
    let mut prev_t: Option<f64> = None;
    let mut n_samples: u64 = 0;
    let mut t_first: Option<f64> = None;
    let mut rate_reported = false;
    let mut exit: u8 = 0;

    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            break;
        }
        let sample = match imu.next_sample() {
            Ok(s) => s,
            Err(e) => {
                if CTRL_C.load(Ordering::SeqCst) >= 1 {
                    break;
                }
                log_error!("error: IMU input failed: {e}");
                log_error!("(device unplugged — restart the tracker to reconnect; exit code 3)");
                exit = 3;
                break;
            }
        };
        // Shared-clock timestamp (wall epoch), not the per-source t.
        let t = epoch.elapsed().as_secs_f64();
        let dt = match prev_t {
            Some(pt) => (t - pt).clamp(0.0, MAX_DT),
            None => 0.0,
        };
        prev_t = Some(t);

        // ESKF prediction: gyro/accel in body (RUB) frame, m/s² and rad/s.
        let gyro = Vector3::new(sample.gx, sample.gy, sample.gz);
        let accel = Vector3::new(sample.ax, sample.ay, sample.az);
        let pose = match eskf.lock() {
            Ok(mut e) => {
                let pose = e.predict(dt, &gyro, &accel);
                // Level reference at IMU rate (Mahony-style gravity
                // correction): pins roll/pitch so the sparse VO orientation
                // is not the only thing keeping the filter level.
                e.correct_gravity(&accel, 0.2);
                pose
            }
            Err(_) => break,
        };
        let q = pose.rotation.quaternion();
        let ypr_deg = quat_to_ypr(neuromancer_ahrs::Quat::new(q.w, q.i, q.j, q.k)).map(f64::to_degrees);
        let out = Frame {
            t,
            ypr_deg,
            position: [pose.translation.x, pose.translation.y, pose.translation.z],
        };
        for sink in sinks.iter_mut() {
            sink.write(&out);
        }

        n_samples += 1;
        if n_samples == 1 {
            t_first = Some(t);
        }
        if !rate_reported && n_samples == 30 {
            let rate = 29.0 / (t - t_first.unwrap_or(t)).max(1e-9);
            log_info!("measured imu_rate={rate:.0} Hz");
            rate_reported = true;
        }
    }

    // --- Clean shutdown ----------------------------------------------------
    let cam_exit = cam_handle.join().unwrap_or(3);
    drop(sinks);
    log_info!("tracker exited");
    if cam_exit != 0 {
        ExitCode::from(cam_exit)
    } else {
        ExitCode::from(exit)
    }
}
