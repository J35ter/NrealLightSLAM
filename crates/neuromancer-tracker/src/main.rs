//! `neuromancer-tracker` — Nreal Light head tracker (Phase 1: 3-DoF).
//!
//! Data flow (spec §1.6): USB IMU → Mahony AHRS → axis mapping →
//! Opentrack UDP (≥ 60 Hz, rate-gated) + 2 Hz HUD + optional logs.
//! Single-threaded event loop, no async runtime (spec §2.7).

use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};

use neuromancer_ahrs::{quat_to_ypr, Mahony};
use neuromancer_tracker::axis::AxisMap;
use neuromancer_tracker::cli::{self, Config, ParseOutcome};
use neuromancer_tracker::imu::{ArDriversSource, ImuError, ImuSource, ReplaySource};
use neuromancer_tracker::output::{Frame, HudSink, ImuLogSink, PoseLogSink, Sink, UdpSink};

/// Ctrl-C press counter: 1 = graceful shutdown requested, ≥2 = force exit.
static CTRL_C: AtomicUsize = AtomicUsize::new(0);

/// Cap on the filter `dt` (seconds): protects the gyro integration from a
/// huge jump after a USB stall or replay gap (spec §3.3: robust to jitter).
const MAX_DT: f64 = 0.1;

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match cli::parse(args) {
        Ok(ParseOutcome::Run(c)) => c,
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
                eprintln!("error: {e}");
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
    let mut sinks: Vec<Box<dyn Sink>> = Vec::new();
    if !cfg.no_udp {
        match UdpSink::bind(&cfg.host, cfg.port, cfg.protocol, cfg.units, cfg.udp_rate) {
            Ok(s) => sinks.push(Box::new(s)),
            Err(e) => eprintln!(
                "warning: UDP output disabled — cannot bind sender to {}:{}: {e}",
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
            Err(e) => eprintln!(
                "warning: pose log disabled — cannot open {}: {e}",
                path.display()
            ),
        }
    }
    let mut imu_log = match &cfg.log_imu {
        Some(path) => match ImuLogSink::open(path) {
            Ok(s) => Some(s),
            Err(e) => {
                eprintln!("warning: IMU log disabled — cannot open {}: {e}", path.display());
                None
            }
        },
        None => None,
    };

    // --- Signal handling (spec §3.5) ---------------------------------------
    if let Err(e) = ctrlc::set_handler(|| {
        let n = CTRL_C.fetch_add(1, Ordering::SeqCst) + 1;
        if n >= 2 {
            // Second Ctrl-C: immediate exit 1.
            std::process::exit(1);
        }
    }) {
        eprintln!("warning: cannot install Ctrl-C handler: {e}");
    }

    // --- Startup confirmation line (spec §3.2) -----------------------------
    let udp_out = if cfg.no_udp {
        "disabled".to_string()
    } else {
        format!("{}:{}", cfg.host, cfg.port)
    };
    println!(
        "device={} kp={} ki={} out={} protocol={} units={} udp_rate={}Hz hud={}",
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
    );

    // --- Main loop (spec §3.3) ---------------------------------------------
    let mut prev_t: Option<f64> = None;
    let mut t_first: Option<f64> = None;
    let mut n_samples: u64 = 0;
    let mut rate_reported = false;

    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            eprintln!("SIGINT received — shutting down cleanly");
            break;
        }

        let sample = match source.next_sample() {
            Ok(s) => s,
            Err(ImuError::Eof) => {
                eprintln!("replay input exhausted — shutting down cleanly");
                break;
            }
            Err(e) => {
                eprintln!("error: IMU input failed: {e}");
                eprintln!("(device unplugged — restart the tracker to reconnect; exit code 3)");
                return ExitCode::from(3);
            }
        };

        // dt from sample arrival timestamps (monotonic), clamped (spec §3.3).
        let dt = match prev_t {
            Some(pt) => (sample.t - pt).clamp(0.0, MAX_DT),
            None => 0.0,
        };
        prev_t = Some(sample.t);

        if let Some(log) = &mut imu_log {
            log.write_sample(&sample);
        }

        // Filter consumes every sample (no input downsampling).
        let q = mahony.update([sample.ax, sample.ay, sample.az], [sample.gx, sample.gy, sample.gz], dt);
        let mut ypr_deg = quat_to_ypr(q).map(f64::to_degrees);
        axis_map.apply(&mut ypr_deg);

        let frame = Frame {
            t: sample.t,
            ypr_deg,
        };
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
            eprintln!("info: measured imu_rate={rate:.0} Hz");
            rate_reported = true;
        }
    }

    // --- Clean shutdown (flush logs via Drop, exit 0) ----------------------
    drop(imu_log);
    drop(sinks);
    eprintln!("tracker exited");
    ExitCode::SUCCESS
}
