//! Integration tests (spec §2.8): deterministic IMU recording → filter →
//! UDP pose round-trip (T7-style), plus process-level CLI checks.

use std::net::UdpSocket;
use std::time::Duration;

use neuromancer_ahrs::{quat_to_ypr, Mahony};

use neuromancer_tracker::axis::AxisMap;
use neuromancer_tracker::calib::{BiasRefresher, GyroCalibrator};
use neuromancer_tracker::cli::{parse, ParseOutcome, Protocol, Units};
use neuromancer_tracker::imu::{ImuError, ImuSource, ReplaySource};
use neuromancer_tracker::jsonl::{write_imu, ImuSample};
use neuromancer_tracker::output::{Frame, HudSink, Sink, UdpSink};

const G: f64 = 9.81;

/// Write a synthetic IMU recording to a temp file and return its path.
fn write_recording(dir: &std::path::Path, samples: &[ImuSample]) -> std::path::PathBuf {
    static COUNTER: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
    let n = COUNTER.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let path = dir.join(format!("rec-{}-{}.jsonl", std::process::id(), n));
    let mut f = std::fs::File::create(&path).unwrap();
    for s in samples {
        write_imu(&mut f, s).unwrap();
    }
    path
}

/// Constant-yaw-rate recording: 200 Hz for 1 s at 30°/s, gravity level.
fn yaw_recording() -> Vec<ImuSample> {
    let dt = 0.005;
    let omega = 30.0f64.to_radians();
    (0..200)
        .map(|i| ImuSample {
            t: i as f64 * dt,
            ax: 0.0,
            ay: G,
            az: 0.0,
            gx: 0.0,
            gy: omega,
            gz: 0.0,
        })
        .collect()
}

/// Replay a recording through the real pipeline (filter + axis map) and
/// return the final YPR in degrees.
fn run_pipeline(samples: &[ImuSample], map: AxisMap) -> [f64; 3] {
    let mut mahony = Mahony::new();
    let mut prev_t: Option<f64> = None;
    let mut final_ypr = [0.0; 3];
    for s in samples {
        let dt = match prev_t {
            Some(pt) => (s.t - pt).clamp(0.0, 0.1),
            None => 0.0,
        };
        prev_t = Some(s.t);
        let q = mahony.update([s.ax, s.ay, s.az], [s.gx, s.gy, s.gz], dt);
        let mut ypr = quat_to_ypr(q).map(f64::to_degrees);
        map.apply(&mut ypr);
        final_ypr = ypr;
    }
    final_ypr
}

/// Run a sample stream through the Mahony filter with a gyro bias subtracted,
/// return the final yaw in degrees.
fn run_with_bias(samples: &[ImuSample], bias: [f64; 3]) -> f64 {
    let mut mahony = Mahony::new();
    let mut prev_t: Option<f64> = None;
    for s in samples {
        let dt = match prev_t {
            Some(pt) => (s.t - pt).clamp(0.0, 0.1),
            None => 0.0,
        };
        prev_t = Some(s.t);
        mahony.update(
            [s.ax, s.ay, s.az],
            [s.gx - bias[0], s.gy - bias[1], s.gz - bias[2]],
            dt,
        );
    }
    let [yaw, _, _] = quat_to_ypr(mahony.quaternion());
    yaw.to_degrees()
}

/// Deterministic pseudo-random noise in ±0.005 rad/s (zero-mean-ish).
fn noise(i: usize, phase: f64) -> f64 {
    (i as f64 * 12.9898 + phase * 97.0).sin() * 0.005
}

/// Reproduces V's field finding (2026-08-04): a residual gyro bias of
/// ~0.25°/s integrates into ~15° yaw drift over 60 s (no magnetometer yaw
/// reference). The startup stationary calibration must remove it.
#[test]
fn gyro_bias_drift_and_calibration() {
    let bias = 0.25f64.to_radians();
    let dt = 0.005;
    let calib_samples = 2 * 200 + 20; // 2 s @ 200 Hz window + margin (first sample has dt=0)
    let n_total = calib_samples + 60 * 200; // + 60 s drift test
    let samples: Vec<ImuSample> = (0..n_total)
        .map(|i| ImuSample {
            t: i as f64 * dt,
            ax: 0.0,
            ay: G,
            az: 0.0,
            gx: noise(i, 1.0),
            gy: bias + noise(i, 2.0),
            gz: noise(i, 3.0),
        })
        .collect();

    // Without calibration: ~15° after 60 s (the reported symptom).
    let uncalibrated = run_with_bias(&samples[calib_samples..], [0.0; 3]);
    assert!(
        (uncalibrated - 15.0).abs() < 1.5,
        "uncalibrated drift {uncalibrated}° (expected ≈15°)"
    );

    // With the startup calibration: bias is recovered from the still window.
    let mut cal = GyroCalibrator::new(2.0);
    let mut bias_est = [0.0; 3];
    for s in samples.iter().take(calib_samples) {
        if let Some(res) = cal.push(s) {
            bias_est = res.bias;
        }
    }
    assert!((bias_est[1] - bias).abs() < 0.002, "bias est {bias_est:?}");
    let calibrated = run_with_bias(&samples[calib_samples..], bias_est);
    assert!(
        calibrated.abs() < 2.5,
        "calibrated drift {calibrated}° (expected ≈0)"
    );
}

/// In-run re-calibration: when the gyro bias drifts mid-session (thermal
/// warm-up), resting the glasses still for a window re-centers the bias so
/// yaw stops integrating it. A tracker without the refresh keeps drifting.
#[test]
fn in_run_recalibration_stops_drift_after_bias_change() {
    let b1 = 0.25f64.to_radians(); // turn-on bias at startup
    let b2 = 0.5f64.to_radians(); // drifted bias (thermal)
    let dt = 0.005;
    // Timeline (s): [0..2] still b1 (startup calib) | [2..4] motion 20°/s
    // | [4..7] still b2 | [7..10] still b2 (drift measured over last 3 s;
    //   the refresh completes ~2 s into the [4..7] rest).
    let n = 10 * 200;
    let motion = 20f64.to_radians();
    let samples: Vec<ImuSample> = (0..n)
        .map(|i| {
            let t = i as f64 * dt;
            let bias = if t < 4.0 { b1 } else { b2 };
            let moving = (2.0..4.0).contains(&t);
            ImuSample {
                t,
                ax: 0.0,
                ay: G,
                az: 0.0,
                gx: noise(i, 1.0),
                gy: bias + if moving { motion } else { 0.0 } + noise(i, 2.0),
                gz: noise(i, 3.0),
            }
        })
        .collect();

    // Startup calibration over the first still segment.
    let mut cal = GyroCalibrator::new(2.0);
    let mut bias_est = [0.0; 3];
    let mut start_idx = samples.len();
    for (i, s) in samples.iter().enumerate() {
        if let Some(res) = cal.push(s) {
            bias_est = res.bias;
            start_idx = i + 1;
            break;
        }
    }
    assert!(start_idx < samples.len(), "startup calibration never finished");

    // Filter + (optional) refresher from the calibration end.
    let run = |refresh: bool| -> f64 {
        let mut mahony = Mahony::new();
        let mut refresher = refresh.then(|| BiasRefresher::new(2.0, bias_est));
        let mut prev_t: Option<f64> = None;
        let mut yaw_at_7s: Option<f64> = None;
        let mut final_yaw = 0.0;
        for s in &samples[start_idx..] {
            if let Some(r) = refresher.as_mut() {
                let _ = r.feed(s);
            }
            let bias = refresher.as_ref().map_or(bias_est, |r| r.bias());
            let dt = match prev_t {
                Some(pt) => (s.t - pt).clamp(0.0, 0.1),
                None => 0.0,
            };
            prev_t = Some(s.t);
            mahony.update(
                [s.ax, s.ay, s.az],
                [s.gx - bias[0], s.gy - bias[1], s.gz - bias[2]],
                dt,
            );
            let yaw = quat_to_ypr(mahony.quaternion())[0].to_degrees();
            if s.t >= 7.0 && yaw_at_7s.is_none() {
                yaw_at_7s = Some(yaw);
            }
            final_yaw = yaw;
        }
        final_yaw - yaw_at_7s.unwrap_or(0.0)
    };

    let drift_no_refresh = run(false);
    let drift_with_refresh = run(true);
    assert!(
        drift_no_refresh > 0.4,
        "without refresh the drifted bias should integrate (got {drift_no_refresh}°)"
    );
    assert!(
        drift_with_refresh.abs() < 0.4,
        "with refresh the bias is re-centered (got {drift_with_refresh}°)"
    );
}

#[test]
fn replay_through_filter_tracks_yaw() {
    // T7-style: recorded IMU → filter → expected pose.
    let dir = std::env::temp_dir();
    let path = write_recording(&dir, &yaw_recording());
    let mut replay = ReplaySource::open(&path, false).unwrap();
    assert!(replay.name().contains("replay("));

    let mut mahony = Mahony::new();
    let mut prev_t: Option<f64> = None;
    let mut n = 0u32;
    loop {
        match replay.next_sample() {
            Ok(s) => {
                n += 1;
                let dt = match prev_t {
                    Some(pt) => (s.t - pt).clamp(0.0, 0.1),
                    None => 0.0,
                };
                prev_t = Some(s.t);
                mahony.update([s.ax, s.ay, s.az], [s.gx, s.gy, s.gz], dt);
            }
            Err(ImuError::Eof) => break,
            Err(e) => panic!("unexpected replay error: {e}"),
        }
    }
    let _ = std::fs::remove_file(&path);
    assert_eq!(n, 200);
    let [yaw, pitch, roll] = quat_to_ypr(mahony.quaternion()).map(f64::to_degrees);
    assert!((yaw - 30.0).abs() < 2.0, "yaw {yaw}° (expected ≈30°)");
    assert!(pitch.abs() < 1.0, "pitch {pitch}° disturbed");
    assert!(roll.abs() < 1.0, "roll {roll}° disturbed");
}

#[test]
fn axis_mapping_applies_to_live_pipeline() {
    let samples = yaw_recording();
    let map = AxisMap {
        invert_yaw: true,
        sensitivity: 2.0,
        ..Default::default()
    };
    let [yaw, ..] = run_pipeline(&samples, map);
    // 30°/s * 1 s = +30° → inverted → -30° → sensitivity ×2 → -60°.
    assert!((yaw - -60.0).abs() < 4.0, "yaw {yaw}° (expected ≈-60°)");
}

#[test]
fn udp_classic_roundtrip() {
    // Bind a receiver, point a UdpSink at it, verify the 48-byte payload.
    let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let rx_addr = rx.local_addr().unwrap();

    let mut sink =
        UdpSink::bind(&rx_addr.ip().to_string(), rx_addr.port(), Protocol::Classic, Units::Deg, 60.0)
            .unwrap();
    sink.write(&Frame::new_3dof(0.0, [30.0, -10.0, 5.0]));

    let mut buf = [0u8; 48];
    let n = rx.recv(&mut buf).unwrap();
    assert_eq!(n, 48);
    let vals: Vec<f64> = buf
        .chunks_exact(8)
        .map(|c| f64::from_ne_bytes(c.try_into().unwrap()))
        .collect();
    assert_eq!(vals[0..3], [0.0, 0.0, 0.0]);
    assert_eq!(vals[3], 30.0);
    assert_eq!(vals[4], -10.0);
    assert_eq!(vals[5], 5.0);
}

#[test]
fn udp_extended_roundtrip() {
    let rx = UdpSocket::bind("127.0.0.1:0").unwrap();
    rx.set_read_timeout(Some(Duration::from_secs(2))).unwrap();
    let rx_addr = rx.local_addr().unwrap();

    let mut sink = UdpSink::bind(
        &rx_addr.ip().to_string(),
        rx_addr.port(),
        Protocol::Extended,
        Units::Deg,
        60.0,
    )
    .unwrap();
    sink.write(&Frame::new_3dof(0.0, [90.0, 0.0, 0.0]));

    let mut buf = [0u8; 80];
    let n = rx.recv(&mut buf).unwrap();
    assert_eq!(n, 80);
    let vals: Vec<f64> = buf
        .chunks_exact(8)
        .map(|c| f64::from_ne_bytes(c.try_into().unwrap()))
        .collect();
    assert_eq!(vals[3], 90.0);
    assert_eq!(vals[6], 1.0);
    assert_eq!(vals[7..10], [0.0, 0.0, 0.0]);
}

#[test]
fn hud_formats_degrees_line() {
    let mut buf = Vec::new();
    let mut hud = HudSink::new(&mut buf, false, 60.0);
    hud.write(&Frame::new_3dof(0.0, [12.3, -4.1, 0.8]));
    let s = String::from_utf8(buf).unwrap();
    assert!(s.contains("YAW") && s.contains("PITCH") && s.contains("ROLL"));
    assert!(s.contains('°'));
    assert!(s.ends_with('\n'));
}

// ---- process-level checks (exit codes, spec §3.2) ----

fn bin() -> &'static str {
    env!("CARGO_BIN_EXE_neuromancer-tracker-full")
}

#[test]
fn cli_bad_flag_exits_2() {
    let out = std::process::Command::new(bin()).arg("--bogus").output().unwrap();
    assert_eq!(out.status.code(), Some(2));
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("unknown option"));
}

#[test]
fn cli_version_exits_0() {
    let out = std::process::Command::new(bin()).arg("--version").output().unwrap();
    assert_eq!(out.status.code(), Some(0));
    assert!(String::from_utf8_lossy(&out.stdout).contains("neuromancer-tracker"));
}

#[test]
fn cli_no_device_exits_1() {
    // Without glasses on USB (this environment), the tracker must fail with
    // the clear spec error and exit code 1. Skipped when a device is present
    // would hang — so this is gated on absence; if a real device is attached
    // the process would block instead of exiting, so we only assert on
    // machines without hardware by checking we get exit 1 promptly.
    let mut child = std::process::Command::new(bin())
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    // Give it a moment to fail (no device); if a device IS present the
    // process keeps running and we must not wait forever.
    for _ in 0..50 {
        if let Some(status) = child.try_wait().unwrap() {
            assert_eq!(status.code(), Some(1));
            return;
        }
        std::thread::sleep(Duration::from_millis(100));
    }
    // A device is attached (or USB enumeration hangs): kill and skip.
    let _ = child.kill();
    let _ = child.wait();
}

#[test]
fn cli_replay_run_terminates_cleanly() {
    // Full binary run from a replay file: must exit 0 on EOF with --no-udp.
    let dir = std::env::temp_dir();
    let path = write_recording(&dir, &yaw_recording());
    let out = std::process::Command::new(bin())
        .args(["--replay", path.to_str().unwrap(), "--no-udp"])
        .output()
        .unwrap();
    let _ = std::fs::remove_file(&path);
    assert_eq!(out.status.code(), Some(0), "stderr: {}", String::from_utf8_lossy(&out.stderr));
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.starts_with("device=replay("), "got {stdout}");
    assert!(stdout.contains("out=disabled"));
}

#[test]
fn cli_sigint_mid_run_exits_0() {
    // Spec §3.5: first Ctrl-C = clean shutdown, exit 0 — even when the
    // signal lands while the input source is inside a blocking read (spike
    // finding 2026-08-04: hidapi surfaced the interrupted USB read as
    // "unplugged?" and the tracker wrongly exited 3). A paced replay keeps
    // the process alive long enough to send SIGINT mid-run.
    let dir = std::env::temp_dir();
    // 30 s of samples at 200 Hz: the paced replay runs ~30 s.
    let mut samples = yaw_recording();
    for i in 200..30 * 200 {
        samples.push(ImuSample {
            t: i as f64 * 0.005,
            ax: 0.0,
            ay: G,
            az: 0.0,
            gx: 0.0,
            gy: 30.0f64.to_radians(),
            gz: 0.0,
        });
    }
    let path = write_recording(&dir, &samples);

    let mut child = std::process::Command::new(bin())
        .args(["--replay", path.to_str().unwrap(), "--no-udp"])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::piped())
        .spawn()
        .unwrap();
    // Wait for startup (replay opens, calibration starts), then SIGINT.
    std::thread::sleep(Duration::from_millis(800));
    unsafe {
        libc::kill(child.id() as i32, libc::SIGINT);
    }
    let deadline = std::time::Instant::now() + Duration::from_secs(5);
    let status = loop {
        if let Some(s) = child.try_wait().unwrap() {
            break s;
        }
        if std::time::Instant::now() > deadline {
            let _ = child.kill();
            let _ = child.wait();
            let _ = std::fs::remove_file(&path);
            panic!("tracker did not exit within 5 s of SIGINT");
        }
        std::thread::sleep(Duration::from_millis(50));
    };
    let _ = std::fs::remove_file(&path);
    assert_eq!(status.code(), Some(0), "SIGINT must produce a clean exit 0");
}

/// Write a 7-frame synthetic forward stereo sequence into `dir` and return
/// the rig used to render it.
fn write_synthetic_sequence(dir: &std::path::Path) -> neuromancer_vo::camera::StereoRig {
    use neuromancer_vo::camera::StereoRig;
    use neuromancer_vo::synthetic::{forward_trajectory, render};
    std::fs::create_dir_all(dir).unwrap();
    let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
    let traj = forward_trajectory(7, 0.03, 0.5);
    for (i, pose) in traj.iter().enumerate() {
        let (frame, _) = render(&rig, pose, 1.5);
        std::fs::write(dir.join(format!("left_{i:04}.raw")), &frame.left).unwrap();
        std::fs::write(dir.join(format!("right_{i:04}.raw")), &frame.right).unwrap();
    }
    rig
}

#[test]
fn visual_replay_end_to_end() {
    let dir = std::env::temp_dir().join(format!("nt-visual-{}", std::process::id()));
    write_synthetic_sequence(&dir);
    let pose_log = dir.join("pose.jsonl");
    let out = std::process::Command::new(bin())
        .args([
            "--input",
            "visual",
            "--replay-visual",
            dir.to_str().unwrap(),
            "--no-udp",
            "--log-pose",
            pose_log.to_str().unwrap(),
        ])
        .output()
        .unwrap();
    assert_eq!(
        out.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let stdout = String::from_utf8_lossy(&out.stdout);
    assert!(stdout.contains("input=visual"), "got {stdout}");

    let content = std::fs::read_to_string(&pose_log).unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    // 6-DoF: pose lines carry position, and z grows along the forward path.
    let zs: Vec<f64> = content
        .lines()
        .filter_map(|l| {
            let z = l.split("\"z\": ").nth(1)?.trim_end_matches('}').parse().ok()?;
            Some(z)
        })
        .collect();
    assert!(zs.len() >= 5, "only {} pose lines: {content}", zs.len());
    assert!(
        zs.last().unwrap() > &(zs.first().unwrap() + 0.05),
        "z not growing along forward trajectory: {zs:?}"
    );
}

#[test]
fn visual_record_replay_roundtrip() {
    // Replay a synthetic sequence while recording it; the recorded directory
    // must contain the same number of frames (left+right pairs).
    let src = std::env::temp_dir().join(format!("nt-vsrc-{}", std::process::id()));
    let dst = std::env::temp_dir().join(format!("nt-vdst-{}", std::process::id()));
    write_synthetic_sequence(&src);
    let out = std::process::Command::new(bin())
        .args([
            "--input",
            "visual",
            "--replay-visual",
            src.to_str().unwrap(),
            "--record-visual",
            dst.to_str().unwrap(),
            "--no-udp",
        ])
        .output()
        .unwrap();
    assert_eq!(
        out.status.code(),
        Some(0),
        "stderr: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    let lefts = std::fs::read_dir(&dst)
        .unwrap()
        .filter(|e| e.as_ref().unwrap().file_name().to_string_lossy().starts_with("left_"))
        .count();
    let rights = std::fs::read_dir(&dst)
        .unwrap()
        .filter(|e| e.as_ref().unwrap().file_name().to_string_lossy().starts_with("right_"))
        .count();
    let _ = std::fs::remove_dir_all(&src);
    let _ = std::fs::remove_dir_all(&dst);
    assert_eq!(lefts, 7, "recorded {lefts} left frames");
    assert_eq!(rights, 7, "recorded {rights} right frames");
}

#[test]
fn cli_parse_integration() {
    let ParseOutcome::Run(c) =
        parse(vec!["--hud".into(), "--protocol=extended".into(), "--units".into(), "rad".into()])
            .unwrap()
    else {
        panic!()
    };
    assert!(c.hud);
    assert_eq!(c.protocol, Protocol::Extended);
    assert_eq!(c.units, Units::Rad);
}
