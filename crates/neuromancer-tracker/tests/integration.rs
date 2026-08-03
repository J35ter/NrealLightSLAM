//! Integration tests (spec §2.8): deterministic IMU recording → filter →
//! UDP pose round-trip (T7-style), plus process-level CLI checks.

use std::net::UdpSocket;
use std::time::Duration;

use neuromancer_ahrs::{quat_to_ypr, Mahony};

use neuromancer_tracker::axis::AxisMap;
use neuromancer_tracker::calib::GyroCalibrator;
use neuromancer_tracker::cli::{parse, ParseOutcome, Protocol, Units};
use neuromancer_tracker::imu::{ImuError, ImuSource, ReplaySource};
use neuromancer_tracker::jsonl::{write_imu, ImuSample};
use neuromancer_tracker::output::{Frame, HudSink, Sink, UdpSink};

const G: f64 = 9.81;

/// Write a synthetic IMU recording to a temp file and return its path.
fn write_recording(dir: &std::path::Path, samples: &[ImuSample]) -> std::path::PathBuf {
    let path = dir.join(format!("rec-{}.jsonl", std::process::id()));
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
    sink.write(&Frame {
        t: 0.0,
        ypr_deg: [30.0, -10.0, 5.0],
    });

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
    sink.write(&Frame {
        t: 0.0,
        ypr_deg: [90.0, 0.0, 0.0],
    });

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
    hud.write(&Frame {
        t: 0.0,
        ypr_deg: [12.3, -4.1, 0.8],
    });
    let s = String::from_utf8(buf).unwrap();
    assert!(s.contains("YAW") && s.contains("PITCH") && s.contains("ROLL"));
    assert!(s.contains('°'));
    assert!(s.ends_with('\n'));
}

// ---- process-level checks (exit codes, spec §3.2) ----

fn bin() -> &'static str {
    env!("CARGO_BIN_EXE_neuromancer-tracker")
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
