//! Minimal IMU-only head tracker CLI (Phase 1, 3-DoF) — the main tracker.
//!
//! Thin CLI over the shared core in `lib.rs` (`Settings`, `ImuTracker`,
//! `UdpSink`): USB IMU → startup gyro-bias calibration → Mahony → YXZ
//! Tait-Bryan yaw/pitch/roll → 2 Hz HUD + optional Opentrack UDP (classic
//! 48-byte 6×f64) at 60 Hz.
//!
//! Usage:
//!   cargo run --release -p neuromancer-tracker
//!   cargo run --release -p neuromancer-tracker -- --no-udp
//!   cargo run --release -p neuromancer-tracker -- --gyro-calib 2 --hud-rate 5
//!
//! Flags (each maps to a `Settings` field, same as the GUI menu):
//!   --no-udp            disable UDP output (HUD only; UDP defaults on)
//!   --host HOST         UDP destination (default 127.0.0.1)
//!   --port PORT         UDP port (default 4242)
//!   --udp-rate HZ       UDP send rate (default 60)
//!   --hud-rate HZ       HUD refresh rate (default 2; 0 = off)
//!   --gyro-calib SECS   startup bias window (default 2.0; 0 = skip)
//!   --kp F, --ki F      Mahony gains (default 1.0 / 0.005)
//!   --units deg|rad     output units (default deg)
//!   --config PATH       settings file to load (default: platform config dir)
//!   --save-config       write the current (post-flag) settings to the config
//!                       file and exit
//!
//! `--config`/`--save-config` let the CLI interoperate with the GUI's TOML
//! settings file: the GUI saves there, and the CLI can read the same file.

use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Instant;

use neuromancer_tracker::{ImuTracker, Pose, Settings, UdpSink};

/// SIGINT flag: 0 = run, >= 1 = shutdown. Set by the libc handler (unix).
/// On Windows the console's own Ctrl+C handling terminates the process, so
/// the flag stays 0 and the loop runs until then.
static CTRL_C: AtomicUsize = AtomicUsize::new(0);

#[cfg(unix)]
extern "C" fn on_sigint(_: libc::c_int) {
    CTRL_C.store(1, Ordering::SeqCst);
}

#[cfg(unix)]
fn install_signal_handler() {
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigint as *const () as libc::sighandler_t;
        libc::sigemptyset(&mut sa.sa_mask);
        sa.sa_flags = 0; // no SA_RESTART: interrupt the blocking USB read
        libc::sigaction(libc::SIGINT, &sa, std::ptr::null_mut());
    }
}

// Windows: no custom handler — console Ctrl+C terminates the process, and
// the blocking USB read is interrupted by process teardown.
#[cfg(windows)]
fn install_signal_handler() {}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();

    // Settings: start from the config file (if any), then overlay CLI flags.
    let mut settings = Settings::load();
    let mut save_config = false;
    let mut it = args.iter().peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--config" => match it.next() {
                Some(p) => {
                    match std::fs::read_to_string(p) {
                        Ok(text) => match toml::from_str(&text) {
                            Ok(s) => settings = s,
                            Err(e) => {
                                eprintln!("warning: config {p} unreadable ({e}) — using defaults");
                                settings = Settings::default();
                            }
                        },
                        Err(_) => eprintln!("warning: config {p} not found — using defaults"),
                    }
                }
                None => {
                    eprintln!("error: --config needs a value");
                    return ExitCode::from(2);
                }
            },
            "--save-config" => save_config = true,
            other => {
                let value: Option<&str> = if it.peek().map(|v| !v.starts_with("--")).unwrap_or(false) {
                    it.next().map(|s| s.as_str())
                } else {
                    None
                };
                match settings.apply_cli(other, value) {
                    Ok(true) => {}
                    Ok(false) => {
                        eprintln!("error: unknown flag: {other}");
                        eprintln!("usage: neuromancer-tracker [--no-udp] [--host H] [--port P] [--udp-rate HZ] [--hud-rate HZ] [--gyro-calib SECS] [--kp F] [--ki F] [--units deg|rad] [--config PATH] [--save-config]");
                        return ExitCode::from(2);
                    }
                    Err(e) => {
                        eprintln!("error: {e}");
                        return ExitCode::from(2);
                    }
                }
            }
        }
    }

    if save_config {
        match settings.save() {
            Ok(path) => {
                println!("settings written to {}", path.display());
                return ExitCode::SUCCESS;
            }
            Err(e) => {
                eprintln!("error: {e}");
                return ExitCode::from(1);
            }
        }
    }

    // --- Open the USB IMU -------------------------------------------------
    let mut tracker = match ImuTracker::open() {
        Ok(t) => t,
        Err(e) => {
            eprintln!("error: {e}");
            return ExitCode::from(1);
        }
    };
    tracker.configure(&settings);
    install_signal_handler();

    // --- UDP sink (Opentrack classic protocol) -----------------------------
    let mut udp: Option<UdpSink> = None;
    if !settings.no_udp {
        match UdpSink::bind(&settings.host, settings.port, settings.udp_rate) {
            Ok(s) => udp = Some(s),
            Err(e) => eprintln!("warning: {e} — UDP disabled"),
        }
    }

    let rad = settings.rad;
    if settings.gyro_calib > 0.0 {
        println!(
            "gyro bias calibration: keep the device still for {:.1}s ...",
            settings.gyro_calib
        );
    }

    // --- Main loop ---------------------------------------------------------
    let mut last_hud = Instant::now();
    let mut rate_reported = false;
    let mut was_calibrating = settings.gyro_calib > 0.0;
    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            println!();
            println!("SIGINT received — shutting down cleanly");
            break;
        }

        match tracker.next_pose() {
            Ok(None) => {
                // Still calibrating — announce the moment it completes.
                if was_calibrating && !tracker.calibrating() {
                    was_calibrating = false;
                    println!("gyro bias calibrated — tracking");
                }
                continue;
            }
            Ok(Some(pose)) => {
                let now = Instant::now();
                if settings.hud_rate > 0.0 && now.duration_since(last_hud).as_secs_f64() >= 1.0 / settings.hud_rate {
                    last_hud = now;
                    print_hud(&pose, rad);
                }
                if let Some(s) = udp.as_mut() {
                    s.send(if rad { pose.ypr_rad } else { pose.ypr_rad.map(f64::to_degrees) });
                }
                if !rate_reported {
                    rate_reported = true;
                    eprintln!("measured imu_rate≈{:.0} Hz", tracker.last_rate_hz);
                }
            }
            Err(e) => {
                if CTRL_C.load(Ordering::SeqCst) >= 1 {
                    break;
                }
                eprintln!("error: {e}");
                eprintln!("(device unplugged — restart the tracker to reconnect; exit code 3)");
                return ExitCode::from(3);
            }
        }
    }

    ExitCode::SUCCESS
}

fn print_hud(pose: &Pose, rad: bool) {
    if rad {
        println!("YAW {:7.3}  PITCH {:7.3}  ROLL {:7.3} rad", pose.ypr_rad[0], pose.ypr_rad[1], pose.ypr_rad[2]);
    } else {
        println!(
            "YAW {:7.1}°  PITCH {:7.1}°  ROLL {:7.1}°",
            pose.ypr_rad[0].to_degrees(),
            pose.ypr_rad[1].to_degrees(),
            pose.ypr_rad[2].to_degrees()
        );
    }
}
