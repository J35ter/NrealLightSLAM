//! Minimal IMU-only fork of the Nreal Light head tracker (Phase 1, 3-DoF).
//!
//! This binary deliberately links ONLY `neuromancer-ahrs` (dependency-free
//! Mahony) + the vendored `ar-drivers` USB backend — no `neuromancer-vo`,
//! no `neuromancer-fusion`, no `neuromancer-tracker` library. It is the
//! build-time fork: no visual/VO overhead in the binary, at the cost of
//! duplicating a small amount of glue (CLI, gyro calibration, HUD/UDP
//! sinks) that the full tracker keeps in its lib.
//!
//! Pipeline (spec §3.3): USB IMU → startup gyro-bias calibration → Mahony
//! (kp=1.0, ki=0.005, same as the main tracker) → YXZ Tait-Bryan
//! yaw/pitch/roll (deg) → 2 Hz HUD + optional Opentrack UDP (classic
//! protocol, 48-byte 6×f64) at 60 Hz.
//!
//! Usage:
//!   cargo run --release -p neuromancer-imu-tracker
//!   cargo run --release -p neuromancer-imu-tracker -- --no-udp
//!   cargo run --release -p neuromancer-imu-tracker -- --gyro-calib 2 --hud-rate 5
//!
//! Flags:
//!   --no-udp            disable UDP output (HUD only; UDP defaults on)
//!   --host HOST         UDP destination (default 127.0.0.1)
//!   --port PORT         UDP port (default 4242)
//!   --udp-rate HZ       UDP send rate (default 60)
//!   --hud-rate HZ       HUD refresh rate (default 2; 0 = off)
//!   --gyro-calib SECS   startup bias window (default 2.0; 0 = skip)
//!   --kp F, --ki F      Mahony gains (default 1.0 / 0.005)
//!   --units deg|rad     output units (default deg)

use std::net::{SocketAddr, UdpSocket};
use std::process::ExitCode;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use ar_drivers::nreal_light::NrealLight;
use ar_drivers::ARGlasses;
use neuromancer_ahrs::{quat_to_ypr, Mahony};

/// SIGINT flag: 0 = run, >= 1 = shutdown. Set by the libc handler.
static CTRL_C: AtomicUsize = AtomicUsize::new(0);

extern "C" fn on_sigint(_: libc::c_int) {
    CTRL_C.store(1, Ordering::SeqCst);
}

fn install_signal_handler() {
    unsafe {
        let mut sa: libc::sigaction = std::mem::zeroed();
        sa.sa_sigaction = on_sigint as *const () as libc::sighandler_t;
        libc::sigemptyset(&mut sa.sa_mask);
        sa.sa_flags = 0; // no SA_RESTART: interrupt the blocking USB read
        libc::sigaction(libc::SIGINT, &sa, std::ptr::null_mut());
    }
}

#[derive(Debug, Clone)]
struct Cli {
    no_udp: bool,
    host: String,
    port: u16,
    udp_rate: f64,
    hud_rate: f64,
    gyro_calib: f64,
    kp: f64,
    ki: f64,
    rad: bool,
}

impl Default for Cli {
    fn default() -> Self {
        Cli {
            no_udp: false,
            host: "127.0.0.1".to_string(),
            port: 4242,
            udp_rate: 60.0,
            hud_rate: 2.0,
            gyro_calib: 2.0,
            kp: 1.0,
            ki: 0.005,
            rad: false,
        }
    }
}

fn parse_nonneg(raw: &str, flag: &str) -> Result<f64, String> {
    let v: f64 = raw
        .parse()
        .map_err(|_| format!("invalid value for {flag}: {raw:?}"))?;
    if v < 0.0 {
        return Err(format!("{flag} must be >= 0: {raw}"));
    }
    Ok(v)
}

fn parse_args(args: &[String]) -> Result<Cli, String> {
    let mut cfg = Cli::default();
    let mut it = args.iter().peekable();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--no-udp" => cfg.no_udp = true,
            "--host" => cfg.host = it.next().ok_or("--host needs a value")?.clone(),
            "--port" => {
                let raw = it.next().ok_or("--port needs a value")?;
                cfg.port = raw
                    .parse()
                    .map_err(|_| format!("invalid port: {raw:?}"))?;
            }
            "--udp-rate" => cfg.udp_rate = parse_nonneg(it.next().ok_or("--udp-rate needs a value")?, "--udp-rate")?,
            "--hud-rate" => cfg.hud_rate = parse_nonneg(it.next().ok_or("--hud-rate needs a value")?, "--hud-rate")?,
            "--gyro-calib" => {
                cfg.gyro_calib = parse_nonneg(it.next().ok_or("--gyro-calib needs a value")?, "--gyro-calib")?
            }
            "--kp" => cfg.kp = parse_nonneg(it.next().ok_or("--kp needs a value")?, "--kp")?,
            "--ki" => cfg.ki = parse_nonneg(it.next().ok_or("--ki needs a value")?, "--ki")?,
            "--units" => {
                let raw = it.next().ok_or("--units needs a value")?;
                cfg.rad = match raw.as_str() {
                    "deg" => false,
                    "rad" => true,
                    other => return Err(format!("--units must be deg|rad, got {other:?}")),
                };
            }
            other => return Err(format!("unknown flag: {other}")),
        }
    }
    Ok(cfg)
}

/// Startup gyro-bias calibration: mean gyro over a "still" window (accel
/// magnitude near g and low angular rate), same detection as the main
/// tracker's calib.rs.
struct Calibrator {
    window_seconds: f64,
    start_t: Option<f64>,
    prev_t: Option<f64>,
    sum: [f64; 3],
    samples: u64,
    done: bool,
}

impl Calibrator {
    fn new(window_seconds: f64) -> Self {
        Calibrator { window_seconds, start_t: None, prev_t: None, sum: [0.0; 3], samples: 0, done: false }
    }

    /// Feed one sample; returns the bias when the window completes.
    fn push(&mut self, t: f64, accel: [f64; 3], gyro: [f64; 3]) -> Option<[f64; 3]> {
        if self.done {
            return None;
        }
        let t0 = *self.start_t.get_or_insert(t);
        let _ = self.prev_t.replace(t);
        let acc_norm = (accel[0] * accel[0] + accel[1] * accel[1] + accel[2] * accel[2]).sqrt();
        let gyro_norm = (gyro[0] * gyro[0] + gyro[1] * gyro[1] + gyro[2] * gyro[2]).sqrt();
        let still = acc_norm > 0.5 * 9.81 && acc_norm < 1.5 * 9.81 && gyro_norm < 0.05;
        if still {
            self.sum[0] += gyro[0];
            self.sum[1] += gyro[1];
            self.sum[2] += gyro[2];
            self.samples += 1;
        }
        if t - t0 >= self.window_seconds {
            self.done = true;
            let n = self.samples.max(1) as f64;
            return Some([self.sum[0] / n, self.sum[1] / n, self.sum[2] / n]);
        }
        None
    }
}

/// Rate-gated sink helper: allow `rate_hz` calls per second.
struct RateGate {
    interval: Duration,
    next: Instant,
}

impl RateGate {
    fn new(rate_hz: f64) -> Self {
        RateGate {
            interval: if rate_hz > 0.0 { Duration::from_secs_f64(1.0 / rate_hz) } else { Duration::MAX },
            next: Instant::now(),
        }
    }
    fn allow(&mut self, now: Instant) -> bool {
        if now >= self.next {
            self.next = now + self.interval;
            true
        } else {
            false
        }
    }
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let cfg = match parse_args(&args) {
        Ok(c) => c,
        Err(e) => {
            eprintln!("error: {e}");
            eprintln!("usage: neuromancer-imu-tracker [--no-udp] [--host H] [--port P] [--udp-rate HZ] [--hud-rate HZ] [--gyro-calib SECS] [--kp F] [--ki F] [--units deg|rad]");
            return ExitCode::from(2);
        }
    };

    // --- Open the USB IMU -------------------------------------------------
    let mut glasses = match NrealLight::new() {
        Ok(g) => g,
        Err(e) => {
            eprintln!("error: no Nreal Light found on USB: {e} (is it plugged in?)");
            return ExitCode::from(1);
        }
    };
    install_signal_handler();

    // --- Sinks ------------------------------------------------------------
    let mut udp: Option<(UdpSocket, SocketAddr, RateGate)> = None;
    if !cfg.no_udp {
        let dest: Result<SocketAddr, _> = format!("{}:{}", cfg.host, cfg.port).parse();
        if let Ok(dest) = dest {
            match UdpSocket::bind(if dest.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" }) {
                Ok(s) => {
                    let _ = s.connect(dest);
                    udp = Some((s, dest, RateGate::new(cfg.udp_rate)));
                }
                Err(e) => eprintln!("warning: UDP bind failed: {e}"),
            }
        } else {
            eprintln!("warning: bad UDP destination — UDP disabled");
        }
    }

    // --- Startup gyro-bias calibration -------------------------------------
    let mut calib = (cfg.gyro_calib > 0.0).then(|| Calibrator::new(cfg.gyro_calib));
    let mut bias = [0.0f64; 3];
    if calib.is_some() {
        println!("gyro bias calibration: keep the device still for {:.1}s ...", cfg.gyro_calib);
    }

    let mut mahony = Mahony::with_gains(cfg.kp, cfg.ki);
    let t0 = Instant::now();
    let mut prev_t: Option<f64> = None;
    let mut n_samples: u64 = 0;
    let mut rate_reported = false;
    let mut last_hud = Instant::now();

    loop {
        if CTRL_C.load(Ordering::SeqCst) >= 1 {
            println!();
            println!("SIGINT received — shutting down cleanly");
            break;
        }

        let (t, accel, gyro) = loop {
            match glasses.read_event() {
                Ok(ar_drivers::GlassesEvent::AccGyro { accelerometer, gyroscope, .. }) => {
                    break (
                        t0.elapsed().as_secs_f64(),
                        [accelerometer.x as f64, accelerometer.y as f64, accelerometer.z as f64],
                        [gyroscope.x as f64, gyroscope.y as f64, gyroscope.z as f64],
                    );
                }
                Ok(_) => continue, // keypress/proximity — skip
                Err(e) => {
                    if CTRL_C.load(Ordering::SeqCst) >= 1 {
                        println!();
                        println!("SIGINT received — shutting down cleanly");
                        return ExitCode::SUCCESS;
                    }
                    eprintln!("error: IMU input failed: {e}");
                    eprintln!("(device unplugged — restart the tracker to reconnect; exit code 3)");
                    return ExitCode::from(3);
                }
            }
        };

        // Startup calibration.
        if let Some(c) = calib.as_mut() {
            if let Some(b) = c.push(t, accel, gyro) {
                bias = b;
                println!(
                    "gyro bias calibrated: gx={:.6} gy={:.6} gz={:.6} rad/s",
                    bias[0], bias[1], bias[2]
                );
                mahony.reset();
                prev_t = Some(t);
                n_samples = 0;
                calib = None;
            } else {
                continue; // still calibrating: no filter/output yet
            }
        }

        let dt = match prev_t {
            Some(pt) => (t - pt).clamp(0.0, 0.1),
            None => 0.0,
        };
        prev_t = Some(t);

        let q = mahony.update(accel, [gyro[0] - bias[0], gyro[1] - bias[1], gyro[2] - bias[2]], dt);
        let ypr = quat_to_ypr(q);
        let ypr_out: [f64; 3] = if cfg.rad { ypr } else { ypr.map(f64::to_degrees) };

        // HUD (2 Hz default).
        let now = Instant::now();
        if cfg.hud_rate > 0.0 && now.duration_since(last_hud).as_secs_f64() >= 1.0 / cfg.hud_rate {
            last_hud = now;
            if cfg.rad {
                println!(
                    "YAW {:7.3}  PITCH {:7.3}  ROLL {:7.3} rad",
                    ypr_out[0], ypr_out[1], ypr_out[2]
                );
            } else {
                println!(
                    "YAW {:7.1}°  PITCH {:7.1}°  ROLL {:7.1}°",
                    ypr_out[0], ypr_out[1], ypr_out[2]
                );
            }
        }

        // UDP (classic 48-byte 6×f64: TX, TY, TZ, Yaw, Pitch, Roll; X/Y/Z in cm).
        if let Some((sock, _, gate)) = udp.as_mut() {
            if gate.allow(now) {
                let vals = [0.0f64, 0.0, 0.0, ypr_out[0], ypr_out[1], ypr_out[2]];
                let mut buf = [0u8; 48];
                for (i, v) in vals.iter().enumerate() {
                    buf[i * 8..i * 8 + 8].copy_from_slice(&v.to_ne_bytes());
                }
                let _ = sock.send(&buf);
            }
        }

        // Measured IMU rate (report once after 30 samples).
        n_samples += 1;
        if !rate_reported && n_samples == 30 {
            eprintln!("measured imu_rate≈{:.0} Hz", 1.0 / dt.max(1e-9));
            rate_reported = true;
        }
    }

    ExitCode::SUCCESS
}
