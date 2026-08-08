//! Shared core of the minimal IMU-only tracker (`neuromancer-imu-tracker`).
//!
//! Both the headless CLI binary (`main.rs`) and the `neuromancer-gui` crate
//! embed this: USB IMU → startup gyro-bias calibration → Mahony AHRS →
//! YXZ Tait-Bryan yaw/pitch/roll, plus an optional Opentrack UDP sink.
//!
//! Deliberately links ONLY `neuromancer-ahrs` + the vendored `ar-drivers`
//! USB backend — no `neuromancer-vo`, no `neuromancer-fusion`.

use std::net::{SocketAddr, UdpSocket};
use std::path::PathBuf;
use std::time::{Duration, Instant};

use ar_drivers::nreal_light::NrealLight;
use ar_drivers::ARGlasses;
use neuromancer_ahrs::{quat_to_ypr, Mahony};

pub use neuromancer_ahrs::Quat;

// ---------------------------------------------------------------------------
// Settings (serde TOML) — one field per CLI switch, so the GUI menu and the
// CLI flags configure the same knobs (spec §4.5-style settings file).
// ---------------------------------------------------------------------------

/// Tracker settings, serialized to TOML by the GUI and parsed from CLI flags
/// by `main.rs`. `Default` is the out-of-the-box config (matches Phase 1).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Settings {
    /// Disable UDP output (HUD/on-screen only).
    pub no_udp: bool,
    /// UDP destination host (Opentrack).
    pub host: String,
    /// UDP destination port.
    pub port: u16,
    /// UDP send rate (Hz).
    pub udp_rate: f64,
    /// HUD refresh rate (Hz; 0 = off).
    pub hud_rate: f64,
    /// Startup gyro-bias calibration window (s; 0 = skip).
    pub gyro_calib: f64,
    /// Mahony proportional gain.
    pub kp: f64,
    /// Mahony integral gain.
    pub ki: f64,
    /// Output units: false = deg, true = rad.
    pub rad: bool,
    /// (GUI) show the 3D cube visualization.
    pub show_cube: bool,
}

impl Default for Settings {
    fn default() -> Self {
        Settings {
            no_udp: false,
            host: "127.0.0.1".to_string(),
            port: 4242,
            udp_rate: 60.0,
            hud_rate: 2.0,
            gyro_calib: 2.0,
            kp: 1.0,
            ki: 0.005,
            rad: false,
            show_cube: true,
        }
    }
}

impl Settings {
    /// Apply a `--flag value` / `--flag` pair (the CLI surface). Returns
    /// `Ok(false)` for a flag it doesn't recognize.
    pub fn apply_cli(&mut self, flag: &str, value: Option<&str>) -> Result<bool, String> {
        let v = || value.ok_or_else(|| format!("{flag} needs a value"));
        Ok(match flag {
            "--no-udp" => {
                self.no_udp = true;
                true
            }
            "--host" => {
                self.host = v()?.to_string();
                // Explicit UDP flags imply UDP output is wanted, overriding a
                // config file that may have no_udp = true.
                self.no_udp = false;
                true
            }
            "--port" => {
                let raw = v()?;
                self.port = raw.parse().map_err(|_| format!("invalid port: {raw:?}"))?;
                self.no_udp = false;
                true
            }
            "--udp-rate" => {
                self.udp_rate = parse_nonneg(v()?, flag)?;
                self.no_udp = false;
                true
            }
            "--hud-rate" => {
                self.hud_rate = parse_nonneg(v()?, flag)?;
                true
            }
            "--gyro-calib" => {
                self.gyro_calib = parse_nonneg(v()?, flag)?;
                true
            }
            "--kp" => {
                self.kp = parse_nonneg(v()?, flag)?;
                true
            }
            "--ki" => {
                self.ki = parse_nonneg(v()?, flag)?;
                true
            }
            "--units" => {
                self.rad = match v()? {
                    "deg" => false,
                    "rad" => true,
                    other => return Err(format!("--units must be deg|rad, got {other:?}")),
                };
                true
            }
            _ => false,
        })
    }

    /// Platform config-dir settings file path.
    /// Linux: `$XDG_CONFIG_HOME|~/.config/neuromancer-tracker/settings.toml`
    /// Windows: `%APPDATA%\neuromancer-tracker\settings.toml`
    pub fn config_path() -> PathBuf {
        #[cfg(target_os = "windows")]
        let base = std::env::var("APPDATA").map(PathBuf::from).unwrap_or_else(|_| PathBuf::from("."));
        #[cfg(not(target_os = "windows"))]
        let base = std::env::var("XDG_CONFIG_HOME")
            .map(PathBuf::from)
            .or_else(|_| std::env::var("HOME").map(|h| PathBuf::from(h).join(".config")))
            .unwrap_or_else(|_| PathBuf::from("."));
        base.join("neuromancer-tracker").join("settings.toml")
    }

    /// Load settings from the platform config path (missing file → defaults).
    pub fn load() -> Self {
        let path = Self::config_path();
        match std::fs::read_to_string(&path) {
            Ok(text) => match toml::from_str(&text) {
                Ok(s) => s,
                Err(e) => {
                    eprintln!("warning: settings file {} unreadable ({e}) — using defaults", path.display());
                    Self::default()
                }
            },
            Err(_) => Self::default(),
        }
    }

    /// Save settings to the platform config path.
    pub fn save(&self) -> Result<PathBuf, String> {
        let path = Self::config_path();
        if let Some(dir) = path.parent() {
            std::fs::create_dir_all(dir).map_err(|e| format!("cannot create {}: {e}", dir.display()))?;
        }
        let text = toml::to_string_pretty(self).map_err(|e| format!("cannot serialize settings: {e}"))?;
        std::fs::write(&path, text).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        Ok(path)
    }
}

fn parse_nonneg(raw: &str, flag: &str) -> Result<f64, String> {
    let v: f64 = raw.parse().map_err(|_| format!("invalid value for {flag}: {raw:?}"))?;
    if v < 0.0 {
        return Err(format!("{flag} must be >= 0: {raw}"));
    }
    Ok(v)
}

// ---------------------------------------------------------------------------
// Startup gyro-bias calibration (same still-window detection as before)
// ---------------------------------------------------------------------------

/// Mean gyro over a "still" window (accel magnitude near g, low angular
/// rate). Returns the bias when the window completes.
pub struct Calibrator {
    window_seconds: f64,
    start_t: Option<f64>,
    sum: [f64; 3],
    samples: u64,
    done: bool,
}

impl Calibrator {
    pub fn new(window_seconds: f64) -> Self {
        Calibrator { window_seconds, start_t: None, sum: [0.0; 3], samples: 0, done: false }
    }

    /// Feed one sample; returns the bias when the window completes.
    pub fn push(&mut self, t: f64, accel: [f64; 3], gyro: [f64; 3]) -> Option<[f64; 3]> {
        if self.done {
            return None;
        }
        let t0 = *self.start_t.get_or_insert(t);
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

// ---------------------------------------------------------------------------
// Rate gate
// ---------------------------------------------------------------------------

pub struct RateGate {
    interval: Duration,
    next: Instant,
}

impl RateGate {
    pub fn new(rate_hz: f64) -> Self {
        RateGate {
            interval: if rate_hz > 0.0 { Duration::from_secs_f64(1.0 / rate_hz) } else { Duration::MAX },
            next: Instant::now(),
        }
    }
    pub fn allow(&mut self, now: Instant) -> bool {
        if now >= self.next {
            self.next = now + self.interval;
            true
        } else {
            false
        }
    }
}

// ---------------------------------------------------------------------------
// UDP sink (Opentrack classic protocol, 48-byte 6×f64)
// ---------------------------------------------------------------------------

pub struct UdpSink {
    socket: UdpSocket,
    gate: RateGate,
}

impl UdpSink {
    pub fn bind(host: &str, port: u16, rate_hz: f64) -> Result<Self, String> {
        let dest: SocketAddr = format!("{host}:{port}").parse().map_err(|e| format!("bad UDP destination {host}:{port}: {e}"))?;
        let socket = UdpSocket::bind(if dest.is_ipv4() { "0.0.0.0:0" } else { "[::]:0" })
            .map_err(|e| format!("UDP bind failed: {e}"))?;
        let _ = socket.connect(dest);
        Ok(UdpSink { socket, gate: RateGate::new(rate_hz) })
    }

    /// Send one pose; rate-gated. `ypr` in the configured output units;
    /// position is fixed at origin (3-DoF), TX/TY/TZ in centimeters.
    pub fn send(&mut self, ypr: [f64; 3]) {
        if !self.gate.allow(Instant::now()) {
            return;
        }
        let vals = [0.0f64, 0.0, 0.0, ypr[0], ypr[1], ypr[2]];
        let mut buf = [0u8; 48];
        for (i, v) in vals.iter().enumerate() {
            buf[i * 8..i * 8 + 8].copy_from_slice(&v.to_ne_bytes());
        }
        let _ = self.socket.send(&buf);
    }
}

// ---------------------------------------------------------------------------
// The tracker core: one `ImuTracker` per device, driven by `next_pose`.
// ---------------------------------------------------------------------------

/// A single filtered pose handed to sinks/GUI: `t` = stream time,
/// `ypr_rad` = yaw/pitch/roll in radians (post-Mahony, pre-axis-map),
/// `q` = the orientation quaternion (world→body, AHRS convention).
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub t: f64,
    pub ypr_rad: [f64; 3],
    pub q: Quat,
}

/// The minimal IMU pipeline. Call [`ImuTracker::open`] once, then loop on
/// [`ImuTracker::next_pose`]. While the startup calibration is running it
/// returns `Ok(None)` (keep the device still).
pub struct ImuTracker {
    glasses: NrealLight,
    calib: Option<Calibrator>,
    mahony: Mahony,
    bias: [f64; 3],
    prev_t: Option<f64>,
    t0: Instant,
    pub last_rate_hz: f64,
}

impl ImuTracker {
    /// Open the USB connection to the Nreal Light.
    pub fn open() -> Result<Self, String> {
        let glasses = NrealLight::new()
            .map_err(|e| format!("no Nreal Light found on USB: {e} (is it plugged in?)"))?;
        Ok(ImuTracker {
            glasses,
            calib: None,
            mahony: Mahony::new(),
            bias: [0.0; 3],
            prev_t: None,
            t0: Instant::now(),
            last_rate_hz: 0.0,
        })
    }

    /// Configure from settings (calibration window + Mahony gains).
    pub fn configure(&mut self, settings: &Settings) {
        self.calib = (settings.gyro_calib > 0.0).then(|| Calibrator::new(settings.gyro_calib));
        self.mahony = Mahony::with_gains(settings.kp, settings.ki);
        self.prev_t = None;
    }

    /// Whether the startup bias calibration is still running.
    pub fn calibrating(&self) -> bool {
        self.calib.is_some()
    }

    /// Block for the next sample and produce the next pose. `Ok(None)` while
    /// calibrating. `Err` on USB failure/unplug.
    pub fn next_pose(&mut self) -> Result<Option<Pose>, String> {
        let (t, accel, gyro) = loop {
            match self.glasses.read_event() {
                Ok(ar_drivers::GlassesEvent::AccGyro { accelerometer, gyroscope, .. }) => {
                    break (
                        self.t0.elapsed().as_secs_f64(),
                        [accelerometer.x as f64, accelerometer.y as f64, accelerometer.z as f64],
                        [gyroscope.x as f64, gyroscope.y as f64, gyroscope.z as f64],
                    );
                }
                Ok(_) => continue, // keypress/proximity — skip
                Err(e) => return Err(format!("IMU input failed: {e}")),
            }
        };

        // Startup calibration consumes samples until the window completes.
        if let Some(c) = self.calib.as_mut() {
            if let Some(b) = c.push(t, accel, gyro) {
                self.bias = b;
                self.mahony.reset();
                self.prev_t = Some(t);
                self.calib = None;
                return Ok(None); // next call produces the first real pose
            }
            return Ok(None);
        }

        let dt = match self.prev_t {
            Some(pt) => (t - pt).clamp(0.0, 0.1),
            None => 0.0,
        };
        self.prev_t = Some(t);
        if dt > 1e-9 {
            self.last_rate_hz = 1.0 / dt;
        }

        let q = self
            .mahony
            .update(accel, [gyro[0] - self.bias[0], gyro[1] - self.bias[1], gyro[2] - self.bias[2]], dt);
        let ypr_rad = quat_to_ypr(q);
        Ok(Some(Pose { t, ypr_rad, q }))
    }
}
