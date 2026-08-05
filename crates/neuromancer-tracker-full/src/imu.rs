//! IMU input layer: `ImuSource` trait behind which the USB backend
//! (`ar-drivers` / Nreal Light) and the dev replay source live (spec §2.3).

use std::fmt;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};

use ar_drivers::nreal_light::NrealLight;
use ar_drivers::ARGlasses;

use crate::jsonl::{self, ImuSample};

/// Errors from an [`ImuSource`].
pub enum ImuError {
    /// I/O or parsing problem (replay file).
    Io(String),
    /// The glasses stopped producing data mid-run (USB unplug → exit 3).
    Unplugged(String),
    /// Replay source reached end of file (clean end).
    Eof,
}

impl fmt::Display for ImuError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ImuError::Io(m) => write!(f, "I/O error: {m}"),
            ImuError::Unplugged(m) => write!(f, "device error: {m}"),
            ImuError::Eof => write!(f, "end of input"),
        }
    }
}

/// A source of IMU samples. Every sample is consumed (no downsampling at
/// input, spec §3.3); `t` is monotonic seconds since stream start.
pub trait ImuSource {
    /// Block for the next sample.
    fn next_sample(&mut self) -> Result<ImuSample, ImuError>;
    /// Human-readable device name for the startup confirmation line.
    fn name(&self) -> String;
}

/// USB backend: Nreal Light glasses via `ar-drivers` (the only production
/// input path — spec §2.3).
pub struct ArDriversSource {
    glasses: Box<dyn ARGlasses>,
    t0: Instant,
    name: String,
}

impl ArDriversSource {
    /// Open the USB connection to the Nreal Light. Returns a clear error
    /// ("no Nreal Light found on USB") when no device is present.
    pub fn open() -> Result<Self, String> {
        let glasses = NrealLight::new().map_err(|e| {
            format!("no Nreal Light found on USB: {e} (is it plugged in?)")
        })?;
        let name = glasses.name().to_string();
        Ok(ArDriversSource {
            glasses: Box::new(glasses),
            t0: Instant::now(),
            name,
        })
    }
}

impl ImuSource for ArDriversSource {
    fn next_sample(&mut self) -> Result<ImuSample, ImuError> {
        loop {
            match self.glasses.read_event() {
                Ok(ar_drivers::GlassesEvent::AccGyro {
                    accelerometer,
                    gyroscope,
                    ..
                }) => {
                    let t = self.t0.elapsed().as_secs_f64();
                    return Ok(ImuSample {
                        t,
                        ax: accelerometer.x as f64,
                        ay: accelerometer.y as f64,
                        az: accelerometer.z as f64,
                        gx: gyroscope.x as f64,
                        gy: gyroscope.y as f64,
                        gz: gyroscope.z as f64,
                    });
                }
                Ok(_) => continue, // keypress / proximity / other events
                Err(e) => {
                    return Err(ImuError::Unplugged(format!(
                        "Nreal Light stopped responding: {e} (unplugged?)"
                    )))
                }
            }
        }
    }

    fn name(&self) -> String {
        self.name.clone()
    }
}

/// Dev/testing aid: feeds a JSONL IMU log (`--log-imu` output) through the
/// same [`ImuSource`] abstraction (spec §2.3, §2.8 — replay at test level).
///
/// With `pacing = true` samples are delivered at the recorded rate
/// (wall-clock sleep), so a replay run behaves like a live stream; tests use
/// `pacing = false` for determinism.
pub struct ReplaySource {
    path: PathBuf,
    reader: BufReader<File>,
    pacing: bool,
    line_no: usize,
    prev_t: Option<f64>,
    prev_wall: Option<Instant>,
    eof: bool,
}

impl ReplaySource {
    pub fn open(path: &Path, pacing: bool) -> Result<Self, String> {
        let file = File::open(path)
            .map_err(|e| format!("cannot open replay file {}: {e}", path.display()))?;
        Ok(ReplaySource {
            path: path.to_path_buf(),
            reader: BufReader::new(file),
            pacing,
            line_no: 0,
            prev_t: None,
            prev_wall: None,
            eof: false,
        })
    }
}

impl ImuSource for ReplaySource {
    fn next_sample(&mut self) -> Result<ImuSample, ImuError> {
        if self.eof {
            return Err(ImuError::Eof);
        }
        let mut line = String::new();
        loop {
            line.clear();
            let n = self
                .reader
                .read_line(&mut line)
                .map_err(|e| ImuError::Io(format!("read {}: {e}", self.path.display())))?;
            if n == 0 {
                self.eof = true;
                return Err(ImuError::Eof);
            }
            self.line_no += 1;
            match jsonl::parse_imu_line(&line) {
                Ok(Some(sample)) => {
                    if self.pacing {
                        if let Some(pt) = self.prev_t {
                            let dt = (sample.t - pt).max(0.0);
                            if dt > 0.0 {
                                std::thread::sleep(Duration::from_secs_f64(dt));
                            }
                        }
                    }
                    self.prev_t = Some(sample.t);
                    self.prev_wall = Some(Instant::now());
                    return Ok(sample);
                }
                Ok(None) => continue, // blank line
                Err(e) => {
                    return Err(ImuError::Io(format!(
                        "{}:{}: {e}",
                        self.path.display(),
                        self.line_no
                    )))
                }
            }
        }
    }

    fn name(&self) -> String {
        format!("replay({})", self.path.display())
    }
}
