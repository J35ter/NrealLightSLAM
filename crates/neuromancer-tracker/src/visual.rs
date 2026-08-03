//! Visual input layer (P2): stereo frame sources behind a common trait.
//!
//! `SlamCameraSource` reads the Nreal Light's SLAM cameras via `ar-drivers`
//! (640×480 grayscale stereo, ~30 fps). `ReplayVisualSource` reads raw
//! grayscale frames from a directory (`left_XXXX.raw`, `right_XXXX.raw` —
//! the `--replay-visual` dev/testing path, mirroring `--replay` for IMU).

use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};
use std::time::Instant;

/// One stereo frame.
pub struct VisualFrame {
    pub t: f64,
    pub left: Vec<u8>,
    pub right: Vec<u8>,
    pub width: u32,
    pub height: u32,
}

/// A source of stereo frames.
pub trait VisualSource {
    fn next_frame(&mut self) -> Result<VisualFrame, String>;
    fn name(&self) -> String;
}

/// Hardware source: Nreal Light SLAM cameras via ar-drivers.
pub struct SlamCameraSource {
    cam: ar_drivers::nreal_light::NrealLightSlamCamera,
    t0: Instant,
}

impl SlamCameraSource {
    pub fn open() -> Result<Self, String> {
        let cam = ar_drivers::nreal_light::NrealLightSlamCamera::new()
            .map_err(|e| format!("cannot open Nreal Light SLAM camera: {e}"))?;
        Ok(SlamCameraSource {
            cam,
            t0: Instant::now(),
        })
    }
}

impl VisualSource for SlamCameraSource {
    fn next_frame(&mut self) -> Result<VisualFrame, String> {
        let frame = self
            .cam
            .get_frame(std::time::Duration::from_secs(2))
            .map_err(|e| format!("SLAM camera read failed: {e}"))?;
        Ok(VisualFrame {
            t: self.t0.elapsed().as_secs_f64(),
            left: frame.left,
            right: frame.right,
            width: 640,
            height: 480,
        })
    }

    fn name(&self) -> String {
        "NrealLightSlamCamera".to_string()
    }
}

/// Dev/testing source: raw grayscale stereo frames from a directory.
pub struct ReplayVisualSource {
    dir: PathBuf,
    index: usize,
    width: u32,
    height: u32,
    t0: Instant,
}

impl ReplayVisualSource {
    pub fn open(dir: &Path, width: u32, height: u32) -> Result<Self, String> {
        if !dir.is_dir() {
            return Err(format!("replay-visual path is not a directory: {}", dir.display()));
        }
        Ok(ReplayVisualSource {
            dir: dir.to_path_buf(),
            index: 0,
            width,
            height,
            t0: Instant::now(),
        })
    }

    fn load(&self, idx: usize, side: &str) -> Result<Vec<u8>, String> {
        let path = self.dir.join(format!("{side}_{idx:04}.raw"));
        let mut buf = Vec::with_capacity((self.width * self.height) as usize);
        File::open(&path)
            .and_then(|mut f| f.read_to_end(&mut buf))
            .map_err(|e| format!("cannot read {}: {e}", path.display()))?;
        if buf.len() != (self.width * self.height) as usize {
            return Err(format!(
                "{}: expected {} bytes, got {}",
                path.display(),
                self.width * self.height,
                buf.len()
            ));
        }
        Ok(buf)
    }
}

impl VisualSource for ReplayVisualSource {
    fn next_frame(&mut self) -> Result<VisualFrame, String> {
        let left = match self.load(self.index, "left") {
            Ok(l) => l,
            Err(e) if self.index == 0 => return Err(e),
            Err(_) => return Err("end of visual replay".to_string()),
        };
        let right = self.load(self.index, "right")?;
        let frame = VisualFrame {
            t: self.t0.elapsed().as_secs_f64(),
            left,
            right,
            width: self.width,
            height: self.height,
        };
        self.index += 1;
        Ok(frame)
    }

    fn name(&self) -> String {
        format!("replay-visual({})", self.dir.display())
    }
}

/// Discover the frame count in a replay-visual directory.
pub fn replay_visual_frames(dir: &Path) -> usize {
    fs::read_dir(dir)
        .map(|rd| {
            rd.filter_map(|e| e.ok())
                .filter(|e| {
                    let n = e.file_name();
                    n.to_string_lossy().starts_with("left_") && n.to_string_lossy().ends_with(".raw")
                })
                .count()
        })
        .unwrap_or(0)
}
