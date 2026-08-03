//! Image frame types shared by the VO pipeline.

/// A grayscale stereo frame pair (left/right, row-major, `width * height`).
#[derive(Debug, Clone, PartialEq)]
pub struct StereoFrame {
    pub width: u32,
    pub height: u32,
    pub left: Vec<u8>,
    pub right: Vec<u8>,
}

impl StereoFrame {
    pub fn new(width: u32, height: u32, left: Vec<u8>, right: Vec<u8>) -> Self {
        debug_assert_eq!(left.len(), (width * height) as usize);
        debug_assert_eq!(right.len(), (width * height) as usize);
        StereoFrame { width, height, left, right }
    }

    pub fn pixel(&self, left: bool, u: u32, v: u32) -> u8 {
        let buf = if left { &self.left } else { &self.right };
        buf[(v * self.width + u) as usize]
    }
}

/// Per-pixel depth (left-camera frame z, meters) for the left image.
/// `f64::INFINITY` marks pixels with no depth.
#[derive(Debug, Clone)]
pub struct DepthMap {
    pub width: u32,
    pub height: u32,
    pub z: Vec<f64>,
}

impl DepthMap {
    pub fn new(width: u32, height: u32, z: Vec<f64>) -> Self {
        debug_assert_eq!(z.len(), (width * height) as usize);
        DepthMap { width, height, z }
    }
}
