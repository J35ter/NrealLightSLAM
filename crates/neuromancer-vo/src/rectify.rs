//! Fisheye undistortion + stereo rectification maps (M8).
//!
//! The OV580 SLAM cameras deliver *raw* (distorted, un-rectified) frames:
//! per-camera intrinsics differ (fx 234.40 vs 234.46, cy 245.7 vs 214.1)
//! and the relative pose is not a pure X baseline. The rest of the pipeline
//! (FAST → KLT → epipolar stereo → triangulation) assumes the canonical
//! rectified rig (`StereoRig::rectified`), so raw frames are remapped once,
//! at startup, into a rectified pair:
//!
//!   1. **Undistort** each camera with its `kc = (k1, k2, p1, p2, k3)`
//!      radial+tangential model (OpenCV order).
//!   2. **Rectify** with Bouguet-style rotations `R_l_rect`, `R_r_rect`
//!      derived from the relative pose, so epipolar lines become horizontal.
//!   3. **Resample** to a shared rectified intrinsics `(fx, fy, cx, cy)`.
//!
//! The resulting `StereoRig` (identical intrinsics, baseline along X) is what
//! the VO pipeline consumes; the rectified images are what FAST/KLT/stereo
//! see. Maps are precomputed as inverse remaps (output pixel → input pixel)
//! and applied with bilinear sampling per frame.

use nalgebra::{Isometry3, Matrix3, Vector3};

use super::camera::StereoRig;

/// Per-camera calibration (from the device's `CameraDescriptor` or a
/// recorded fallback). `kc` is OpenCV order `(k1, k2, p1, p2, k3)`.
#[derive(Debug, Clone, Copy)]
pub struct CalibCam {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// Radial (k1,k2,k3) + tangential (p1,p2) distortion coefficients.
    pub kc: [f64; 5],
    pub width: u32,
    pub height: u32,
}

/// Precomputed undistort+rectify remaps plus the rectified rig they feed.
pub struct Rectifier {
    map_x_l: Vec<f32>,
    map_y_l: Vec<f32>,
    map_x_r: Vec<f32>,
    map_y_r: Vec<f32>,
    width: u32,
    height: u32,
    /// The rectified rig: identical intrinsics, right camera `baseline` to
    /// the right, no rotation — what `VoPipeline` expects.
    pub rig: StereoRig,
}

/// Radial+tangential distortion of normalized coordinates (forward model).
/// `(x, y)` are normalized (pinhole) coords; returns distorted coords.
fn distort(x: f64, y: f64, kc: &[f64; 5]) -> (f64, f64) {
    let [k1, k2, p1, p2, k3] = *kc;
    let r2 = x * x + y * y;
    let r4 = r2 * r2;
    let r6 = r4 * r2;
    let radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    let xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    let yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;
    (xd, yd)
}

/// Build one inverse remap: for every rectified pixel, the source (raw)
/// pixel coordinate. `r_rect` maps the original camera frame into the
/// rectified frame; we invert it to pull each rectified ray back to the
/// original camera, then apply the camera's distortion.
fn build_map(
    cam: &CalibCam,
    r_rect: &Matrix3<f64>,
    fx_rect: f64,
    fy_rect: f64,
    cx_rect: f64,
    cy_rect: f64,
) -> (Vec<f32>, Vec<f32>) {
    let n = (cam.width * cam.height) as usize;
    let mut mx = vec![0.0f32; n];
    let mut my = vec![0.0f32; n];
    let r_rect_t = r_rect.transpose();
    for v in 0..cam.height {
        for u in 0..cam.width {
            // Normalized coords in the rectified frame.
            let xn = (u as f64 - cx_rect) / fx_rect;
            let yn = (v as f64 - cy_rect) / fy_rect;
            // Pull back to the original (undistorted) camera frame.
            let dir = r_rect_t * Vector3::new(xn, yn, 1.0);
            if dir.z <= 1e-9 {
                mx[(v * cam.width + u) as usize] = -1.0;
                my[(v * cam.width + u) as usize] = -1.0;
                continue;
            }
            // Project through the original intrinsics (undistorted pixel).
            let uu = cam.fx * dir.x / dir.z + cam.cx;
            let vu = cam.fy * dir.y / dir.z + cam.cy;
            // Apply distortion → raw pixel.
            let (xdu, ydu) = distort((uu - cam.cx) / cam.fx, (vu - cam.cy) / cam.fy, &cam.kc);
            let uraw = cam.fx * xdu + cam.cx;
            let vraw = cam.fy * ydu + cam.cy;
            mx[(v * cam.width + u) as usize] = uraw as f32;
            my[(v * cam.width + u) as usize] = vraw as f32;
        }
    }
    (mx, my)
}

impl Rectifier {
    /// Build rectification maps from the two cameras' calibration and their
    /// relative pose (`left_t_right`: pose of the right camera in the left
    /// camera frame).
    pub fn new(left: &CalibCam, right: &CalibCam, left_t_right: Isometry3<f64>) -> Self {
        let width = left.width;
        let height = left.height;

        // Bouguet rectification rotation: make the baseline the X axis.
        let t = left_t_right.translation.vector;
        let tn = t.norm();
        let e1 = t / tn;
        let e2 = Vector3::new(-e1.y, e1.x, 0.0).normalize();
        let e3 = e1.cross(&e2);
        let r_rect = Matrix3::from_rows(&[e1.transpose(), e2.transpose(), e3.transpose()]);
        let r_l_rect = r_rect;
        let r_r_rect = r_rect * left_t_right.rotation.to_rotation_matrix().transpose();

        // Shared rectified intrinsics (mean of the two cameras).
        let fx_rect = (left.fx + right.fx) / 2.0;
        let fy_rect = (left.fy + right.fy) / 2.0;
        let cx_rect = (left.cx + right.cx) / 2.0;
        let cy_rect = (left.cy + right.cy) / 2.0;

        let (map_x_l, map_y_l) = build_map(left, &r_l_rect, fx_rect, fy_rect, cx_rect, cy_rect);
        let (map_x_r, map_y_r) = build_map(right, &r_r_rect, fx_rect, fy_rect, cx_rect, cy_rect);

        let rig = StereoRig::rectified(fx_rect, fy_rect, cx_rect, cy_rect, tn, width, height);

        Rectifier { map_x_l, map_y_l, map_x_r, map_y_r, width, height, rig }
    }

    /// Rectify one raw stereo pair into the rectified pair (bilinear
    /// sampling; out-of-image samples are clamped to the edge pixel).
    pub fn apply(&self, left: &[u8], right: &[u8]) -> (Vec<u8>, Vec<u8>) {
        let (wl, hl) = (self.width as i32, self.height as i32);
        let sample = |img: &[u8], mx: &[f32], my: &[f32]| -> Vec<u8> {
            let mut out = vec![0u8; (wl * hl) as usize];
            for v in 0..hl {
                for u in 0..wl {
                    let idx = (v * wl + u) as usize;
                    let sx = mx[idx];
                    let sy = my[idx];
                    if sx < 0.0 || sy < 0.0 {
                        continue; // black border
                    }
                    let x0 = sx.floor() as i32;
                    let y0 = sy.floor() as i32;
                    let x1 = (x0 + 1).min(wl - 1).max(0);
                    let y1 = (y0 + 1).min(hl - 1).max(0);
                    let x0 = x0.clamp(0, wl - 1);
                    let y0 = y0.clamp(0, hl - 1);
                    let dx = sx - x0 as f32;
                    let dy = sy - y0 as f32;
                    let a = img[(y0 * wl + x0) as usize] as f32;
                    let b = img[(y0 * wl + x1) as usize] as f32;
                    let c = img[(y1 * wl + x0) as usize] as f32;
                    let d = img[(y1 * wl + x1) as usize] as f32;
                    let top = a + (b - a) * dx;
                    let bot = c + (d - c) * dx;
                    out[idx] = (top + (bot - top) * dy).round() as u8;
                }
            }
            out
        };
        (sample(left, &self.map_x_l, &self.map_y_l), sample(right, &self.map_x_r, &self.map_y_r))
    }

    /// Validate a rectified pair: report the fraction of pixels that are not
    /// black border (sampled from inside the raw image), as a rectification
    /// sanity metric.
    pub fn coverage(&self) -> f64 {
        let n = (self.width * self.height) as f64;
        let good = |mx: &[f32], my: &[f32]| {
            mx.iter().zip(my.iter()).filter(|(x, y)| **x >= 0.0 && **y >= 0.0).count() as f64
        };
        (good(&self.map_x_l, &self.map_y_l) + good(&self.map_x_r, &self.map_y_r)) / (2.0 * n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion};

    fn calib_cam(fx: f64, cx: f64, cy: f64) -> CalibCam {
        CalibCam { fx, fy: fx, cx, cy, kc: [0.0; 5], width: 640, height: 480 }
    }

    #[test]
    fn zero_distortion_identity_pose_gives_identity_rectification() {
        // Identical cameras, pure X baseline, no distortion: rectification
        // must be (near) identity — rectified image ≈ raw image.
        let left = calib_cam(234.4, 320.0, 240.0);
        let right = calib_cam(234.4, 320.0, 240.0);
        let pose = Isometry3::from_parts(
            Translation3::new(0.103, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let rect = Rectifier::new(&left, &right, pose);
        // Rectified rig has the baseline and shared intrinsics.
        assert!((rect.rig.left_t_right.translation.vector.x - 0.103).abs() < 1e-9);
        assert!((rect.rig.left.fx - 234.4).abs() < 1e-9);

        // Feed an impulse image; rectified output should reproduce it at the
        // same location (identity remap).
        let mut img = vec![0u8; 640 * 480];
        img[240 * 640 + 320] = 255;
        let (ol, _or) = rect.apply(&img, &img);
        assert_eq!(ol[240 * 640 + 320], 255, "center pixel preserved");
        assert_eq!(ol[240 * 640 + 319], 0);
    }

    #[test]
    fn distortion_shifts_center_ray_back() {
        // A positive k1 pulls pixels outward; the rectified center pixel must
        // still map to a raw sample near the image center.
        let mut left = calib_cam(234.4, 320.0, 240.0);
        left.kc = [0.05, 0.0, 0.0, 0.0, 0.0];
        let right = left;
        let pose = Isometry3::from_parts(
            Translation3::new(0.103, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let rect = Rectifier::new(&left, &right, pose);
        let idx = (240 * 640 + 320) as usize;
        let (sx, sy) = (rect.map_x_l[idx], rect.map_y_l[idx]);
        // Center maps to center (distortion is zero at r=0).
        assert!((sx - 320.0).abs() < 1e-3, "sx={sx}");
        assert!((sy - 240.0).abs() < 1e-3, "sy={sy}");
        // Off-center, positive k1 (barrel) ⇒ the raw sample for a rectified
        // pixel above center is further from center than the destination
        // pixel (undistortion pushes pixels outward): src_y < 160.
        let idx2 = (160 * 640 + 320) as usize;
        let sy2 = rect.map_y_l[idx2];
        assert!(sy2 < 160.0, "k1>0 barrel: dest 160 → src {sy2}");
    }

    #[test]
    fn coverage_is_reasonable() {
        let left = calib_cam(234.4, 320.0, 240.0);
        let right = calib_cam(234.4, 320.0, 240.0);
        let pose = Isometry3::from_parts(
            Translation3::new(0.103, 0.0, 0.0),
            UnitQuaternion::identity(),
        );
        let rect = Rectifier::new(&left, &right, pose);
        let cov = rect.coverage();
        assert!(cov > 0.98, "coverage {cov}");
    }
}
