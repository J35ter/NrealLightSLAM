//! Pinhole camera model and rectified stereo rig.
//!
//! M1 scope: undistorted pinhole projection (the synthetic renderer and all
//! M1–M4 math use it). Real Nreal Light fisheye intrinsics + rectification
//! maps are a hardware-spike item (M7, spec Appendix D.4) and will slot in
//! behind the same `project`/`unproject` API.

use nalgebra::{Isometry3, Point2, Point3, Translation3, UnitQuaternion, Vector3};

/// A pinhole camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraModel {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub width: u32,
    pub height: u32,
}

impl CameraModel {
    pub fn new(fx: f64, fy: f64, cx: f64, cy: f64, width: u32, height: u32) -> Self {
        CameraModel { fx, fy, cx, cy, width, height }
    }

    /// Project a point expressed in the camera frame to pixels.
    /// Returns `None` for points at or behind the camera (`z <= 0`).
    pub fn project(&self, p: &Point3<f64>) -> Option<Point2<f64>> {
        if p.z <= 0.0 {
            return None;
        }
        Some(Point2::new(self.fx * p.x / p.z + self.cx, self.fy * p.y / p.z + self.cy))
    }

    /// Is the pixel inside the image?
    pub fn contains(&self, px: &Point2<f64>) -> bool {
        px.x >= 0.0 && px.x < self.width as f64 && px.y >= 0.0 && px.y < self.height as f64
    }

    /// Inverse projection: the 3D point (in the camera frame) that projects
    /// to `px` at the given depth `z`.
    pub fn unproject(&self, px: &Point2<f64>, z: f64) -> Point3<f64> {
        Point3::new((px.x - self.cx) * z / self.fx, (px.y - self.cy) * z / self.fy, z)
    }

    /// Unit ray direction (camera frame) through a pixel.
    pub fn ray(&self, px: &Point2<f64>) -> Vector3<f64> {
        Vector3::new((px.x - self.cx) / self.fx, (px.y - self.cy) / self.fy, 1.0)
    }
}

/// A stereo pair of pinhole cameras.
#[derive(Debug, Clone)]
pub struct StereoRig {
    pub left: CameraModel,
    pub right: CameraModel,
    /// Pose of the **right** camera in the **left** camera frame:
    /// `p_left = left_t_right * p_right`. (Matrix notation `A_T_B` — a
    /// transform from frame B to frame A — is written snake_case here for
    /// the linter.)
    pub left_t_right: Isometry3<f64>,
}

impl StereoRig {
    /// Canonical rectified rig: identical intrinsics, the right camera
    /// `baseline` meters to the right of the left one, no rotation.
    /// For a point at depth `z`, `u_right = u_left - fx*baseline/z`.
    pub fn rectified(
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        baseline: f64,
        width: u32,
        height: u32,
    ) -> Self {
        let cam = CameraModel::new(fx, fy, cx, cy, width, height);
        StereoRig {
            left: cam,
            right: cam,
            left_t_right: Isometry3::from_parts(
                Translation3::new(baseline, 0.0, 0.0),
                UnitQuaternion::identity(),
            ),
        }
    }

    /// Project a point in the left-camera frame into both images:
    /// `(left_px, right_px)`.
    pub fn project_point(&self, p_left: &Point3<f64>) -> (Option<Point2<f64>>, Option<Point2<f64>>) {
        let l = self.left.project(p_left);
        let r = self.right.project(&(self.left_t_right.inverse() * p_left));
        (l, r)
    }

    /// Horizontal disparity (px) of a point at left-camera depth `z`:
    /// `u_right = u_left - disparity`.
    pub fn disparity_at(&self, z: f64) -> f64 {
        self.left.fx * self.left_t_right.translation.vector.x / z
    }

    /// Metric scale: `z = fx * baseline / disparity`.
    pub fn depth_from_disparity(&self, disparity: f64) -> Option<f64> {
        let d = self.left.fx * self.left_t_right.translation.vector.x;
        if disparity > 0.0 && d > 0.0 {
            Some(d / disparity)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn project_unproject_roundtrip() {
        let cam = CameraModel::new(500.0, 500.0, 320.0, 240.0, 640, 480);
        let p = Point3::new(0.1, -0.2, 1.5);
        let px = cam.project(&p).unwrap();
        let back = cam.unproject(&px, p.z);
        assert!((back.x - p.x).abs() < 1e-9 && (back.y - p.y).abs() < 1e-9);
        assert!(cam.contains(&px));
    }

    #[test]
    fn project_rejects_behind_camera() {
        let cam = CameraModel::new(500.0, 500.0, 320.0, 240.0, 640, 480);
        assert!(cam.project(&Point3::new(0.0, 0.0, -1.0)).is_none());
        assert!(cam.project(&Point3::new(0.0, 0.0, 0.0)).is_none());
    }

    #[test]
    fn rectified_rig_disparity_geometry() {
        let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        // A point dead ahead at 1.5 m: same row, shifted left in the right cam.
        let p_left = Point3::new(0.0, 0.0, 1.5);
        let (l, r) = rig.project_point(&p_left);
        let (l, r) = (l.unwrap(), r.unwrap());
        assert_eq!(l.y, r.y); // epipolar alignment (rectified)
        let expected = rig.disparity_at(1.5);
        assert!((l.x - r.x - expected).abs() < 1e-9, "l={l:?} r={r:?} d={expected}");
        // depth from disparity round-trips.
        let z = rig.depth_from_disparity(rig.disparity_at(1.5)).unwrap();
        assert!((z - 1.5).abs() < 1e-9);
    }
}
