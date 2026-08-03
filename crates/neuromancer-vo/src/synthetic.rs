//! Synthetic stereo renderer: a textured fronto-parallel plane rendered from
//! known camera poses. Provides ground-truth depth (per-pixel) and
//! ground-truth camera trajectory — the CI-equivalent for VO accuracy in
//! M3/M4 (spec Appendix D.2) without needing the glasses.

use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion, Vector3};

use super::camera::StereoRig;
use super::frame::{DepthMap, StereoFrame};

/// Deterministic aperiodic grayscale texture: smooth value noise (fine +
/// coarse octaves) PLUS a hard-quantized octave — real scenes have both soft
/// texture and sharp edges, and FAST needs the latter. Periodic sine textures
/// would create near-degenerate LK minima. Any `(x, y)` plane coordinate
/// (meters) → 0..255.
pub fn texture(x: f64, y: f64) -> u8 {
    let n1 = value_noise(x / 0.03, y / 0.03);
    let n2 = value_noise(x / 0.12, y / 0.12);
    // 5-level quantization of a fine octave: hard edges ~6.5 px apart.
    let hard = (value_noise(x / 0.02, y / 0.02) * 5.0).floor() / 5.0;
    ((n1 * 0.3 + n2 * 0.2 + hard * 0.5) * 255.0).clamp(0.0, 255.0) as u8
}

/// Deterministic hash of integer lattice coordinates → [0, 1).
fn hash2(x: f64, y: f64) -> f64 {
    let s = (x * 12.9898 + y * 78.233).sin() * 43758.5453;
    s - s.floor()
}

/// Smooth interpolated value noise (bilinear + smoothstep), range [0, 1).
fn value_noise(x: f64, y: f64) -> f64 {
    let xi = x.floor();
    let yi = y.floor();
    let xf = x - xi;
    let yf = y - yi;
    let smooth = |t: f64| t * t * (3.0 - 2.0 * t);
    let u = smooth(xf);
    let v = smooth(yf);
    let a = hash2(xi, yi);
    let b = hash2(xi + 1.0, yi);
    let c = hash2(xi, yi + 1.0);
    let d = hash2(xi + 1.0, yi + 1.0);
    let ab = a + (b - a) * u;
    let cd = c + (d - c) * u;
    ab + (cd - ab) * v
}

/// Render the textured plane at world `z = plane_z` from a camera with pose
/// `world_T_left` (world → left camera). Returns the stereo pair plus the
/// left-image depth map (left-camera z, meters; `INFINITY` = no depth).
///
/// The right frame is painted by projecting each *left* pixel's plane point
/// into the right camera — geometrically consistent (shared 3D points), no
/// occlusion modeling (fine for features/geometry tests).
pub fn render(
    rig: &StereoRig,
    #[allow(non_snake_case)] world_T_left: &Isometry3<f64>,
    plane_z: f64,
) -> (StereoFrame, DepthMap) {
    let (w, h) = (rig.left.width, rig.left.height);
    let mut left = vec![0u8; (w * h) as usize];
    let mut right = vec![0u8; (w * h) as usize];
    let mut depth = vec![f64::INFINITY; (w * h) as usize];

    #[allow(non_snake_case)]
    let left_T_world = world_T_left.inverse();
    let cam_world = left_T_world.translation.vector;
    let rot_world = left_T_world.rotation;

    for v in 0..h {
        for u in 0..w {
            // Ray through the pixel (camera frame), intersect the world plane.
            let dir_cam = Vector3::new(
                (u as f64 - rig.left.cx) / rig.left.fx,
                (v as f64 - rig.left.cy) / rig.left.fy,
                1.0,
            );
            let dir_world = rot_world * dir_cam;
            if dir_world.z.abs() < 1e-9 {
                continue;
            }
            let t = (plane_z - cam_world.z) / dir_world.z;
            if t <= 0.0 {
                continue;
            }
            let p_world = cam_world + t * dir_world;
            let p_left = world_T_left * Point3::from(p_world);
            let color = texture(p_world.x, p_world.y);
            let idx = (v as usize) * w as usize + u as usize;
            left[idx] = color;
            depth[idx] = p_left.z;

            // Same 3D point into the right camera.
            if let (_, Some(rp)) = rig.project_point(&p_left) {
                let (ru, rv) = (rp.x.round() as i64, rp.y.round() as i64);
                if ru >= 0 && ru < w as i64 && rv >= 0 && rv < h as i64 {
                    right[rv as usize * w as usize + ru as usize] = color;
                }
            }
        }
    }

    (StereoFrame::new(w, h, left, right), DepthMap::new(w, h, depth))
}

/// A forward trajectory: the camera translates `+Z` (toward the plane) by
/// `step_m` per step with a small per-step rotation about world Y (yaw).
/// Ground truth for M4 pose tests.
pub fn forward_trajectory(steps: usize, step_m: f64, yaw_deg: f64) -> Vec<Isometry3<f64>> {
    (0..steps)
        .map(|i| {
            let z = i as f64 * step_m;
            let yaw = (i as f64 * yaw_deg).to_radians();
            Isometry3::from_parts(
                Translation3::new(0.0, 0.0, z),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), yaw),
            )
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Point2;

    const FX: f64 = 500.0;
    const BASELINE: f64 = 0.12;
    const PLANE_Z: f64 = 1.5;

    fn rig() -> StereoRig {
        StereoRig::rectified(FX, 500.0, 320.0, 240.0, BASELINE, 640, 480)
    }

    /// Depth map matches the analytic ray–plane intersection.
    #[test]
    fn depth_matches_ray_plane() {
        let (_, depth) = render(&rig(), &Isometry3::identity(), PLANE_Z);
        let px = Point2::new(320.0, 240.0); // center ray: straight ahead
        let ray = rig().left.ray(&px);
        let t = PLANE_Z / ray.z;
        let expected = t; // depth along the camera z-axis for the identity pose
        let got = depth.z[240 * 640 + 320];
        assert!((got - expected).abs() < 1e-9, "got {got}, expected {expected}");
        assert!(depth.z[0].is_finite()); // corner pixels also hit the plane
    }

    /// The right frame is the left frame shifted left by the disparity:
    /// a plane point's color appears at `u_left` and `u_left - d`.
    #[test]
    fn right_frame_matches_disparity() {
        let (frame, depth) = render(&rig(), &Isometry3::identity(), PLANE_Z);
        let d = rig().disparity_at(PLANE_Z);
        // Pick pixels away from the borders; colors must agree along the
        // horizontal epipolar line (same v).
        let mut matched = 0usize;
        let mut total = 0usize;
        for v in (100..380).step_by(7) {
            for u in (100..500).step_by(5) {
                let idx = v * 640 + u;
                if !depth.z[idx].is_finite() {
                    continue;
                }
                total += 1;
                let ru = u as f64 - d;
                if !(0.0..640.0).contains(&ru) {
                    continue;
                }
                // Tolerance of a pixel or two for the discrete splatting.
                let ru_int = ru.round() as i64;
                let mut best = false;
                for cand in (ru_int - 1)..=(ru_int + 1) {
                    if (0..640).contains(&cand) {
                        let ri = v * 640 + cand as usize;
                        if frame.right[ri] == frame.left[idx] {
                            best = true;
                        }
                    }
                }
                if best {
                    matched += 1;
                }
            }
        }
        // Most sampled pixels should have a matching right pixel.
        assert!(
            matched >= total * 9 / 10,
            "only {matched}/{total} left pixels matched the right frame"
        );
    }

    /// The trajectory is metric: consecutive poses differ by exactly step_m.
    #[test]
    fn trajectory_is_metric() {
        let traj = forward_trajectory(5, 0.05, 0.0);
        for pair in traj.windows(2) {
            let dz = pair[1].translation.vector.z - pair[0].translation.vector.z;
            assert!((dz - 0.05).abs() < 1e-12);
        }
    }

    /// The plane texture is feature-rich (enough contrast for FAST in M2).
    #[test]
    fn texture_has_contrast() {
        let mut vals = std::collections::HashSet::new();
        for i in 0..2000 {
            vals.insert(texture((i as f64) * 0.13, (i as f64) * 0.17));
        }
        assert!(vals.len() > 100, "only {} distinct gray levels", vals.len());
    }
}
