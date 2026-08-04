//! Inter-frame motion estimation (M4).
//!
//! Pipeline refinement vs the approved plan (spec Appendix D): the plan said
//! "PnP + RANSAC". DLT-PnP degenerates on planar scenes (our synthetic scene
//! is a plane) and the pipeline already has **metric stereo depth in both
//! frames**, so motion is estimated as rigid 3D–3D alignment: RANSAC over
//! 3-point Umeyama samples, validated by reprojection (3D–2D, into the B
//! camera), then **Gauss–Newton refinement against reprojection error** over
//! the inliers — the classic VO final step that removes most depth-noise bias
//! (2D pixels are far more accurate than stereo depths).

use nalgebra::{Isometry3, Matrix3, Point2, Point3, Translation3, UnitQuaternion, Vector2, Vector3};

use super::camera::{CameraModel, StereoRig};
use super::klt::TrackedPoint;
use super::stereo::StereoMatcher;

/// Result of a motion estimate.
#[derive(Debug, Clone)]
pub struct MotionEstimate {
    /// `A_T_B`: maps points in the keyframe (A) camera frame into the current
    /// (B) camera frame.
    pub pose: Isometry3<f64>,
    /// RANSAC inliers / total correspondences.
    pub inliers: usize,
    pub total: usize,
}

/// Closed-form rigid alignment (Horn's method / Umeyama): finds `R, t`
/// minimizing `Σ ||R·src_i + t − dst_i||²`. Requires ≥ 3 points.
pub fn umeyama(src: &[Point3<f64>], dst: &[Point3<f64>]) -> Option<(Matrix3<f64>, Vector3<f64>)> {
    if src.len() < 3 || src.len() != dst.len() {
        return None;
    }
    let n = src.len() as f64;
    let src_mean = src.iter().fold(Point3::origin(), |acc, p| acc + p.coords) / n;
    let dst_mean = dst.iter().fold(Point3::origin(), |acc, p| acc + p.coords) / n;

    let mut cov = Matrix3::zeros();
    for (s, d) in src.iter().zip(dst.iter()) {
        let s0 = s - src_mean;
        let d0 = d - dst_mean;
        cov += d0 * s0.transpose();
    }

    let svd = cov.svd(true, true);
    let (u, vt) = (svd.u?, svd.v_t?);
    let mut r = u * vt;
    if r.determinant() < 0.0 {
        let mut uf = u;
        uf.column_mut(2).scale_mut(-1.0);
        r = uf * vt;
    }
    let t = dst_mean.coords - r * src_mean.coords;
    Some((r, t))
}

/// Gauss–Newton refinement of a pose against 3D–2D reprojection error.
///
/// Parameterization: `p = R(ω)·P + t` with `ω` the rotation vector and
/// `R = exp([ω]×)`. Jacobians: `dp/dω_k = e_k × (R·P)`, `dp/dt_k = e_k`,
/// chained through the pinhole projection.
fn refine_reprojection(
    src3d: &[Point3<f64>],
    dst2d: &[Point2<f64>],
    cam: &CameraModel,
    init: &Isometry3<f64>,
    iterations: usize,
) -> Isometry3<f64> {
    let mut omega = init.rotation.scaled_axis();
    let mut t = init.translation.vector;
    for _ in 0..iterations {
        let r = UnitQuaternion::from_scaled_axis(omega);
        let mut jtj = nalgebra::Matrix6::<f64>::zeros();
        let mut jtr = nalgebra::Vector6::<f64>::zeros();
        let mut residual_norm = 0.0;
        for i in 0..src3d.len() {
            let p = r * src3d[i].coords + t;
            if p.z <= 1e-9 {
                continue;
            }
            let px = Point2::new(cam.fx * p.x / p.z + cam.cx, cam.fy * p.y / p.z + cam.cy);
            let res = Vector2::new(px.x - dst2d[i].x, px.y - dst2d[i].y);
            residual_norm += res.norm_squared();

            // ∂u/∂p, ∂v/∂p (pinhole).
            let du = Vector3::new(cam.fx / p.z, 0.0, -cam.fx * p.x / (p.z * p.z));
            let dv = Vector3::new(0.0, cam.fy / p.z, -cam.fy * p.y / (p.z * p.z));
            // dp/dω_k = e_k × (R·P)
            let rp = r * src3d[i].coords;
            let mut j = nalgebra::Matrix2x6::<f64>::zeros();
            for k in 0..3 {
                let d_rot = match k {
                    0 => Vector3::new(0.0, -rp.z, rp.y),
                    1 => Vector3::new(rp.z, 0.0, -rp.x),
                    _ => Vector3::new(-rp.y, rp.x, 0.0),
                };
                j[(0, k)] = du.dot(&d_rot);
                j[(1, k)] = dv.dot(&d_rot);
                // dp/dt_k = e_k → column k+3 is just du/dv components.
                j[(0, k + 3)] = du[k];
                j[(1, k + 3)] = dv[k];
            }
            jtj += j.transpose() * j;
            jtr += j.transpose() * res;
        }
        let delta = match jtj.lu().solve(&(-jtr)) {
            Some(d) => d,
            None => break,
        };
        omega += delta.fixed_rows::<3>(0);
        t += delta.fixed_rows::<3>(3);
        if delta.norm() < 1e-8 {
            break;
        }
        let _ = residual_norm;
    }
    Isometry3::from_parts(Translation3::from(t), UnitQuaternion::from_scaled_axis(omega))
}

/// RANSAC rigid-motion estimation from 3D–3D correspondences (Umeyama
/// minimal solver) validated by reprojection into the `cam` image, refined
/// by Gauss–Newton over the inlier set.
///
/// Returns `None` when fewer than `min_inliers` correspondences survive
/// reprojection validation — a degenerate estimate (bas-relief / low
/// texture / motion blur) must not be trusted, otherwise a single bad frame
/// injects explosive pose drift into the accumulated trajectory (M9).
pub fn ransac_motion(
    src3d: &[Point3<f64>],
    dst3d: &[Point3<f64>],
    dst2d: &[Point2<f64>],
    cam: &CameraModel,
    iterations: usize,
    threshold_px: f64,
    min_inliers: usize,
) -> Option<MotionEstimate> {
    let n = src3d.len();
    if n < 3 {
        return None;
    }
    // Deterministic RNG (LCG) — reproducible tests.
    let mut state = 0x9E3779B97F4A7C15u64;
    let mut rand = move || {
        state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (state >> 33) as usize % n
    };

    let reproj_err = |r: &Matrix3<f64>, t: &Vector3<f64>, i: usize| -> f64 {
        let p_b = r * src3d[i].coords + t;
        match cam.project(&Point3::from(p_b)) {
            Some(p) => (p - dst2d[i]).norm(),
            None => f64::INFINITY,
        }
    };

    let mut best: Option<(Matrix3<f64>, Vector3<f64>, usize)> = None;
    for _ in 0..iterations {
        let (i, j, k) = (rand(), rand(), rand());
        if i == j || i == k || j == k {
            continue;
        }
        let Some((r, t)) = umeyama(&[src3d[i], src3d[j], src3d[k]], &[dst3d[i], dst3d[j], dst3d[k]])
        else {
            continue;
        };
        let inliers = (0..n).filter(|&m| reproj_err(&r, &t, m) < threshold_px).count();
        if best.as_ref().is_none_or(|(_, _, b)| inliers > *b) {
            best = Some((r, t, inliers));
        }
    }
    let (r, t, _) = best?;

    // Re-estimate from all inliers, then refine against reprojection error.
    let idx: Vec<usize> = (0..n).filter(|&m| reproj_err(&r, &t, m) < threshold_px).collect();
    if idx.len() < min_inliers {
        return None;
    }
    let (rf, tf) = umeyama(
        &idx.iter().map(|&m| src3d[m]).collect::<Vec<_>>(),
        &idx.iter().map(|&m| dst3d[m]).collect::<Vec<_>>(),
    )?;
    let coarse = Isometry3::from_parts(Translation3::from(tf), UnitQuaternion::from_matrix(&rf));
    // Refine against the INLIER subset only — the outliers would dominate the
    // Gauss–Newton normal equations.
    let inlier_src: Vec<Point3<f64>> = idx.iter().map(|&m| src3d[m]).collect();
    let inlier_px: Vec<Point2<f64>> = idx.iter().map(|&m| dst2d[m]).collect();
    let fine = refine_reprojection(&inlier_src, &inlier_px, cam, &coarse, 20);
    let inliers = (0..n)
        .filter(|&m| {
            let p_b = fine * src3d[m];
            match cam.project(&p_b) {
                Some(p) => (p - dst2d[m]).norm() < threshold_px,
                None => false,
            }
        })
        .count();
    if inliers < min_inliers {
        return None;
    }
    Some(MotionEstimate {
        pose: fine,
        inliers,
        total: n,
    })
}

/// Full inter-frame motion estimate:
/// features in the keyframe A → stereo depth in A (3D_A); KLT tracks into
/// the current frame B → stereo depth in B (3D_B); RANSAC + refinement.
#[allow(clippy::too_many_arguments)]
pub fn estimate_motion(
    rig: &StereoRig,
    matcher: &StereoMatcher,
    left_a: &[u8],
    right_a: &[u8],
    left_b: &[u8],
    right_b: &[u8],
    features_a: &[Point2<f64>],
    tracked_b: &[TrackedPoint],
) -> Option<MotionEstimate> {
    let mut src = Vec::new();
    let mut dst = Vec::new();
    let mut px_b = Vec::new();
    for (f, t) in features_a.iter().zip(tracked_b.iter()) {
        if !t.ok {
            continue;
        }
        // Skip (not abort) features that fail stereo matching — they are
        // RANSAC outliers-to-be, not reasons to give up the frame.
        let (da, _) = match matcher.match_feature(rig, left_a, right_a, f) {
            Some(m) => m,
            None => continue,
        };
        let pa = match matcher.triangulate(rig, f, da) {
            Some(p) => p,
            None => continue,
        };
        let (db, _) = match matcher.match_feature(rig, left_b, right_b, &t.point) {
            Some(m) => m,
            None => continue,
        };
        let pb = match matcher.triangulate(rig, &t.point, db) {
            Some(p) => p,
            None => continue,
        };
        src.push(pa);
        dst.push(pb);
        px_b.push(t.point);
    }
    if src.len() < 6 {
        return None;
    }
    // M9: reject degenerate estimates — require the inlier set to be a
    // meaningful fraction of the correspondences (2.5%), with an absolute
    // floor. Measured on real scenes (2026-08-04): garbage poses have
    // inliers 1–29, valid poses ≥30; a gate of ~20–30 cleanly separates
    // them. Frames that fail (bas-relief, motion blur, low texture) return
    // None and the pipeline keeps the previous keyframe instead of injecting
    // explosive drift into the trajectory.
    let min_inliers = (src.len() / 40).max(20);
    // M9 drift tuning (2026-08-04, measured on live still headset): the old
    // 300 iters / 3.0 px threshold let near-degenerate solutions through —
    // systematic dz bias (mean +0.027 m/step) and ~15° yaw drift. 2000 iters
    // / 1.5 px cut path 4.8 m → 1.46 m (−70%) and endpoint drift 1.3 m →
    // 0.21 m (−83%); the dz bias vanished. Tighter threshold rejects more
    // frames (26 vs 45 posed/20 s), but the min-inlier gate above turns those
    // into None — the pipeline keeps the prior keyframe pose instead of
    // injecting jitter. Starvation is safe; looseness is not.
    ransac_motion(&src, &dst, &px_b, &rig.left, 2000, 1.5, min_inliers)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::detect_corners_fast;
    use crate::klt::klt_track;
    use crate::synthetic;
    use crate::synthetic::forward_trajectory;

    #[test]
    fn umeyama_recovers_known_transform() {
        let r_true = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let t_true = Vector3::new(0.03, -0.02, 0.1);
        let mut src = Vec::new();
        let mut dst = Vec::new();
        for i in 0..20 {
            let p = Point3::new(
                (i as f64 * 1.3).sin() * 2.0,
                (i as f64 * 2.1).cos() * 1.5,
                (i as f64 * 0.7).sin() * 0.5 + 1.0,
            );
            dst.push(Point3::from(r_true * p.coords + t_true));
            src.push(p);
        }
        let (r, t) = umeyama(&src, &dst).unwrap();
        let r_est = UnitQuaternion::from_matrix(&r);
        assert!((r_est.angle() - 0.2).abs() < 1e-9, "angle {}", r_est.angle());
        assert!((t - t_true).norm() < 1e-9);
    }

    /// End-to-end: render two frames with a known inter-frame motion, detect
    /// + track + stereo-match + estimate, compare to ground truth.
    #[test]
    fn motion_estimate_matches_ground_truth() {
        let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let plane_z = 1.5;
        // Forward translation + small yaw: the well-constrained regime.
        // (Lateral translation at small rotation suffers the bas-relief
        // ambiguity — weakly observable from reprojection alone; IMU fusion
        // in P2b resolves it.)
        let gt_pose = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.04),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.01),
        );
        let (fa, _) = synthetic::render(&rig, &Isometry3::identity(), plane_z);
        let (fb, _) = synthetic::render(&rig, &gt_pose, plane_z);

        let features = detect_corners_fast(&fa.left, 640, 480, 20);
        let features: Vec<Point2<f64>> = features
            .into_iter()
            .filter(|c| c.x > 30.0 && c.x < 610.0 && c.y > 30.0 && c.y < 450.0)
            .collect();
        assert!(features.len() > 300, "{} features", features.len());

        let tracked = klt_track(&fa.left, &fb.left, 640, 480, &features, 5, 12);
        let matcher = StereoMatcher::new(5, 0.5, 10.0);
        let est = estimate_motion(
            &rig, &matcher, &fa.left, &fa.right, &fb.left, &fb.right, &features, &tracked,
        )
        .expect("motion estimate");

        let t_err = (est.pose.translation.vector - gt_pose.translation.vector).norm();
        let r_err = est.pose.rotation.angle_to(&gt_pose.rotation);
        // Inlier RATE at a fixed px threshold is a threshold artifact (depth
        // noise scales with off-center distance); accuracy is what matters,
        // and the trajectory test validates it end-to-end.
        assert!(est.inliers >= 300, "too few inliers {}/{}", est.inliers, est.total);
        assert!(t_err < 0.005, "translation error {t_err} m");
        assert!(r_err < 0.005, "rotation error {r_err} rad");
    }

    #[test]
    fn trajectory_recovery_forward_motion() {
        let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let matcher = StereoMatcher::new(5, 0.5, 10.0);
        let traj = forward_trajectory(5, 0.05, 0.5);
        let mut poses: Vec<Isometry3<f64>> = Vec::new();
        for pair in traj.windows(2) {
            let (fa, _) = synthetic::render(&rig, &pair[0], 1.5);
            let (fb, _) = synthetic::render(&rig, &pair[1], 1.5);
            let features = detect_corners_fast(&fa.left, 640, 480, 20);
            let features: Vec<Point2<f64>> = features
                .into_iter()
                .filter(|c| c.x > 40.0 && c.x < 600.0 && c.y > 40.0 && c.y < 440.0)
                .collect();
            let tracked = klt_track(&fa.left, &fb.left, 640, 480, &features, 5, 12);
            let est = estimate_motion(
                &rig, &matcher, &fa.left, &fa.right, &fb.left, &fb.right, &features, &tracked,
            );
            let prev = if poses.is_empty() { pair[0] } else { *poses.last().unwrap() };
            let next = est.map(|e| prev * e.pose).unwrap_or(prev);
            poses.push(next);
            let t_err = (next.translation.vector - pair[1].translation.vector).norm();
            assert!(t_err < 0.02, "step {} translation error {t_err}", poses.len());
        }
        let end = *poses.last().unwrap();
        assert!(
            (end.translation.vector.z - 0.2).abs() < 0.03,
            "final z {} (expected ~0.2)",
            end.translation.vector.z
        );
    }
}
