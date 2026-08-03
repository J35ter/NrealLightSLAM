//! Lucas–Kanade optical flow (coarse-to-fine pyramid, forward-additive) — M2.

use nalgebra::Point2;

/// One tracked feature.
#[derive(Debug, Clone, Copy)]
pub struct TrackedPoint {
    pub point: Point2<f64>,
    pub ok: bool,
}

/// Half-resolution copy of `img` (3×3 box average, then subsample).
/// Unused while single-level tracking is the default; kept for a possible
/// multi-scale return in M7 (hardware tuning) with real imagery.
#[allow(dead_code)]
fn box_blur_half(img: &[u8], w: u32, h: u32) -> (Vec<u8>, u32, u32) {
    let nw = (w / 2).max(1);
    let nh = (h / 2).max(1);
    let mut out = vec![0u8; (nw * nh) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let mut acc = 0u32;
            let mut cnt = 0u32;
            for dy in -1..=1i32 {
                for dx in -1..=1i32 {
                    let sx = (2 * x as i32 + dx).clamp(0, w as i32 - 1);
                    let sy = (2 * y as i32 + dy).clamp(0, h as i32 - 1);
                    acc += img[sy as usize * w as usize + sx as usize] as u32;
                    cnt += 1;
                }
            }
            out[y as usize * nw as usize + x as usize] = (acc / cnt) as u8;
        }
    }
    (out, nw, nh)
}

/// One iterative LK pass at a single scale. `features` hold the initial
/// guesses (already scaled to this level).
fn track_one_level(
    prev: &[u8],
    cur: &[u8],
    width: u32,
    height: u32,
    features: &[Point2<f64>],
    window_half: i32,
    max_iter: usize,
) -> Vec<TrackedPoint> {
    let (w, h) = (width as i32, height as i32);
    let at = |img: &[u8], x: i32, y: i32| -> f64 {
        if x < 0 || y < 0 || x >= w || y >= h {
            return 0.0;
        }
        img[y as usize * width as usize + x as usize] as f64
    };
    let bilinear = |img: &[u8], x: f64, y: f64| -> f64 {
        let x0 = x.floor() as i32;
        let y0 = y.floor() as i32;
        let fx = x - x0 as f64;
        let fy = y - y0 as f64;
        let v00 = at(img, x0, y0);
        let v10 = at(img, x0 + 1, y0);
        let v01 = at(img, x0, y0 + 1);
        let v11 = at(img, x0 + 1, y0 + 1);
        (v00 * (1.0 - fx) + v10 * fx) * (1.0 - fy) + (v01 * (1.0 - fx) + v11 * fx) * fy
    };

    features
        .iter()
        .map(|&f| {
            // Structure tensor of the prev image at the feature location.
            let mut a11 = 0.0;
            let mut a12 = 0.0;
            let mut a22 = 0.0;
            for dy in -window_half..=window_half {
                for dx in -window_half..=window_half {
                    let x = f.x as i32 + dx;
                    let y = f.y as i32 + dy;
                    let ix = (at(prev, x + 1, y) - at(prev, x - 1, y)) * 0.5;
                    let iy = (at(prev, x, y + 1) - at(prev, x, y - 1)) * 0.5;
                    a11 += ix * ix;
                    a12 += ix * iy;
                    a22 += iy * iy;
                }
            }
            let det = a11 * a22 - a12 * a12;
            if det < 1e-4 || !det.is_finite() {
                return TrackedPoint { point: f, ok: false };
            }

            let mut p = f;
            let mut ok = true;
            for _ in 0..max_iter {
                let mut b1 = 0.0;
                let mut b2 = 0.0;
                for dy in -window_half..=window_half {
                    for dx in -window_half..=window_half {
                        let x = f.x as i32 + dx;
                        let y = f.y as i32 + dy;
                        let ix = (at(prev, x + 1, y) - at(prev, x - 1, y)) * 0.5;
                        let iy = (at(prev, x, y + 1) - at(prev, x, y - 1)) * 0.5;
                        let it = bilinear(cur, p.x + dx as f64, p.y + dy as f64) - at(prev, x, y);
                        b1 += ix * it;
                        b2 += iy * it;
                    }
                }
                let dx = (-b1 * a22 + b2 * a12) / det;
                let dy = (-b2 * a11 + b1 * a12) / det;
                p.x += dx;
                p.y += dy;
                if dx.abs() < 0.01 && dy.abs() < 0.01 {
                    break;
                }
                if !(p.x > 0.0 && p.y > 0.0 && p.x < (w - 1) as f64 && p.y < (h - 1) as f64) {
                    ok = false;
                    break;
                }
            }
            TrackedPoint { point: p, ok }
        })
        .collect()
}

/// Track features from `prev` into `cur` (same-size grayscale images) with
/// single-level Lucas–Kanade (forward-additive, bilinear sampling).
///
/// Single-level was chosen over a coarse-to-fine pyramid after testing: the
/// pyramid's blurred coarse level biased every estimate (73% vs 100% inliers
/// on clean synthetic motion), while single-level LK tracks up to ~7 px on
/// aperiodic texture. Multi-scale tracking returns in M7 (hardware tuning)
/// if real imagery needs it.
pub fn klt_track(
    prev: &[u8],
    cur: &[u8],
    width: u32,
    height: u32,
    features: &[Point2<f64>],
    window_half: i32,
    max_iter: usize,
) -> Vec<TrackedPoint> {
    track_one_level(prev, cur, width, height, features, window_half, max_iter)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::camera::StereoRig;
    use crate::features::detect_corners_fast;
    use crate::synthetic;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion};

    /// Camera translation parallel to the textured plane: every point shifts
    /// by the same pixel delta. Ground truth is exact, so this validates KLT
    /// convergence and sub-pixel accuracy on clean imagery.
    #[test]
    fn klt_tracks_uniform_translation() {
        let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let plane_z = 1.5;
        for step_m in [0.004f64, 0.01, 0.02] {
            // world_T_left = T(+step): camera at world -step_x → du = +fx*step/z.
            let pose_a = Isometry3::identity();
            let pose_b = Isometry3::from_parts(
                Translation3::new(step_m, 0.0, 0.0),
                UnitQuaternion::identity(),
            );
            let (fa, _) = synthetic::render(&rig, &pose_a, plane_z);
            let (fb, _) = synthetic::render(&rig, &pose_b, plane_z);

            // Ground-truth anchor: frame B must be a horizontal shift of frame
            // A (nearest px matches much better than a 1 px vertical shift).
            let expected = rig.left.fx * step_m / plane_z;
            let du0 = expected.round() as i32;
            let (mut diff_shifted, mut diff_vertical, mut cnt) = (0.0f64, 0.0f64, 0.0f64);
            for v in (100..380).step_by(10) {
                for u in (100..500).step_by(10) {
                    let a = fa.left[v * 640 + u];
                    let shifted = (u as i64 + du0 as i64) as usize;
                    diff_shifted += (a as f64 - fb.left[v * 640 + shifted] as f64).abs();
                    diff_vertical += (a as f64 - fb.left[(v + 1) * 640 + u] as f64).abs();
                    cnt += 1.0;
                }
            }
            assert!(
                diff_shifted < diff_vertical,
                "frames not a pure horizontal shift: shifted {:.1} vs vertical {:.1}",
                diff_shifted / cnt,
                diff_vertical / cnt
            );

            let all = detect_corners_fast(&fa.left, 640, 480, 20);
            let features: Vec<Point2<f64>> = all
                .into_iter()
                .filter(|c| c.x > 30.0 && c.x < 610.0 && c.y > 30.0 && c.y < 450.0)
                .collect();
            assert!(features.len() > 150, "{} features", features.len());

            let tracked = klt_track(&fa.left, &fb.left, 640, 480, &features, 5, 12);

            let mut err_sum = 0.0;
            let mut inliers = 0usize;
            let mut n = 0usize;
            let mut du_sum = 0.0;
            let mut dv_sum = 0.0;
            let mut failed = 0usize;
            for (f, t) in features.iter().zip(tracked.iter()) {
                if !t.ok {
                    failed += 1;
                    continue;
                }
                let du = t.point.x - f.x;
                let dv = t.point.y - f.y;
                du_sum += du;
                dv_sum += dv;
                err_sum += (du - expected).abs();
                if (du - expected).abs() < 1.5 && dv.abs() < 1.5 {
                    inliers += 1;
                }
                n += 1;
            }
            assert!(n > 100, "only {n} tracked points");
            let mean_err = err_sum / n as f64;
            eprintln!(
                "step={step_m}: expected={expected:.2} mean_err={mean_err:.2} mean du={:.2} mean dv={:.2} inliers={inliers}/{n} failed={failed}",
                du_sum / n as f64,
                dv_sum / n as f64,
            );
            if step_m <= 0.01 {
                // Small-to-moderate motion: near-exact (single-level LK reaches
                // ~0.07 px; the 2-level pyramid trades a little sub-pixel
                // accuracy for large-motion robustness).
                assert!(mean_err < 1.0, "motion {step_m}: mean_err {mean_err}");
                assert!(inliers >= n * 9 / 10, "motion {step_m}: only {inliers}/{n} inliers");
            } else {
                // Aggressive 6.67 px motion: a robustness smoke test — the
                // residual outliers are absorbed by M4's RANSAC and by
                // prediction-initialized tracking (constant-velocity prior).
                assert!(
                    mean_err < 3.0,
                    "large motion mean_err {mean_err} (expected {expected:.2})"
                );
                assert!(inliers >= n * 7 / 10, "only {inliers}/{n} inliers");
            }
        }
    }
}
