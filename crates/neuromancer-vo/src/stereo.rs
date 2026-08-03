//! Epipolar stereo matching + metric triangulation (M3).
//!
//! The rig is rectified (spec Appendix D), so a left-image feature's match
//! lies on the same row in the right image, shifted left by the disparity
//! `d = fx*baseline/depth`. SSD window search over a depth-bounded disparity
//! range, parabolic subpixel refinement, then metric depth
//! `z = fx*baseline/d`.

use nalgebra::{Point2, Point3};

use super::camera::StereoRig;

/// Stereo matcher configuration.
pub struct StereoMatcher {
    /// Half-size of the SSD matching window (a `(2*window_half+1)²` patch).
    pub window_half: i32,
    /// Depth search bounds (left-camera z, meters).
    pub min_depth: f64,
    pub max_depth: f64,
}

impl StereoMatcher {
    pub fn new(window_half: i32, min_depth: f64, max_depth: f64) -> Self {
        StereoMatcher { window_half, min_depth, max_depth }
    }

    /// Disparity search range (px) implied by the depth bounds.
    fn disparity_range(&self, rig: &StereoRig) -> (i32, i32) {
        let fxb = rig.left.fx * rig.left_t_right.translation.vector.x;
        let d_max = (fxb / self.min_depth).ceil().max(1.0) as i32;
        let d_min = (fxb / self.max_depth).floor().max(1.0) as i32;
        (d_min, d_max.max(d_min))
    }

    /// SSD cost of matching left pixel `(u, v)` with right pixel `(u - d, v)`.
    #[allow(clippy::too_many_arguments)]
    fn ssd(
        left: &[u8],
        right: &[u8],
        w: i32,
        h: i32,
        u: i32,
        v: i32,
        d: i32,
        window_half: i32,
    ) -> f64 {
        let mut sum = 0.0;
        for j in -window_half..=window_half {
            let ly = v + j;
            if ly < 0 || ly >= h {
                return f64::INFINITY;
            }
            for i in -window_half..=window_half {
                let lx = u + i;
                let rx = u - d + i;
                if lx < 0 || lx >= w || rx < 0 || rx >= w {
                    return f64::INFINITY;
                }
                let dl = left[ly as usize * w as usize + lx as usize] as f64;
                let dr = right[ly as usize * w as usize + rx as usize] as f64;
                let e = dl - dr;
                sum += e * e;
            }
        }
        sum
    }

    /// Match one left-image feature in the right image. Returns
    /// `(disparity_px, cost)` on success, `None` when no valid match exists
    /// within the depth range (occlusion / border / low texture).
    pub fn match_feature(
        &self,
        rig: &StereoRig,
        left: &[u8],
        right: &[u8],
        f: &Point2<f64>,
    ) -> Option<(f64, f64)> {
        let (w, h) = (rig.left.width as i32, rig.left.height as i32);
        let (d_min, d_max) = self.disparity_range(rig);
        let u = f.x.round() as i32;
        let v = f.y.round() as i32;

        let mut best_d = -1;
        let mut best_cost = f64::INFINITY;
        for d in d_min..=d_max {
            let cost = Self::ssd(left, right, w, h, u, v, d, self.window_half);
            if cost < best_cost {
                best_cost = cost;
                best_d = d;
            }
        }
        if best_d < 0 || best_cost.is_infinite() {
            return None;
        }

        // Parabolic subpixel refinement on the cost curve.
        let c0 = best_cost;
        let cm = Self::ssd(left, right, w, h, u, v, best_d - 1, self.window_half);
        let cp = Self::ssd(left, right, w, h, u, v, best_d + 1, self.window_half);
        let denom = cm - 2.0 * c0 + cp;
        let sub = if denom.abs() > 1e-12 {
            (cm - cp) / (2.0 * denom)
        } else {
            0.0
        };
        Some((best_d as f64 + sub.clamp(-0.5, 0.5), c0))
    }

    /// Match many features; output aligns with the input (None = no match).
    pub fn match_features(
        &self,
        rig: &StereoRig,
        left: &[u8],
        right: &[u8],
        features: &[Point2<f64>],
    ) -> Vec<Option<(f64, f64)>> {
        features
            .iter()
            .map(|f| self.match_feature(rig, left, right, f))
            .collect()
    }

    /// Triangulate a matched feature into the left-camera frame (meters).
    /// `disparity` in px; returns the 3D point or `None` when the disparity is
    /// non-positive (behind/at infinity).
    pub fn triangulate(&self, rig: &StereoRig, f: &Point2<f64>, disparity: f64) -> Option<Point3<f64>> {
        let z = rig.depth_from_disparity(disparity)?;
        Some(rig.left.unproject(f, z))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::features::detect_corners_fast;
    use crate::synthetic;

    fn rig() -> StereoRig {
        StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480)
    }

    #[test]
    fn triangulation_metric_depth() {
        let rig = rig();
        let matcher = StereoMatcher::new(5, 0.5, 10.0);
        // Point dead ahead at 1.5 m → known disparity fx*b/z.
        let d = rig.disparity_at(1.5);
        let p = matcher
            .triangulate(&rig, &Point2::new(320.0, 240.0), d)
            .unwrap();
        assert!((p.z - 1.5).abs() < 1e-9, "z = {}", p.z);
        assert!(p.x.abs() < 1e-9 && p.y.abs() < 1e-9);
        // Non-positive disparity → None.
        assert!(matcher.triangulate(&rig, &Point2::new(320.0, 240.0), 0.0).is_none());
    }

    /// Matched depth must match the renderer's ground-truth depth map.
    #[test]
    fn stereo_depth_matches_ground_truth() {
        let rig = rig();
        let (frame, depth) = synthetic::render(&rig, &nalgebra::Isometry3::identity(), 1.5);
        let matcher = StereoMatcher::new(5, 0.5, 10.0);
        let corners = detect_corners_fast(&frame.left, 640, 480, 20);
        assert!(corners.len() > 200, "{} corners", corners.len());

        let mut rel_errs: Vec<f64> = Vec::new();
        for c in &corners {
            let (d, _) = match matcher.match_feature(&rig, &frame.left, &frame.right, c) {
                Some(m) => m,
                None => continue,
            };
            let Some(p) = matcher.triangulate(&rig, c, d) else {
                continue;
            };
            let gt = depth.z[c.y as usize * 640 + c.x as usize];
            if !gt.is_finite() {
                continue;
            }
            rel_errs.push((p.z - gt).abs() / gt);
        }
        assert!(rel_errs.len() > 200, "only {} matched", rel_errs.len());
        rel_errs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = rel_errs[rel_errs.len() / 2];
        let inliers = rel_errs.iter().filter(|e| **e < 0.05).count();
        assert!(
            median < 0.02,
            "median relative depth error {:.3} ({} matches)",
            median,
            rel_errs.len()
        );
        assert!(
            inliers * 10 >= rel_errs.len() * 9,
            "only {inliers}/{} within 5%",
            rel_errs.len()
        );
    }
}
