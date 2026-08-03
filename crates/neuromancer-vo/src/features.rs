//! FAST-9 corner detection with non-maximum suppression (M2).

use nalgebra::Point2;

/// Bresenham circle of radius 3 around a pixel (16 points, starting at 12
/// o'clock, clockwise).
const CIRCLE: [(i32, i32); 16] = [
    (0, -3),
    (1, -3),
    (2, -2),
    (3, -1),
    (3, 0),
    (3, 1),
    (2, 2),
    (1, 3),
    (0, 3),
    (-1, 3),
    (-2, 2),
    (-3, 1),
    (-3, 0),
    (-3, -1),
    (-2, -2),
    (-1, -3),
];

/// Length of the longest contiguous run of `true` in the circular 16-tuple.
fn longest_run(hits: [bool; 16]) -> usize {
    let mut best = 0;
    let mut cur = 0;
    for i in 0..32 {
        if hits[i % 16] {
            cur += 1;
            best = best.max(cur);
        } else {
            cur = 0;
        }
    }
    best
}

/// Detect FAST-9 corners.
///
/// A pixel is a corner when at least 9 of the 16 circle pixels are
/// contiguously all brighter than `center + threshold` or all darker than
/// `center - threshold`. Corners are then non-maximum-suppressed on their
/// score (sum of |circle pixel − center|), keeping local maxima.
///
/// Returns integer-precision corner positions (as f64 points) sorted by
/// strength (strongest first).
pub fn detect_corners_fast(img: &[u8], width: u32, height: u32, threshold: u8) -> Vec<Point2<f64>> {
    let w = width as i32;
    let h = height as i32;
    assert_eq!(img.len(), (w * h) as usize, "image buffer size mismatch");
    let idx = |x: i32, y: i32| -> usize { (y as usize) * width as usize + (x as usize) };
    let t = threshold as i32;

    let mut candidates: Vec<(i32, i32, u16)> = Vec::new();
    for y in 3..h - 3 {
        for x in 3..w - 3 {
            let c = img[idx(x, y)] as i32;
            let mut hits_bright = [false; 16];
            let mut hits_dark = [false; 16];
            let mut score = 0u16;
            for (i, (dx, dy)) in CIRCLE.iter().enumerate() {
                let v = img[idx(x + dx, y + dy)] as i32;
                score = score.saturating_add((v - c).unsigned_abs() as u16);
                hits_bright[i] = v > c + t;
                hits_dark[i] = v < c - t;
            }
            if longest_run(hits_bright) >= 9 || longest_run(hits_dark) >= 9 {
                candidates.push((x, y, score));
            }
        }
    }

    // Non-maximum suppression: greedy, strongest first, 3×3 exclusion.
    candidates.sort_by_key(|c| std::cmp::Reverse(c.2));
    let mut kept = Vec::with_capacity(candidates.len());
    let mut taken = vec![false; img.len()];
    for (x, y, _) in candidates {
        let occupied = (x - 1..=x + 1).any(|xx| {
            (y - 1..=y + 1)
                .any(|yy| xx >= 0 && yy >= 0 && xx < w && yy < h && taken[idx(xx, yy)])
        });
        if !occupied {
            taken[idx(x, y)] = true;
            kept.push(Point2::new(x as f64, y as f64));
        }
    }
    kept
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic;

    #[test]
    fn finds_many_corners_on_textured_plane() {
        let rig = crate::camera::StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let (frame, _) = synthetic::render(&rig, &nalgebra::Isometry3::identity(), 1.5);
        let corners = detect_corners_fast(&frame.left, 640, 480, 20);
        assert!(corners.len() > 200, "only {} corners", corners.len());
        for c in corners.iter().take(20) {
            assert!(c.x >= 0.0 && c.x < 640.0 && c.y >= 0.0 && c.y < 480.0);
        }
    }

    #[test]
    fn finds_none_on_blank_and_flat_gradient() {
        let blank = vec![128u8; 640 * 480];
        assert!(detect_corners_fast(&blank, 640, 480, 20).is_empty());

        // A gentle linear gradient has no corner structure.
        let grad: Vec<u8> = (0..640 * 480).map(|i| (i % 640) as u8 / 2).collect();
        assert!(detect_corners_fast(&grad, 640, 480, 20).is_empty());
    }

    #[test]
    fn high_threshold_reduces_count() {
        let rig = crate::camera::StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let (frame, _) = synthetic::render(&rig, &nalgebra::Isometry3::identity(), 1.5);
        let low = detect_corners_fast(&frame.left, 640, 480, 10).len();
        let high = detect_corners_fast(&frame.left, 640, 480, 60).len();
        assert!(high < low, "threshold 60 gave {high} ≥ {low} (threshold 10)");
    }
}
