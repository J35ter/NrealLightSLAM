//! Stateful VO driver: consumes a stereo frame sequence and produces the
//! accumulated 6-DoF camera pose (M5).
//!
//! Frame-to-frame VO: each frame is a keyframe (features detected + stereo
//! depth), tracked to the next via KLT, motion estimated, pose accumulated
//! `world_T_cam_new = world_T_cam_old · A_T_B`. The first frame defines the
//! world origin. Keyframe selection / drift management are M7 tuning items.

use nalgebra::{Isometry3, Point2};

use super::camera::StereoRig;
use super::features::detect_corners_fast;
use super::klt::klt_track;
use super::motion::estimate_motion;
use super::stereo::StereoMatcher;

/// Feature detection threshold (gray levels).
const FAST_THRESHOLD: u8 = 20;
/// Cap on features per frame — keeps RANSAC/pipeline runtime bounded for
/// real-time use (11k uncapped features made tests take ~50 s).
const MAX_FEATURES: usize = 1500;

struct PrevFrame {
    left: Vec<u8>,
    right: Vec<u8>,
    features: Vec<Point2<f64>>,
}

/// Frame-to-frame stereo VO.
pub struct VoPipeline {
    rig: StereoRig,
    matcher: StereoMatcher,
    prev: Option<PrevFrame>,
    pose: Isometry3<f64>,
    frames: u64,
}

impl VoPipeline {
    pub fn new(rig: StereoRig, matcher: StereoMatcher) -> Self {
        VoPipeline {
            rig,
            matcher,
            prev: None,
            pose: Isometry3::identity(),
            frames: 0,
        }
    }

    /// Process one stereo frame (grayscale left/right). Returns the updated
    /// world→camera pose on success; `None` when motion could not be
    /// estimated for this frame (the previous keyframe is kept).
    pub fn process(&mut self, left: &[u8], right: &[u8]) -> Option<Isometry3<f64>> {
        match &self.prev {
            None => {
                self.prev = Some(PrevFrame {
                    left: left.to_vec(),
                    right: right.to_vec(),
                    features: detect_corners_fast(left, self.rig.left.width, self.rig.left.height, FAST_THRESHOLD)
                        .into_iter()
                        .take(MAX_FEATURES)
                        .collect(),
                });
                self.frames += 1;
                Some(self.pose)
            }
            Some(prev) => {
                let tracked = klt_track(
                    &prev.left,
                    left,
                    self.rig.left.width,
                    self.rig.left.height,
                    &prev.features,
                    5,
                    12,
                );
                let est = estimate_motion(
                    &self.rig,
                    &self.matcher,
                    &prev.left,
                    &prev.right,
                    left,
                    right,
                    &prev.features,
                    &tracked,
                )?;
                self.pose *= est.pose;
                self.prev = Some(PrevFrame {
                    left: left.to_vec(),
                    right: right.to_vec(),
                    features: detect_corners_fast(left, self.rig.left.width, self.rig.left.height, FAST_THRESHOLD)
                        .into_iter()
                        .take(MAX_FEATURES)
                        .collect(),
                });
                self.frames += 1;
                Some(self.pose)
            }
        }
    }

    pub fn pose(&self) -> Isometry3<f64> {
        self.pose
    }

    pub fn frames(&self) -> u64 {
        self.frames
    }

    pub fn reset(&mut self) {
        self.prev = None;
        self.pose = Isometry3::identity();
        self.frames = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synthetic::{forward_trajectory, render};
    use nalgebra::Translation3;

    /// Run the pipeline over a forward trajectory; the accumulated pose must
    /// track the ground truth within tolerance (the motion-estimation
    /// accuracy carries through the incremental integration).
    #[test]
    fn pipeline_recovers_forward_trajectory() {
        let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
        let matcher = StereoMatcher::new(5, 0.5, 10.0);
        let mut vo = VoPipeline::new(rig.clone(), matcher);

        let traj = forward_trajectory(6, 0.04, 0.5);
        let mut last_pose = Isometry3::identity();
        for (i, pose) in traj.iter().enumerate() {
            let (frame, _) = render(&rig, pose, 1.5);
            if let Some(p) = vo.process(&frame.left, &frame.right) {
                last_pose = p;
                // Cumulative translation should track the ground truth.
                let err = (p.translation.vector - pose.translation.vector).norm();
                let budget = 0.02 + 0.008 * i as f64;
                assert!(
                    err < budget,
                    "frame {i}: translation error {err} (budget {budget})"
                );
            }
        }
        assert!(vo.frames() >= 5);
        assert!(
            (last_pose.translation.vector.z - 0.2).abs() < 0.04,
            "final z {} (expected ~0.2)",
            last_pose.translation.vector.z
        );
        let _ = Translation3::new(0.0, 0.0, 0.0);
    }
}
