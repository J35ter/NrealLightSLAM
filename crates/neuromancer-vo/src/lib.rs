//! `neuromancer-vo` — pure-Rust stereo visual odometry for the Nreal Light
//! head tracker (Phase 2, 6-DoF). Approved approach (spec Appendix D):
//! rectified stereo → FAST corners → KLT optical flow → epipolar stereo
//! matching → triangulation (metric depth) → PnP + RANSAC → incremental
//! 6-DoF pose. Milestone M1 (this module set): camera model, rectified rig,
//! and a synthetic stereo renderer with ground-truth pose and depth.
//!
//! # Conventions
//!
//! - Camera frames: OpenCV-like — +Z forward, +X right, +Y down; pixels
//!   `(u, v)` with `u` right, `v` down, origin top-left.
//! - Poses are `nalgebra::Isometry3<f64>` mapping **world → camera**
//!   (`world_T_cam` names: "pose of the camera in the world frame").
//! - The synthetic world is y-down too (consistent with the camera frame),
//!   with the textured plane at `z = plane_z > 0` in front of the camera.

pub mod camera;
pub mod frame;
pub mod synthetic;
