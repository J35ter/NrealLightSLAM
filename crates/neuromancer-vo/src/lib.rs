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
pub mod features;
pub mod frame;
pub mod klt;
pub mod motion;
pub mod pipeline;
pub mod rectify;
pub mod stereo;
pub mod synthetic;

// Re-export the nalgebra types that appear in this crate's public API
// (StereoRig::left_t_right, project/unproject, etc.) so downstream crates
// don't need to match nalgebra versions directly.
pub use nalgebra::{Isometry3, Matrix3, Point2, Point3, Quaternion, Translation3, UnitQuaternion, Vector3};
