//! `neuromancer-tracker` library crate: shared modules for the tracker
//! binary and its integration tests (CLI, IMU sources, JSONL, axis mapping,
//! output sinks).

pub mod axis;
pub mod calib;
pub mod cli;
pub mod imu;
pub mod jsonl;
pub mod log;
pub mod output;
pub mod visual;
