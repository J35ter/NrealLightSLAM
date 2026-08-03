//! Dev tool (M7 spike follow-up): characterize the Nreal Light SLAM camera's
//! actual frame delivery rate.
//!
//! The tracker's first-30-frame fps snapshot showed 6–28 fps across runs, so
//! this probe opens the OV580 camera directly via `ar-drivers` (no VO/HUD/
//! recording in the way) and times every `get_frame` call, recording:
//!   - wall-clock gaps between successive frame reads,
//!   - the device's own per-frame timestamp deltas,
//!   - how long each `get_frame` read takes.
//!
//! Usage:
//!   cargo run --release -p neuromancer-tracker --example cam_probe [seconds]
//!     — bare camera read timing (default 15 s)
//!   cargo run --release -p neuromancer-tracker --example cam_probe --vo [seconds]
//!     — camera read + full tracker VO pipeline (what the binary actually runs),
//!       reporting read time, vo.process time and pose yield separately.
//! Not part of the shipped tracker binary.

use std::time::{Duration, Instant};

use neuromancer_vo::camera::StereoRig;
use neuromancer_vo::pipeline::VoPipeline;
use neuromancer_vo::stereo::StereoMatcher;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let vo_mode = args.iter().any(|a| a == "--vo");
    let seconds: f64 = args
        .iter()
        .skip(1)
        .find(|a| a.parse::<f64>().is_ok())
        .and_then(|a| a.parse().ok())
        .unwrap_or(15.0);
    let deadline = Instant::now() + Duration::from_secs_f64(seconds);

    let mut cam = ar_drivers::nreal_light::NrealLightSlamCamera::new()
        .expect("cannot open Nreal Light SLAM camera");
    eprintln!(
        "camera opened — probing for {seconds:.1} s (vo={vo_mode}) ..."
    );

    // The exact rig/matcher the tracker binary uses (main.rs).
    let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
    let mut vo = VoPipeline::new(rig, StereoMatcher::new(5, 0.5, 10.0));

    let mut wall_gaps: Vec<f64> = Vec::new(); // seconds between read returns
    let mut read_ms: Vec<f64> = Vec::new(); // duration of each get_frame call
    let mut vo_ms: Vec<f64> = Vec::new(); // duration of each vo.process call
    let mut prev_wall: Option<Instant> = None;
    let mut poses = 0usize;
    let start = Instant::now();
    let mut frames = 0usize;

    while Instant::now() < deadline {
        let t0 = Instant::now();
        match cam.get_frame(Duration::from_secs(2)) {
            Ok(f) => {
                frames += 1;
                let t1 = Instant::now();
                if let Some(p) = prev_wall {
                    wall_gaps.push(t1.duration_since(p).as_secs_f64());
                }
                prev_wall = Some(t1);
                read_ms.push(t1.duration_since(t0).as_secs_f64() * 1000.0);
                let (v, pose) = if vo_mode {
                    let v0 = Instant::now();
                    let pose = vo.process(&f.left, &f.right);
                    (v0.elapsed().as_secs_f64() * 1000.0, pose.is_some())
                } else {
                    (0.0, false)
                };
                vo_ms.push(v);
                if pose {
                    poses += 1;
                }
                eprintln!(
                    "frame {frames:4}  wall_gap={:7.1} ms  read={:7.1} ms  vo={:6.1} ms  pose={}",
                    wall_gaps.last().map(|g| g * 1000.0).unwrap_or(0.0),
                    read_ms.last().copied().unwrap_or(0.0),
                    v,
                    pose,
                );
            }
            Err(e) => eprintln!("get_frame error: {e}"),
        }
    }
    let elapsed = start.elapsed().as_secs_f64();

    println!("\n=== probe result ({} s, vo={vo_mode}) ===", seconds);
    println!("frames: {frames},  mean wall rate: {:.2} fps", frames as f64 / elapsed.max(1e-9));
    if vo_mode {
        println!(
            "pose yield: {poses}/{} ({:.0}%)",
            frames,
            poses as f64 / frames.max(1) as f64 * 100.0
        );
    }
    if !wall_gaps.is_empty() {
        print_stats("wall gap (ms)", &wall_gaps, |g| g * 1000.0);
        print_hist("wall gap", &wall_gaps, 0.0, 0.20);
    }
    print_stats("read duration (ms)", &read_ms, |r| r);
    if vo_mode {
        print_stats("vo.process (ms)", &vo_ms, |v| v);
    }
}

fn print_stats<F: Fn(f64) -> f64>(label: &str, data: &[f64], map: F) {
    let mut v: Vec<f64> = data.iter().copied().map(&map).collect();
    v.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let p = |q: f64| v[(v.len() as f64 * q).floor() as usize];
    let sum: f64 = v.iter().sum();
    println!(
        "{label}: n={}  min={:.1}  p50={:.1}  p90={:.1}  p95={:.1}  p99={:.1}  max={:.1}  mean={:.1}",
        v.len(),
        v[0],
        p(0.50),
        p(0.90),
        p(0.95),
        p(0.99),
        v[v.len() - 1],
        sum / v.len() as f64,
    );
}

/// Histogram of gaps in [lo, hi) seconds.
fn print_hist(label: &str, data: &[f64], lo: f64, hi: f64) {
    const BINS: usize = 20;
    let mut counts = [0usize; BINS];
    for &g in data {
        let t = (g - lo) / (hi - lo) * BINS as f64;
        let b = if t < 0.0 {
            0
        } else if t >= BINS as f64 {
            BINS - 1
        } else {
            t as usize
        };
        counts[b] += 1;
    }
    println!("{label} histogram [{lo:.2}..{hi:.2}) s, {} bins of {:.0} ms:", BINS, (hi - lo) * 1000.0 / BINS as f64);
    let max = counts.iter().copied().max().unwrap_or(1).max(1);
    for (i, &c) in counts.iter().enumerate() {
        let bar = "#".repeat(c * 40 / max);
        println!(
            "  {:>7.1} ms | {:5} {bar}",
            (lo + (i as f64 + 0.5) * (hi - lo) / BINS as f64) * 1000.0,
            c
        );
    }
}
