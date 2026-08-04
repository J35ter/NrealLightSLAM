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
//!   cargo run --release -p neuromancer-tracker --example cam_probe --stats [seconds]
//!     — pipeline funnel counters per frame, replicating `estimate_motion`
//!       exactly: FAST corners → KLT tracked → stereo-matched in keyframe A →
//!       triangulated in A → stereo-matched in current frame B → triangulated
//!       in B → 3D-3D pairs into RANSAC → inliers. Plus pose deltas.
//!   cargo run --release -p neuromancer-tracker --example cam_probe --calib
//!     — dump the factory calibration the glasses store on-device
//!       (`CameraDescriptor` per camera: intrinsics, distortion kc,
//!       leftcam_q_rightcam, imu_to_camera) — the M8 calibration source.
//! Not part of the shipped tracker binary.

use std::time::{Duration, Instant};

use nalgebra::{Isometry3, Point2};

use neuromancer_vo::camera::{CameraModel, StereoRig};
use neuromancer_vo::features::detect_corners_fast;
use neuromancer_vo::klt::klt_track;
use neuromancer_vo::motion::ransac_motion;
use neuromancer_vo::pipeline::VoPipeline;
use neuromancer_vo::stereo::StereoMatcher;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let vo_mode = args.iter().any(|a| a == "--vo");
    let stats_mode = args.iter().any(|a| a == "--stats");
    let calib_mode = args.iter().any(|a| a == "--calib");
    if calib_mode {
        dump_calibration();
        return;
    }
    let seconds: f64 = args
        .iter()
        .skip(1)
        .find(|a| a.parse::<f64>().is_ok())
        .and_then(|a| a.parse().ok())
        .unwrap_or(15.0);
    if stats_mode {
        run_stats(seconds);
        return;
    }
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

/// Per-frame pipeline funnel, replicating `motion::estimate_motion` exactly
/// (M4/M5): FAST corners → KLT track → stereo-match + triangulate in both
/// frames → 3D-3D pairs → RANSAC inliers. Reports per-frame counts and the
/// pose delta (translation magnitude + rotation angle) so the tracking-point
/// variance in a real scene is visible.
fn run_stats(seconds: f64) {
    let rig = StereoRig::rectified(500.0, 500.0, 320.0, 240.0, 0.12, 640, 480);
    let matcher = StereoMatcher::new(5, 0.5, 10.0);
    let cam: &CameraModel = &rig.left;

    let mut camera = ar_drivers::nreal_light::NrealLightSlamCamera::new()
        .expect("cannot open Nreal Light SLAM camera");
    eprintln!("camera opened — pipeline stats for {seconds:.1} s ...");

    let deadline = Instant::now() + Duration::from_secs_f64(seconds);
    let start = Instant::now();

    // The keyframe (frame A): features detected once, tracked into each new
    // frame B (matches VoPipeline: every frame is a keyframe, motion is
    // frame-to-frame A→B).
    type Keyframe = (Vec<u8>, Vec<u8>, Vec<Point2<f64>>);
    let mut prev: Option<Keyframe> = None;

    let mut frames = 0usize;
    let mut posed_frames = 0usize;
    let mut trans_mags: Vec<f64> = Vec::new();
    let mut rot_angles: Vec<f64> = Vec::new();

    while Instant::now() < deadline {
        let f = match camera.get_frame(Duration::from_secs(2)) {
            Ok(f) => f,
            Err(e) => {
                eprintln!("get_frame error: {e}");
                continue;
            }
        };
        frames += 1;

        match &prev {
            None => {
                // First frame: keyframe only.
                let feats: Vec<Point2<f64>> = detect_corners_fast(
                    &f.left, 640, 480, 20,
                )
                .into_iter()
                .take(1500)
                .collect();
                println!(
                    "frame {frames:4}  keyframe: {} corners",
                    feats.len()
                );
                prev = Some((f.left, f.right, feats));
            }
            Some((left_a, right_a, feats_a)) => {
                // KLT A→B.
                let tracked = klt_track(left_a, &f.left, 640, 480, feats_a, 5, 12);
                let n_tracked = tracked.iter().filter(|t| t.ok).count();

                // Stereo-match + triangulate in A and B, exactly like
                // estimate_motion's loop.
                let mut src = Vec::new();
                let mut dst = Vec::new();
                let mut px_b = Vec::new();
                let mut n_matched_a = 0usize;
                let mut n_matched_b = 0usize;
                let mut n_tri_a = 0usize;
                let mut n_tri_b = 0usize;
                for (feat, t) in feats_a.iter().zip(tracked.iter()) {
                    if !t.ok {
                        continue;
                    }
                    let Some((da, _)) = matcher.match_feature(&rig, left_a, right_a, feat) else {
                        continue;
                    };
                    n_matched_a += 1;
                    let Some(pa) = matcher.triangulate(&rig, feat, da) else {
                        continue;
                    };
                    n_tri_a += 1;
                    let Some((db, _)) = matcher.match_feature(&rig, &f.left, &f.right, &t.point)
                    else {
                        continue;
                    };
                    n_matched_b += 1;
                    let Some(pb) = matcher.triangulate(&rig, &t.point, db) else {
                        continue;
                    };
                    n_tri_b += 1;
                    src.push(pa);
                    dst.push(pb);
                    px_b.push(t.point);
                }

                // RANSAC + GN refinement (same params as estimate_motion).
                let est = ransac_motion(&src, &dst, &px_b, cam, 300, 3.0);
                let (inliers, pose) = match &est {
                    Some(e) => (Some(e.inliers), e.pose),
                    None => (None, Isometry3::identity()),
                };
                let trans_mag = pose.translation.vector.norm();
                let rot_angle = pose.rotation.angle();
                if est.is_some() {
                    posed_frames += 1;
                    trans_mags.push(trans_mag);
                    rot_angles.push(rot_angle);
                }
                println!(
                    "frame {frames:4}  corners={:<4} tracked={:<4} matchA={:<4} triA={:<4} matchB={:<4} triB={:<4} pairs={:<4} ransac={}{}  t={:.4} m  r={:.3} rad",
                    feats_a.len(),
                    n_tracked,
                    n_matched_a,
                    n_tri_a,
                    n_matched_b,
                    n_tri_b,
                    src.len(),
                    inliers.map(|i| format!("{i}/")).unwrap_or_else(|| "none/".to_string()),
                    src.len(),
                    trans_mag,
                    rot_angle,
                );

                // New keyframe (frame-to-frame).
                let feats_b: Vec<Point2<f64>> = detect_corners_fast(
                    &f.left, 640, 480, 20,
                )
                .into_iter()
                .take(1500)
                .collect();
                prev = Some((f.left, f.right, feats_b));
            }
        }
    }
    let elapsed = start.elapsed().as_secs_f64();

    println!("\n=== pipeline stats ({} s) ===", seconds);
    println!("frames: {frames} ({:.1} fps), pose frames: {posed_frames}", frames as f64 / elapsed.max(1e-9));
    if !trans_mags.is_empty() {
        print_stats("pose |t| (m)", &trans_mags, |x| x);
        print_stats("pose rot (rad)", &rot_angles, |x| x);
    }
}

/// Dump the factory calibration stored on the glasses (M8 calibration
/// source). `NrealLight::cameras()` reads the on-device `SLAM_camera` config
/// JSON: focal length (fc), principal point (cc), distortion (kc, 5
/// coefficients), leftcam_q_rightcam stereo rotation, and imu_to_camera.
/// Note ar-drivers 0.4.3 only surfaces `kc[0]` into the descriptor and sets
/// `stereo_rotation` only on the right camera — the raw JSON is richer, so
/// verify against `get_config_float_array` paths if values look off.
fn dump_calibration() {
    use ar_drivers::ARGlasses;

    let mut glasses = match ar_drivers::nreal_light::NrealLight::new() {
        Ok(g) => g,
        Err(e) => {
            eprintln!("cannot open Nreal Light: {e}");
            std::process::exit(1);
        }
    };
    let cams = match glasses.cameras() {
        Ok(c) => c,
        Err(e) => {
            eprintln!("cannot read camera descriptors: {e}");
            std::process::exit(1);
        }
    };
    println!("=== on-device factory calibration (CameraDescriptor) ===");
    for (i, c) in cams.iter().enumerate() {
        println!("--- camera {i}: {} ---", c.name);
        println!("  resolution: {}x{}", c.resolution.x, c.resolution.y);
        println!("  intrinsic_matrix (fx 0 cx; 0 fy cy; 0 0 1):");
        println!("    [{:.4}, 0, {:.4}]", c.intrinsic_matrix[(0, 0)], c.intrinsic_matrix[(0, 2)]);
        println!("    [0, {:.4}, {:.4}]", c.intrinsic_matrix[(1, 1)], c.intrinsic_matrix[(1, 2)]);
        println!("    [0, 0, 1]");
        println!(
            "  distortion kc: [{:.6}, {:.6}, {:.6}, {:.6}, {:.6}]",
            c.distortion[0], c.distortion[1], c.distortion[2], c.distortion[3], c.distortion[4]
        );
        let q = c.stereo_rotation;
        println!(
            "  stereo_rotation quat (w,i,j,k): [{:.6}, {:.6}, {:.6}, {:.6}]",
            q.w, q.i, q.j, q.k
        );
        let t = c.imu_to_camera.translation.vector;
        let qi = c.imu_to_camera.rotation;
        println!(
            "  imu_to_camera t: [{:.6}, {:.6}, {:.6}] m",
            t.x, t.y, t.z
        );
        println!(
            "  imu_to_camera q: [{:.6}, {:.6}, {:.6}, {:.6}]",
            qi.w, qi.i, qi.j, qi.k
        );
    }
}
