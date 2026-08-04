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
use neuromancer_vo::rectify::Rectifier;
use neuromancer_vo::stereo::StereoMatcher;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let vo_mode = args.iter().any(|a| a == "--vo");
    let stats_mode = args.iter().any(|a| a == "--stats");
    let calib_mode = args.iter().any(|a| a == "--calib");
    let rect_check: Option<String> = args
        .iter()
        .position(|a| a == "--rect-check")
        .and_then(|i| args.get(i + 1).cloned());
    let depth_check: Option<String> = args
        .iter()
        .position(|a| a == "--depth-check")
        .and_then(|i| args.get(i + 1).cloned());
    if let Some(dir) = rect_check {
        run_rect_check(&dir);
        return;
    }
    if let Some(dir) = depth_check {
        run_depth_check(&dir);
        return;
    }
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
    // M8: use the real calibration (live device descriptors) and rectify
    // each frame, matching what the tracker binary does.
    let (rectifier, _) = neuromancer_tracker::cam_calib::build_rectifier(true);
    let rig = rectifier.rig.clone();
    let matcher = StereoMatcher::new(5, 0.5, 10.0);
    let cam: &CameraModel = &rig.left;

    let mut camera = ar_drivers::nreal_light::NrealLightSlamCamera::new()
        .expect("cannot open Nreal Light SLAM camera");
    eprintln!("camera opened — pipeline stats for {seconds:.1} s (rectified, baseline {:.3} m) ...",
        rig.left_t_right.translation.vector.x);

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
        // M8: rectify the raw pair before feature detection / matching.
        let (r_left, r_right) = rectifier.apply(&f.left, &f.right);

        match &prev {
            None => {
                // First frame: keyframe only.
                let feats: Vec<Point2<f64>> = detect_corners_fast(
                    &r_left, 640, 480, 20,
                )
                .into_iter()
                .take(1500)
                .collect();
                println!(
                    "frame {frames:4}  keyframe: {} corners",
                    feats.len()
                );
                prev = Some((r_left, r_right, feats));
            }
            Some((left_a, right_a, feats_a)) => {
                // KLT A→B (rectified images).
                let tracked = klt_track(left_a, &r_left, 640, 480, feats_a, 5, 12);
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
                    let Some((db, _)) = matcher.match_feature(&rig, &r_left, &r_right, &t.point)
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
                    &r_left, 640, 480, 20,
                )
                .into_iter()
                .take(1500)
                .collect();
                prev = Some((r_left, r_right, feats_b));
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

    let glasses = match ar_drivers::nreal_light::NrealLight::new() {
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

/// Rectification sanity check: load one raw recorded frame pair, rectify it
/// with the unit's calibration, detect corners in the rectified left image,
/// and measure the vertical offset of their best same-row stereo matches in
/// the rectified right image. After correct rectification the residual
/// vertical offset should be ~0 px (epipolar lines horizontal); the raw
/// (unrectified) pair shows the camera's vertical misalignment (~30 px).
fn run_rect_check(dir: &str) {
    let (left_c, right_c, left_t_right) = neuromancer_tracker::cam_calib::fallback_calib_for_probe();
    let rect = Rectifier::new(&left_c, &right_c, left_t_right);
    eprintln!("rectifier built: baseline {:.3} m, coverage {:.1}%", rect.rig.left_t_right.translation.vector.x, rect.coverage() * 100.0);

    let read = |side: &str| -> Vec<u8> {
        let p = format!("{dir}/{side}_0000.raw");
        std::fs::read(&p).unwrap_or_else(|e| panic!("cannot read {p}: {e}"))
    };
    let (raw_l, raw_r) = (read("left"), read("right"));
    let (rl, rr) = rect.apply(&raw_l, &raw_r);

    let matcher = StereoMatcher::new(5, 0.5, 10.0);
    let feats: Vec<Point2<f64>> = detect_corners_fast(&rl, 640, 480, 20)
        .into_iter()
        .filter(|c| c.x > 60.0 && c.x < 580.0 && c.y > 60.0 && c.y < 420.0)
        .collect();
    eprintln!("detected {} corners in rectified left", feats.len());

    // For each corner, find the best stereo match; the winning disparity's
    // row in the right image should equal the left row (epipolar alignment).
    let mut v_offsets: Vec<f64> = Vec::new();
    for f in &feats {
        if let Some((_, _)) = matcher.match_feature(&rect.rig, &rl, &rr, f) {
            // match_feature assumes same-row matching (it only searches the
            // left's row in the right image), so this only verifies a match
            // EXISTS. For a true epipolar check we measure the vertical
            // offset directly with a small search band instead.
            let (u, v) = (f.x.round() as i32, f.y.round() as i32);
            let mut best = f64::INFINITY;
            let mut best_dv = 0.0;
            for dv in -3i32..=3 {
                let mut cost = f64::INFINITY;
                for d in 5..=60 {
                    let mut s = 0.0;
                    let mut ok = true;
                    for j in -2..=2 {
                        for i in -2..=2 {
                            let lx = u + i;
                            let ly = v + j;
                            let rx = u - d + i;
                            let ry = v + dv + j;
                            if !(0..640).contains(&lx)
                                || !(0..480).contains(&ly)
                                || !(0..640).contains(&rx)
                                || !(0..480).contains(&ry)
                            {
                                ok = false;
                                break;
                            }
                            let dl = rl[(ly * 640 + lx) as usize] as f64;
                            let dr = rr[(ry * 640 + rx) as usize] as f64;
                            let e = dl - dr;
                            s += e * e;
                        }
                        if !ok {
                            break;
                        }
                    }
                    if ok && s < cost {
                        cost = s;
                    }
                }
                if cost < best {
                    best = cost;
                    best_dv = dv as f64;
                }
            }
            if best.is_finite() {
                v_offsets.push(best_dv);
            }
        }
    }
    if v_offsets.is_empty() {
        eprintln!("no matches found — check rectification");
        return;
    }
    v_offsets.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let med = v_offsets[v_offsets.len() / 2];
    let mean: f64 = v_offsets.iter().sum::<f64>() / v_offsets.len() as f64;
    let p90 = v_offsets[(v_offsets.len() as f64 * 0.9) as usize];
    eprintln!("stereo-match vertical offset over {} corners: mean {mean:.2} px, median {med:.2} px, p90 {p90:.2} px", v_offsets.len());
    eprintln!("(rectified epipolar lines should give median ≈ 0 px)");
}

/// Depth sanity check (M8): rectify a recorded raw pair, stereo-match +
/// triangulate with the real intrinsics, and report the depth distribution.
/// With the factory calibration the depths should be metric-plausible
/// (roughly 0.5–10 m for an indoor scene), where the old hardcoded rig
/// (fx=500, baseline 0.12) systematically mis-scaled everything.
fn run_depth_check(dir: &str) {
    let (left_c, right_c, left_t_right) = neuromancer_tracker::cam_calib::fallback_calib_for_probe();
    let rect = Rectifier::new(&left_c, &right_c, left_t_right);
    let read = |side: &str| -> Vec<u8> {
        let p = format!("{dir}/{side}_0000.raw");
        std::fs::read(&p).unwrap_or_else(|e| panic!("cannot read {p}: {e}"))
    };
    let (raw_l, raw_r) = (read("left"), read("right"));
    let (rl, rr) = rect.apply(&raw_l, &raw_r);
    let rig = &rect.rig;
    let matcher = StereoMatcher::new(5, 0.5, 10.0);
    let feats: Vec<Point2<f64>> = detect_corners_fast(&rl, 640, 480, 20)
        .into_iter()
        .filter(|c| c.x > 60.0 && c.x < 580.0 && c.y > 60.0 && c.y < 420.0)
        .collect();
    eprintln!("{} corners", feats.len());
    let mut depths: Vec<f64> = Vec::new();
    for f in &feats {
        let Some((d, _)) = matcher.match_feature(rig, &rl, &rr, f) else { continue };
        let Some(p) = matcher.triangulate(rig, f, d) else { continue };
        depths.push(p.z);
    }
    depths.sort_by(|a, b| a.partial_cmp(b).unwrap());
    if depths.is_empty() {
        eprintln!("no depths");
        return;
    }
    let p = |q: f64| depths[(depths.len() as f64 * q) as usize];
    let n = depths.len() as f64;
    let mean: f64 = depths.iter().sum::<f64>() / n;
    let med = p(0.5);
    eprintln!("depths n={}  p10={:.2}  med={:.2}  p90={:.2}  max={:.1} m   mean={:.2} m", depths.len(), p(0.1), med, p(0.9), depths[depths.len()-1], mean);
    eprintln!("fraction within [0.3, 20] m: {:.0}%", depths.iter().filter(|z| **z >= 0.3 && **z <= 20.0).count() as f64 / n * 100.0);
}
