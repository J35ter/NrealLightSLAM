# Changelog

All notable changes per release. Semver per spec §5.6
(`neuromancer-tracker` + `neuromancer-ahrs` share the workspace version).

## Unreleased — Phase 2 (6-DoF, in progress)

### Added (milestones M1–M6, spec Appendix D)

- `neuromancer-vo` crate (pure Rust, nalgebra): pinhole `CameraModel`,
  rectified `StereoRig` (metric depth = `fx·baseline/disparity`), synthetic
  stereo renderer (aperiodic value-noise texture, ground-truth depth +
  trajectory).
- `--input imu|visual` switch (default `imu`): `visual` = 6-DoF with the IMU
  fully off — the full stereo VO pipeline runs end-to-end (M5).
- FAST-9 corner detection + single-level Lucas–Kanade optical flow (M2;
  a coarse-to-fine pyramid was tried and removed — its blurred coarse level
  biased estimates).
- Epipolar stereo matching (SSD, depth-bounded disparity, subpixel) +
  metric triangulation (M3; median depth error < 2% on synthetic).
- Inter-frame motion estimation (M4): RANSAC over 3-point Umeyama alignment,
  validated by reprojection, refined by Gauss–Newton against reprojection
  error (refinement over PnP: DLT-PnP degenerates on planar scenes and stereo
  depth exists in both frames). < 5 mm / < 5 mrad in the well-constrained
  regime; bas-relief ambiguity for lateral motion at small rotation noted for
  P2b IMU fusion.
- `VoPipeline`: stateful frame-to-frame VO driver (1500-feature cap).
- 6-DoF outputs (M5): `Frame` gains position (meters); UDP `TX/TY/TZ` real in
  **centimeters** (Opentrack's translation convention); HUD appends `X/Y/Z`
  when non-zero (P1 output unchanged); pose log gains `x/y/z`.
- Visual sources: `SlamCameraSource` (hardware, ar-drivers) and
  `ReplayVisualSource` (`--replay-visual DIR`, raw `left_XXXX.raw` /
  `right_XXXX.raw`); `--record-visual DIR` captures them (M6, round-trip
  verified).
- 79 tests total; clippy clean.

### M7 hardware spike (2026-08-04, glasses on .240) — findings

- `--input visual` opens/streams on hardware; record→replay round-trip
  verified (357 frames → `/tmp/spike_seq`, clean exits).
- **Camera rate is 30 fps, steady** (probe-verified; the earlier "~6 fps
  device" reading was wrong). See D.5: `ar-drivers` `get_frame` is
  alignment-dependent — reads that land mid-frame take ~74 ms (permanent
  ~13 fps state) vs ~33 ms when aligned, and VO pose cost (~26 ms/frame)
  is on top. End-to-end is ~10–17 fps with a pose every frame. New
  `cam_probe` example (`crates/neuromancer-tracker/examples/cam_probe.rs`)
  times every read; the tracker now also logs a **sustained** rate at exit
  (first-30-frame snapshot was skewed by the 395 ms warm-up frame).
  `ar-drivers` device timestamps are unusable (all deltas 0).
- **IMU+camera coexistence confirmed** in both orders (spec D.1 risk did
  not materialize).
- Real IMU rate measured at **1000 Hz** (not the ~200 Hz assumed when OQ3
  closed).
- **Fixed: Ctrl-C race** — SIGINT landing inside a blocking USB IMU read
  made hidapi report "unplugged?" and the tracker wrongly exit 3. Root
  cause: `ctrlc` sets the flag from a separate waiter thread, so it could
  lose the scheduling race against the interrupted read's error return.
  Replaced `ctrlc` with a direct `sigaction` handler that increments the
  flag in signal context (async-signal-safe; `_exit` on second press;
  no SA_RESTART so the blocking read returns EINTR) — the flag is always
  set before the read error is observed. Regression test
  `cli_sigint_mid_run_exits_0` (80 tests total).
- Intrinsics still not wired (known): km-scale pose garbage on real
  scenes — M8 (CameraDescriptor + fisheye rectification + `imu_to_camera`)
  next.
- **M8: real camera calibration wired.** The tracker now builds the
  rectified rig from the glasses' on-device `CameraDescriptor` (intrinsics
  fx≈234, cx≈322/319, cy≈246/214; distortion kc; `imu_to_camera`), with
  per-frame undistort+rectify remaps (`neuromancer-vo::rectify`, Bouguet
  rotations + bilinear sampling) and head-frame pose alignment
  (`world_T_head = world_T_cam · imu_to_camera_left`). Stereo baseline
  0.103 m (from `imu_to_camera`, matches `leftcam_q_rightcam`).
  Replay uses this unit's captured constants. Verified: epipolar rows
  aligned (stereo-match vertical offset median 0.00 px), metric depth
  sanity 100% within [0.3, 20] m on recorded frames, 85 tests green,
  clippy clean. Remaining km-scale pose on some real frames is M9
  (RANSAC minimum-inlier gate, feature/keyframe management).
- **M9 first tranche: real-scene pose stability.** (1) Enclosed-space
  constraint: `StereoMatcher::triangulate` rejects points outside
  `[0.5, 10] m` — max measured depth now 10.0 m exactly (was 16.1 m).
  (2) RANSAC minimum-inlier gate (`(n/40).max(20)`): degenerate frames
  return None instead of an explosive pose. (3) Keyframe always advances
  in `VoPipeline` — previously a failed estimate left `prev` stuck on the
  dark auto-exposure first frame and the tracker never recovered. Live
  result: poses sane (< 10 m) 9% → 100% (20/20, 23/23 in two runs),
  |pos| median 4.1e14 → ~0.24 m, inter-pose delta bounded (max 0.2–0.5 m).
- **M9 second tranche: RANSAC drift tuning.** RANSAC budget 300 iters /
  3.0 px threshold → **2000 iters / 1.5 px** (motion.rs `estimate_motion`
  + `cam_probe --stats`). On a live still headset (20 s): path 4.8 m →
  1.46 m (−70%), endpoint drift 1.3 m → 0.21 m (−83%), systematic +z bias
  gone (mean −0.0012 m/step, t = −0.2 vs +0.027, t = 2.0), yaw no longer
  drifts (~15° systematic → ±12.4° wander). Trade-off: fewer posed frames
  (26 vs 45 per 20 s) — safe because rejected frames hit the D.8
  min-inlier gate and the pipeline keeps the prior keyframe pose.
- **`ar-drivers` vendored + `get_frame` read fix (M8 action item):**
  `vendor/ar-drivers` replaces the crates.io dep. `get_frame` now
  accumulates reads into a persistent buffer and extracts exactly one
  615908-byte frame, carrying the over-read tail (start of the next frame)
  instead of discarding it and re-reading. The old discard-and-retry made
  reads that landed mid-frame cost ~74 ms (a persistent ~13 fps state);
  probe-verified 10/10 runs now at 29.8 fps with no 13 fps state. Upstream
  `examples/` + dev-deps (clap, opencv) removed; clippy silenced at the
  crate root (third-party).

### Pending

- M9 (rest): residual still-headset jitter + motion-heavy validation of
  the 1.5 px threshold — feature quality filters, keyframe thinning,
  tighter min-inlier gate; IMU fusion in P2b.

## v0.1.0 — 2026-08-04 — Phase 1 (3-DoF) release

First pinned release: stable IMU-only 3-DoF tracking, verified on real
hardware (V, 2026-08-03/04).

### Added

- `neuromancer-ahrs` crate (dependency-free): Mahony complementary filter,
  quaternion math, YXZ Tait-Bryan yaw/pitch/roll (RUB frame), 14 unit tests.
- `neuromancer-tracker` crate: USB IMU (ar-drivers/rusb) → filter → outputs.
- Opentrack UDP output, `--protocol classic|extended` (OQ1 — both variants,
  default classic) and `--units deg|rad` (default **degrees** — stock
  Opentrack reads degrees; the original "radians" spec text was corrected).
- 2 Hz HUD readout (`--hud`, `--hud-rate`), fixed-width columns, `\r` redraw.
- IMU/pose JSONL logs (`--log-imu`, `--log-pose`), append-only, best-effort.
- Hand-rolled zero-dep CLI (`--host/--port/--no-udp/--kp/--ki/--invert-*`,
  `--sensitivity`, `--udp-rate`, `--version`, `--help`).
- `--replay <PATH>` dev/testing input (JSONL IMU) — hardware-free testing.
- `--log-level error|warn|info|debug` (default `error`): stderr output is
  gated; UDP send warnings only with `--log-level warning` (throttled once/5 s,
  unthrottled at `debug`).
- `--gyro-calib <SEC>` (default 2.0, 0=off): startup stationary gyro-bias
  calibration **plus in-run refresh on stillness** — fixes yaw drift from
  residual turn-on bias (measured ~15°/60 s ≈ 0.25°/s on real glasses →
  < 2.5° in tests) and tracks thermal bias drift mid-session.
- Exit codes 0/1/2/3 (clean SIGINT / no device / CLI error / USB unplug);
  first Ctrl-C graceful, second immediate.
- `start-tracker.sh` launcher (HUD + UDP + protocol switches, env-overridable)
  and `tools/udp_listen.py` diagnostic (parses packets like Opentrack).
- 56 tests total (ahrs 14, tracker unit 30, integration 12); clippy clean.

### Fixed / notable

- Mahony kinematics corrected to the canonical `q̇ = ½ q ⊗ ω` with a
  discriminating regression test (a prior review had mis-flipped the operand
  order).
- UDP warning spam fixed (Linux ICMP Ok/Err flapping) via 5 s cooldown.
- `ar-drivers` 0.4.3 `nreal` feature-map bug worked around (`+ "rusb"`).
- `--log-pose`/`--log-imu` accept hostnames/IPv6; NaN/inf JSONL rejected.

### Known limits (Phase 1)

- Yaw has no absolute reference (no magnetometer); drift is bias-driven and
  now calibrated, but still present over very long sessions.
- UDP decimator emits 50 Hz from a ~1650 Hz IMU stream (every 4th sample;
  "60 Hz max, no minimum").
- T2/T3/T5/T6 need glasses: T3 re-test pending after the calibration fix.
