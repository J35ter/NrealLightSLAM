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

### Pending

- M7 hardware spike (glasses on .240): SlamCamera @ ~30 fps, intrinsics
  sanity + fisheye rectification, IMU+camera coexistence, head-frame
  alignment (`imu_to_camera`), real-scene VO tuning.

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
