# Changelog

All notable changes per release. Semver per spec §5.6
(`neuromancer-tracker` + `neuromancer-ahrs` share the workspace version).

## Unreleased — Phase 2 (6-DoF, in progress)

### Added (milestone M1, spec Appendix D)

- `neuromancer-vo` crate (pure Rust, nalgebra): pinhole `CameraModel`,
  rectified `StereoRig` (metric scale via baseline), and a **synthetic
  stereo renderer** (textured plane, ground-truth depth + trajectory) as the
  CI-equivalent for VO tests. 7 unit tests.
- `--input imu|visual` switch (default `imu`; `visual` = 6-DoF with the IMU
  off — wired to a clear "under construction" error until the VO pipeline
  lands; `imu+visual` fusion rejected as P2b).

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
