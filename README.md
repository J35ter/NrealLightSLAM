# neuromancer-tracker — Nreal Light head tracker

Phase 1 (3-DoF) implementation of [`DEVELOPMENT_SPEC.md`](DEVELOPMENT_SPEC.md):
reads the Nreal Light IMU (accelerometer + gyroscope) over USB, fuses it with a
Mahony complementary filter, and streams yaw/pitch/roll to **Opentrack** over
UDP, with a 2 Hz on-screen HUD for visual verification.

```
Nreal Light ──USB──▶ [ImuSource] ──▶ [Mahony AHRS] ──▶ [axis mapping] ──▶ [UDP → Opentrack]
                       │                                                     └─▶ [HUD 2 Hz]
                       └─▶ [--log-imu]  (raw samples, JSONL)     [--log-pose] (filtered pose)
```

## Workspace

| Crate | Role |
|-------|------|
| `crates/neuromancer-ahrs` | Dependency-free AHRS: Mahony filter, quaternion math, YXZ Tait-Bryan YPR (RUB frame) — reusable standalone |
| `crates/neuromancer-tracker` | The tracker binary (lib + bin) |

## Build & test (canonical, spec §5.2)

```bash
cargo build --release          # Linux dev build
cargo test --workspace         # ahrs (13) + tracker unit (18) + integration (10)
```

Windows (`mini` @ 192.168.0.231, P4): native build preferred — `cargo build --release --target x86_64-pc-windows-msvc` (Rust 1.97.1 + VS Build Tools already verified there).

## Quick start (hardware)

1. Plug the Nreal Light into USB.
2. Start Opentrack with the **UDP over network** tracker listening on
   `127.0.0.1:4242`.
3. Run:

```bash
neuromancer-tracker --hud                 # UDP out (default) + 2 Hz HUD
```

Rotate the headset — HUD shows live degrees, Opentrack gets the pose.

## Testing without hardware: `--replay`

The tracker's production input is USB-exclusive (spec §2.3), but for
development/verification you can feed a JSONL IMU log through the same
`ImuSource` abstraction:

```bash
# record once (needs glasses):  append raw IMU to a file
neuromancer-tracker --log-imu /tmp/imu.jsonl --no-udp

# replay anywhere (dev box, CI): paced like a live stream
neuromancer-tracker --replay /tmp/imu.jsonl --hud
```

Replay files use the spec §4.4 IMU log format: `{"t": <monotonic s>, "ax": .., "ay": .., "az": .., "gx": .., "gy": .., "gz": ..}`.

## CLI reference

```
neuromancer-tracker [OPTIONS]

  --host <IP>          Opentrack host (default 127.0.0.1)
  --port <PORT>        Opentrack UDP port (default 4242)
  --no-udp             Disable UDP output (HUD-only mode)
  --protocol <P>       classic|extended  (default classic)      — OQ1: user switch
  --udp-rate <HZ>      UDP output rate   (default 60)            — G2/§4.2 resolution
  --units <U>          deg|rad for UDP + pose log (default deg)  — wire-format resolution
  --hud                Enable 2 Hz on-screen orientation readout
  --hud-rate <HZ>      HUD update rate (default 2)
  --kp <FLOAT>         Mahony proportional gain (default 1.0)
  --ki <FLOAT>         Mahony integral gain (default 0.005)
  --invert-yaw         Invert yaw axis
  --invert-pitch       Invert pitch axis
  --invert-roll        Invert roll axis
  --sensitivity <F>    Output sensitivity multiplier (default 1.0)
  --log-imu <PATH>     Append raw IMU samples to file (JSONL)
  --log-pose <PATH>    Append filtered pose to file (JSONL)
  --replay <PATH>      Dev/testing: read IMU from a JSONL file instead of USB
  --log-level <LEVEL>  error|warn|info|debug (default error — warnings hidden)
  --gyro-calib <SEC>   Startup gyro-bias calibration window (default 2.0, 0=off)
  --input <SOURCE>     imu|visual (default imu; visual = P2 6-DoF, IMU off — under construction)
  --version            Print version and exit
  --help               Print help and exit
```

## Wire format (spec §4.2 + OQ1)

- **Classic** (default): 48 bytes, native-endian 6×f64
  `[TX, TY, TZ, Yaw, Pitch, Roll]`, translation `0` (3-DoF).
- **Extended**: 80 bytes, 10×f64 — the same 6 + `[1.0, 0, 0, 0]`
  (pose-valid default + 3 reserved doubles).
- Stateless datagrams, no handshake; sent at `--udp-rate` Hz (rate-gated, max 60 by default).
  With a 200 Hz IMU stream the decimator yields 50 Hz (every 4th sample —
  "60 Hz max, no minimum" per spec §4.2; set `--udp-rate` higher to emit every
  3rd sample if you need ≥ 60 Hz).
- **Units: degrees by default** (`--units deg|rad`).

> ⚠ **Deliberate deviation from the spec text:** DEVELOPMENT_SPEC.md says
> "radians on the wire" (§3.3, §4.2, §4.4), but the stock Opentrack UDP
> tracker copies the received doubles raw and adds ±90/±180° offset options
> ([`tracker-udp/ftnoir_tracker_udp.cpp`](https://github.com/opentrack/opentrack/blob/master/tracker-udp/ftnoir_tracker_udp.cpp))
> — it interprets rotation as **degrees**. Sending radians would overshoot
> ~57× in game. Resolved by user decision: degrees default, `--units rad`
> keeps the spec's variant reachable. The pose log follows the wire units.

## Exit codes & signals (spec §3.2, §3.5)

| Code | Meaning |
|------|---------|
| 0 | Clean shutdown (Ctrl-C / replay EOF) |
| 1 | No Nreal Light on USB / replay file error / IMU start failure |
| 2 | CLI usage error |
| 3 | IMU read failure mid-run (USB unplug — restart to reconnect) |

- First Ctrl-C → flush logs, clean exit 0. Second Ctrl-C → immediate exit 1.
- Startup prints a confirmation line: `device=... kp=... ki=... out=... protocol=... units=... udp_rate=...Hz hud=...`.

## Log levels (`--log-level`)

Messages go to stderr, gated by verbosity. Default is `error` — the run is
quiet: only errors print (plus the startup line and HUD on stdout).

- `--log-level warning` — also shows throttled warnings (e.g. "UDP send
  failed … is Opentrack listening?" at most once per 5 s).
- `--log-level info` — also shows the gyro-bias calibration result and the
  measured `imu_rate`.
- `--log-level debug` — shows every UDP send failure (unthrottled).

## Yaw drift & gyro calibration (`--gyro-calib`)

The Nreal Light has **no magnetometer**, so yaw has no absolute reference and
any residual gyro bias integrates into linear yaw drift. Measured on real
glasses (2026-08-04): ~15°/60 s ≈ 0.25°/s constant — classic turn-on bias
left after the driver's static device calibration.

By default the tracker runs a **2 s stationary gyro-bias calibration at
startup**: keep the glasses still (table is ideal), it measures the mean gyro
while still and subtracts it from every sample. Drift after calibration is
well under the T3 budget. Adjust with `--gyro-calib <SECONDS>` (0 disables).
Calibration waits for stillness (max 5× the window) and logs a warning if the
device was moving.

Because the bias also drifts with temperature during a session (thermal
warm-up), the tracker **refreshes it in-run**: whenever the glasses rest still
for the same window mid-session, the bias is re-measured and swapped in
silently (visible as "in-run gyro bias refreshed" at `--log-level info`). No
persistence across boots — the turn-on bias is random per power cycle, so a
fresh 2 s measurement is always more accurate than a saved value.

## Scripts & tools

- `start-tracker.sh` — launcher: HUD + UDP output with the protocol switch
  (env-overridable: `PROTOCOL`, `HOST`, `PORT`, `UDP_RATE`, `UNITS`,
  `LOG_LEVEL`, `GYRO_CALIB`). Builds the release binary on first use.
- `tools/udp_listen.py` — UDP diagnostic that parses packets exactly like
  Opentrack's "UDP over network" tracker (48 B / 80 B, degrees, rejects
  NaN/Inf). Use it to isolate the tracker's UDP output from Opentrack's
  config: `python3 tools/udp_listen.py 10` in one terminal, run the tracker
  in another.
- `CHANGELOG.md` — per-release notes; P1 is pinned as tag `v0.1.0`.

## Conventions & implementation notes

- **Frames:** RUB body frame (+X right, +Y up, +Z back — matches `ar-drivers`
  and the Android sensor frame). Quaternion rotates world → body. Euler angles
  are YXZ Tait-Bryan (yaw → pitch → roll), extracted in
  `neuromancer-ahrs::quat_to_ypr`.
- **Yaw has no absolute reference** (no magnetometer in the Nreal Light):
  yaw is gyro integration, corrected only indirectly; slow drift is expected
  (spec §2.4, T3). Pitch/roll are gravity-stabilized.
- **`dt` from sample timestamps** (monotonic), clamped to 100 ms — robust to
  USB jitter (spec §3.3). Every IMU sample is consumed; outputs are
  rate-gated decimators inside each sink, single-threaded loop (spec §2.7).
- **f64 math in `neuromancer-ahrs`** (spec §2.4 mentioned f32): f64 chosen for
  deterministic drift behavior over long runs; the crate stays dependency-free.
- **Dependency tree is minimal:** `neuromancer-ahrs` has zero deps; the
  binary adds only `ar-drivers` (required USB input, spec §2.6), `ctrlc`
  (cross-platform Ctrl-C, needed for P4), and the path dep on the AHRS crate.
  No async runtime, no GUI.
- **`ar-drivers` 0.4.3 quirk:** its `nreal` feature map omits `rusb`, but the
  Nreal Light backend uses rusb/libusb unconditionally — the tracker enables
  `features = ["nreal", "rusb"]` explicitly (spec §5.4: libusb/WinUSB on Windows).

## Docs / spec status

- Development spec: `DEVELOPMENT_SPEC.md` (approved; open questions OQ1/OQ3
  are being resolved in implementation — see this README).
- `cargo doc --open` for the `neuromancer-ahrs` API documentation.
