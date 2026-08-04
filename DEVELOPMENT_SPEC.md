# Nreal Light Head Tracker — Development Specification

**Document owner:** Neuromancer (Hermes, default profile)
**Prepared for:** Codex CLI (DeepSeek V4 Flash backend)
**Status:** Complete — reviewed by Neuromancer 2026-08-01, approved by V (sections 1–4), Codex-ready. **Phase 1 implemented 2026-08-03 — see Appendix C (implementation handoff); V hardware testing scheduled 2026-08-04. Phase 2 started 2026-08-04 — see Appendix D (P2 visual-only plan, approved); P1 pinned as git tag `v0.1.0`**
**Last updated:** 2026-08-01

---

## 1. Overview

### 1.1 Purpose

This document specifies a cross-platform **head tracker** for the
**Nreal Light AR glasses** that streams orientation (yaw/pitch/roll) to
**Opentrack** for game/VR-style head-look control.

The tracker replaces a previously attempted camera-based SLAM pipeline
(OpenCV StereoBM + FAST-9/BRIEF/DLT/ESKF) which suffered persistent X/Z
position drift and was abandoned. The final product offers a **choice of
3-DoF and 6-DoF tracking modes**. Phase 1 delivers stable IMU-only 3-DoF;
later phases add 6-DoF, Opentrack integration, and a Windows GUI build.

### 1.2 Phased roadmap (final product = 3-DoF + 6-DoF choice)

The end product must let the user select **3-DoF** or **6-DoF** tracking.
Each phase is independently shippable; the phase-1 result is already usable
day-to-day.

| Phase | Goal | Deliverable | Mode |
|-------|------|-------------|------|
| **P1** | Stable **3-DoF** tracking | IMU-only orientation (yaw/pitch/roll), AHRS crate, CLI binary, **2 Hz HUD output** | 3-DoF |
| **P2** | **6-DoF** tracking | Add positional tracking (X/Y/Z) — IMU integration + source of position (visual/inertial, TBD) | 3-DoF + 6-DoF |
| **P3** | **Opentrack integration** | Reliable UDP streaming into Opentrack, axis mapping, calibration UX | 3-DoF + 6-DoF |
| **P4** | **Windows version + GUI** | Windows build on `mini` (.231), native GUI (tracker control, calibration, mode switch) | 3-DoF + 6-DoF |

Phase 1 (this spec's primary scope) targets goals G1–G7 below; phases 2–4
extend them (see section 5 for the full roadmap).

### 1.3 Goals — Phase 1 (non-negotiable)

| # | Goal |
|---|------|
| G1 | Track head orientation in 3 degrees of freedom (yaw, pitch, roll) |
| G2 | Stream orientation to Opentrack as a UDP pose at ≥ 60 Hz |
| G3 | Reusable AHRS module — decoupled from hardware, usable by other projects |
| G4 | Cross-platform architecture: Linux (dev) and Windows (target: `mini` @ 192.168.0.231) |
| G5 | No camera / SLAM / OpenCV dependency in phase 1 |
| G6 | Deterministic, testable, documented |
| G7 | **2 Hz HUD output** — on-screen yaw/pitch/roll readout for visual capture and orientation verification |

### 1.4 Non-goals (explicitly out of scope for Phase 1)

- 6-DoF tracking / positional tracking (X/Y/Z translation fixed at `0` in P1)
- Camera-based SLAM, marker tracking, or visual odometry (P2 re-evaluates)
- Video passthrough or rendering
- Audio, hand tracking, or any other Nreal Light sensor
- Opentrack plugin development (tracker runs standalone, speaks UDP)
- GUI (P4)

### 1.5 Users & usage context

- **Primary user:** V — home lab, Steam Machine + gaming PC, AR experimentation
- **Runtime environment:** Nreal Light glasses → IMU data → tracker process →
  Opentrack UDP → game/application head-look
- **IMU data source:** the Nreal Light IMU (accelerometer + gyroscope),
  read **exclusively over USB** (see §2.3) — no network input path

### 1.6 High-level data flow

```
Nreal Light glasses
   │  IMU samples (accel + gyro) — USB (exclusive input)
   ▼
[IMU source layer]  ← ar-drivers via libusb (direct USB read)
   │  6×f64 [ax, ay, az, gx, gy, gz]
   ▼
[AHRS filter layer]  ← Mahony complementary filter (reusable crate)
   │  orientation quaternion
   ▼
[Coordinate & calibration layer]  ← axis mapping, inversion, sensitivity
   │  yaw/pitch/roll (degrees)
   ▼
[Output layer]
   ├─▶ UDP sender — Opentrack-compatible 6×f64 pose [0,0,0,yaw,pitch,roll]
   │      → Opentrack (UDP tracker) → game
   └─▶ HUD display — 2 Hz on-screen readout (yaw/pitch/roll)
          → visual capture / orientation verification
```

### 1.6a HUD output (Phase 1 — G7)

The tracker renders a **heads-up orientation readout** on screen at **2 Hz**
(two updates per second) so the headset orientation can be visually captured
(e.g. recorded on a phone/camera alongside the glasses) and reported back for
verification and calibration.

| Property | Value |
|----------|-------|
| Update rate | 2 Hz |
| Content | yaw, pitch, roll in degrees (live-filtered values) |
| Format | Large, high-contrast, human-readable — e.g. `YAW 12.3°  PITCH -4.1°  ROLL 0.8°` |
| Purpose | Visual capture, drift verification, calibration feedback |
| Implementation | Console line or simple on-screen overlay (no GUI framework in P1) |
| CLI flag | `--hud` (on/off), default off unless specified |
| Interaction with UDP | Independent — HUD runs alongside any output mode |

### 1.7 Existing assets (reuse, don't reinvent)

Section removed!

### 1.8 Success criteria

1. `cargo build --release` succeeds on Linux AND Windows (`mini` @ .231)
2. With real IMU data, yaw/pitch/roll track head motion smoothly in Opentrack
3. Latency ≤ 20 ms end-to-end at 60 Hz output
4. AHRS crate usable standalone (documented API + tests)
5. Runs headless, no GPU, < 50 MB RAM, single binary
6. HUD renders stable, readable yaw/pitch/roll at 2 Hz (`--hud`)

### 1.9 Open questions

- OQ1: Which Opentrack UDP protocol variant — classic 6×f64 or extended 10-double? Both - have a switch for the user to chose one!
- OQ2: Does `mini` have Rust toolchain installed, or does setup need to include it? Answer: To be tested when Windows implementation nears.
- OQ3: IMU sample rate of the Nreal Light over USB (100 Hz? 200 Hz?) — affects filter dt handling. Answer: test for best solution during first phase implementation
- OQ4: Should the tracker also log raw IMU to a file for offline replay/testing? Answer: with an appropriate switch

## 2. Backend Technologies

### 2.1 Language & toolchain

| Choice | Value | Rationale |
|--------|-------|-----------|
| Language | **Rust** (edition 2021) | Memory safety without GC, single static binary, first-class cross-compilation to Windows, existing codebase already in Rust |
| Toolchain | Stable Rust via rustup | `cargo build --release`; Windows target `x86_64-pc-windows-msvc` |
| Dev platform | Linux (this box, .240) | Primary development and testing |
| Build target | Windows (`mini` @ 192.168.0.231) | Phase 4; cross-compile with `cargo build --target x86_64-pc-windows-msvc` |
| Editions policy | Rust 2021 (current crates) | No forced migration; 2024 edition optional later |

### 2.2 Crate architecture

Section removed!

### 2.3 IMU data source (input)

| Source | Transport | Phase | Notes |
|--------|-----------|-------|-------|
| **USB (exclusive)** | ar-drivers via libusb, direct read of Nreal Light IMU | P1 (Linux) / P4 (Windows) | Feature-gated `rusb`; needs libusb per platform |

**The tracker reads IMU input exclusively from the glasses via USB.**
There is no network/input path — no UDP receive, no file input. The glasses
are the single source of accelerometer + gyroscope samples.

Implications:

- The tracker owns the USB connection to the glasses (no middleman capture app)
- USB availability on a platform is a hard requirement for the tracker to run
- Input decoding lives behind a thin `ImuSource` trait so the ar-drivers
  backend can be swapped/upgraded without touching the filter or outputs
- Windows USB support (ar-drivers/libusb on Windows) is required for P4 —
  flagged as a P4 dependency risk (see §5)
- Replay/testing feeds recorded samples into the same `ImuSource` abstraction
  at the unit/integration level, not through a live input path

### 2.4 AHRS / sensor fusion

| Aspect | Choice | Rationale |
|--------|--------|-----------|
| Algorithm | **Mahony complementary filter** | Proven for 3-DoF orientation, cheap (f32 math only), already implemented and tested in `neuromancer-ahrs` |
| Gains | `kp` (proportional), `ki` (integral) | CLI-tunable at runtime (`--kp`, `--ki`); defaults 1.0 / 0.005 |
| Output | Unit quaternion → yaw/pitch/roll | YXZ Tait-Bryan convention, RUB frame, degrees for HUD, degrees on the wire by default (`--units deg|rad`) |
| Fusion rate | ≥ 100 Hz input, output at configured rate | Filter consumes every sample; output loop decimates |

No magnetometer in Nreal Light — heading (yaw) reference comes from gyro
integration + accel gravity correction. Expect slow yaw drift; HUD + P3
calibration covers this.

### 2.5 Outputs (see Section 4 for full spec)

| Output | Transport | Rate | Phase |
|--------|-----------|------|-------|
| Opentrack UDP | UDP 6×f64 `[0,0,0,yaw,pitch,roll]` | ≥ 60 Hz | P1 (proto) / P3 (polish) |
| HUD | Console/overlay text | 2 Hz | P1 |
| Raw IMU log | File (JSONL) | sample rate | P1 (`--log-imu`) |

### 2.6 Third-party dependencies (minimal by design)

| Crate | Where | Why |
|-------|-------|-----|
| `ar-drivers` (**required**, not optional) | tracker binary | USB IMU source — the only input path (§2.3); no feature gate, it is a default dependency |
| `clap` (optional) | tracker binary | CLI parsing — may keep hand-rolled parser for zero deps (already implemented) |

**Policy:** keep the dependency tree as small as possible. The AHRS crate
must stay dependency-free. The binary may add small, well-known crates
(`clap` for CLI if the hand-rolled parser outgrows its use) but nothing
heavy — no tokio, no async runtime, no GUI in P1.


### 2.7 Threading & performance model

- Single-threaded event loop (read IMU → filter → send) is sufficient for P1
- No async runtime — plain `std` sockets with `set_nonblocking`/timeouts
- Target: < 10% of one core, < 50 MB RAM, no GPU
- Latency budget: filter < 1 ms, send < 1 ms, socket buffering dominates —
  well under the 20 ms end-to-end target

### 2.8 Testing & validation

| Level | What | Tooling |
|-------|------|---------|
| Unit | Mahony convergence, quaternion math, YPR extraction | `cargo test` |
| Integration | USB IMU samples (recorded) → tracker → UDP pose round-trip | `cargo test` + replay script |
| Replay | Deterministic IMU recording → expected pose | Raw IMU log (OQ4) replayed into filter |
| Manual | HUD visual verification, real glasses | `--hud`, physical capture |

### 2.9 Versioning & release

- Semver on both crates; workspace-level `Cargo.toml` optional (not required)
- Releases: `cargo build --release` → single binary per platform
- Windows binary produced on `mini` (native build) or cross-compiled from Linux

## 3. Functionality

### 3.1 Functional overview (Phase 1)

The tracker is a **long-running headless process** that:

1. Connects to the Nreal Light glasses over **USB** (sole input path, §2.3)
2. Streams IMU samples (accel + gyro) through the Mahony filter in real time
3. Continuously publishes the fused orientation:
   - **UDP** → Opentrack-compatible pose packets (≥ 60 Hz)
   - **HUD** → 2 Hz on-screen yaw/pitch/roll readout (`--hud`)
4. Keeps running until stopped — no interaction required once started

### 3.2 Startup sequence

| Step | Action | Failure behavior |
|------|--------|------------------|
| 1 | Parse CLI args (validate flags/values) | Print usage, exit code 2 |
| 2 | Open USB connection to glasses (ar-drivers) | Clear error + exit code 1: "no Nreal Light found on USB" |
| 3 | Start IMU stream (accel + gyro) | Error + exit code 1 |
| 4 | Initialize filter (kp/ki, neutral quaternion) | — |
| 5 | Bind UDP sender (host/port) if enabled | Warn + continue (HUD still works) |
| 6 | Enter main loop | — |

Startup must print a **confirmation line** with the detected device and
effective settings (e.g. `device=NrealLight imu_rate=200Hz kp=1.0 ki=0.005 out=127.0.0.1:4242`).

### 3.3 Main loop (per-iteration behavior)

```
loop {
    sample = usb_read_imu()            // blocking read, next sample
    quat   = mahony.update(sample)     // filter step (dt from sample timing)
    ypr    = quat_to_ypr(quat)         // yaw/pitch/roll, degrees
    apply_axis_mapping(&mut ypr)       // inversion flags, sensitivity
    if udp_enabled && due(60 Hz): send_pose(ypr)   // wire units: deg default (--units)
    if hud_enabled && due(2 Hz):   hud_print(ypr)  // degrees on screen
}
```

Key properties:
- **Every IMU sample is consumed** — no downsampling at input
- Output loops are **time-gated decimators** (send at most every N ms), not
  separate threads — keeps P1 single-threaded and deterministic
- `dt` for the filter is derived from **sample arrival timestamps** (monotonic
  clock), not assumed constant — robust to USB jitter

### 3.4 CLI interface (Phase 1)

```
neuromancer-tracker [OPTIONS]

  --host <IP>          Opentrack host (default 127.0.0.1)
  --port <PORT>        Opentrack UDP port (default 4242)
  --no-udp             Disable UDP output (HUD-only mode)
  --hud                Enable 2 Hz on-screen orientation readout
  --hud-rate <HZ>      HUD update rate (default 2)
  --kp <FLOAT>         Mahony proportional gain (default 1.0)
  --ki <FLOAT>         Mahony integral gain (default 0.005)
  --invert-yaw         Invert yaw axis
  --invert-pitch       Invert pitch axis
  --invert-roll        Invert roll axis
  --sensitivity <F>    Output sensitivity multiplier (default 1.0)
  --log-imu <PATH>     Append raw IMU samples to file (JSONL)
  --log-pose <PATH>    Append filtered pose to file (JSONL, wire units)
  --replay <PATH>      Dev/testing: read IMU from a JSONL file instead of USB
  --log-level <LEVEL>  error|warn|info|debug (default error — warnings hidden)
  --gyro-calib <SEC>   Startup gyro-bias calibration window (default 2.0, 0=off)
  --version            Print version and exit
  --help               Print help and exit
```


### 3.5 Runtime behavior details

**HUD (G7):**
- Prints one line every 500 ms: `YAW 12.3°  PITCH -4.1°  ROLL 0.8°`
- Uses `\r` (carriage return) to redraw in place on the same line — stable
  terminal output for video capture; falls back to newline if not a TTY
- Values are the **live filtered** orientation (post axis-mapping, degrees)

**UDP output:**
- 48-byte native-endian 6×f64 `[TX, TY, TZ, Yaw, Pitch, Roll]`
- X/Y/Z = 0 (3-DoF, P1); YXZ Tait-Bryan; **degrees** on the wire by default (see §4.2 Units)
- Sent to `--host:--port` at up to 60 Hz (or `--hud`-only when `--no-udp`)

**Axis mapping (coordinate & calibration layer):**
- Inversion flags flip sign of the corresponding angle
- Sensitivity scales angle (deg) before output
- P1: flags only (no UI); P3 adds interactive calibration (§5)

**Signals & shutdown:**
- Ctrl-C / SIGINT → flush log, close USB cleanly, exit 0
- Second Ctrl-C → immediate exit 1
- USB unplug mid-run → clear error, exit code 3 (restart to reconnect)

**Logging:**
- `--log-imu`: append-only JSONL `{t, ax, ay, az, gx, gy, gz}` for replay/tests
- `--log-pose`: optional pose stream log for latency/drift analysis
- Default: minimal stderr logging (startup line, errors only)

### 3.6 Phase 2+ functionality preview (see §5)

- **P2 (6-DoF):** mode flag `--mode 3dof|6dof`; positional source hooks into
  the output layer; HUD gains X/Y/Z readout
- **P3 (Opentrack):** interactive calibration (reset origin, axis wizard),
  protocol variant selection (OQ1), drift-correction UX
- **P4 (GUI):** native GUI wraps all CLI flags; live plot; mode switch
  becomes a UI control instead of a flag

### 3.7 Acceptance tests (Phase 1)

| Test | Procedure | Pass condition |
|------|-----------|----------------|
| T1 | `cargo build --release` on Linux | Builds clean |
| T2 | Run with glasses on USB, `--hud`, rotate headset by hand | HUD yaw/pitch/roll tracks motion with correct sign and roughly correct magnitude |
| T3 | Hold headset still on table 60 s | Yaw drift < 5° (pitch/roll stable) |
| T4 | `--no-udp --hud` | Runs HUD-only; no UDP packets (verify with `tcpdump`/Wireshark) |
| T5 | Default run with Opentrack listening | Opentrack receives pose; head-look works in game |
| T6 | Unplug USB mid-run | Clean error, exit code 3 |
| T7 | `--log-imu /tmp/imu.jsonl` + replay through filter in tests | Replayed pose matches live run within tolerance |
| T8 | `cargo test` (ahrs crate) | 9 existing tests pass, no regressions |

## 4. Output Options

### 4.1 Output matrix (Phase 1)

| Output | Enabled by | Transport | Rate | Content | Phase |
|--------|-----------|-----------|------|---------|-------|
| **Opentrack UDP** | default | UDP, 48-byte 6×f64 | ≤ 60 Hz | pose `[0,0,0,yaw,pitch,roll]` | P1 (proto) / P3 (polish) |
| **HUD** | `--hud` | stdout / console | 2 Hz (default, `--hud-rate`) | yaw/pitch/roll degrees | P1 |
| **IMU log** | `--log-imu <PATH>` | file, JSONL | sample rate | raw `{t,ax,ay,az,gx,gy,gz}` | P1 |
| **Pose log** | `--log-pose <PATH>` | file, JSONL | output rate | filtered pose + timestamps | P1 |

Outputs are **independent and combinable** — HUD can run with or without UDP,
logs can run alongside either. `--no-udp` disables only the UDP path.

### 4.2 Opentrack UDP protocol (primary output)

**Wire format — 48-byte native-endian 6×f64:**

```
offset  size  field      meaning
0       8     TX         translation X — 0.0 (3-DoF)
8       8     TY         translation Y — 0.0 (3-DoF)
16      8     TZ         translation Z — 0.0 (3-DoF)
24      8     Yaw        yaw   (degrees by default, YXZ Tait-Bryan)
32      8     Pitch      pitch (degrees by default, YXZ Tait-Bryan)
40      8     Roll       roll  (degrees by default, YXZ Tait-Bryan)
```

| Property | Value |
|----------|-------|
| Byte order | **native endianness** of the sending machine (Opentrack reads host-endian) |
| Frame | right-up-back (RUB) |
| Euler convention | YXZ Tait-Bryan (yaw → pitch → roll order) |
| Units | **degrees** on the wire by default (`--units deg|rad`) — resolved in P1: Opentrack's UDP tracker reads rotation as degrees (verified in `tracker-udp/ftnoir_tracker_udp.cpp`); the original "radians" here would overshoot ~57× |
| Rate | 60 Hz max; no minimum — send latest pose whenever due |
| Port | default 4242 (Opentrack "UDP over network" tracker) |
| Host | default 127.0.0.1; any LAN host supported |
| Packet behavior | stateless — each packet is a complete pose; no handshake, no ack |

**OQ1 — resolved in P1:** the tracker ships a `--protocol classic|extended`
switch (default classic). Classic = 48-byte 6×f64; extended = 80-byte 10×f64
with X/Y/Z slots 0–2 at 0 and fields 6–9 defaulting to `[1.0, 0, 0, 0]`
(pose-valid default + reserved). Verified against opentrack master: the
stock UDP tracker reads only the first 6 doubles (`readDatagram(..., 48)`)
and interprets rotation in **degrees** — see the Units row above.

### 4.3 HUD output (G7)

| Property | Value |
|----------|-------|
| Rate | 2 Hz default, `--hud-rate <HZ>` overridable |
| Line format | `YAW 12.3°  PITCH -4.1°  ROLL 0.8°` |
| Redraw | in-place via `\r` when TTY; newline fallback when piped |
| Values | live filtered orientation, post axis-mapping, **degrees** |
| Purpose | visual capture on camera; drift verification; calibration feedback |
| Implementation | plain stdout — no GUI, no rendering framework (P1) |

HUD is the **verification interface** for phase 1: record the screen
alongside the glasses and read back the orientation the tracker believes
the headset has.

### 4.4 Logging outputs

**IMU log (`--log-imu <PATH>`, JSONL, append):**
```json
{"t": 1785530000.123, "ax": 0.02, "ay": -0.98, "az": 0.05, "gx": 0.001, "gy": -0.002, "gz": 0.0005}
```
- `t` = monotonic seconds since start; one JSON object per line
- Used for: deterministic replay in tests, filter tuning offline, bug repro

**Pose log (`--log-pose <PATH>`, JSONL, append):**
```json
{"t": 1785530000.133, "yaw": 0.21, "pitch": -0.07, "roll": 0.01}
```
- Values in the selected wire units (`--units`, default **degrees**); used
  for latency/drift analysis

Both logs are best-effort: a write failure logs a warning and continues —
never crashes the tracker loop.

### 4.5 Output-layer architecture

```
[Orientation core]  (quaternion → ypr, axis mapping applied)
        │
        ├──▶ [UdpSink]    — 48-byte pose, rate-gated (default on)
        ├──▶ [HudSink]    — console line, rate-gated (--hud)
        ├──▶ [PoseLogSink]— JSONL file (--log-pose)
        └──▶ [ImuLogSink] — raw samples at source (--log-imu)
```

- Each sink implements a small `Sink` trait (`fn write(&mut self, frame)`);
  the main loop holds a list and calls each enabled sink when its rate gate
  fires — single-threaded, deterministic
- Sinks are **write-only consumers** — no sink feeds back into the filter
- Adding a Phase-2 6-DoF output = new sink + richer frame type (X/Y/Z ≠ 0)

### 4.6 Phase 2–4 output evolution

| Phase | Change |
|-------|--------|
| **P2 (6-DoF)** | Frame gains real TX/TY/TZ (position); HUD adds X/Y/Z line; pose log includes position; UDP unchanged in shape |
| **P3 (Opentrack)** | Protocol variant selection (OQ1), interactive calibration feedback via HUD, drift-correction hints |
| **P4 (GUI)** | HUD becomes a GUI panel with live plot; logs stream into the app; UDP output remains the game-facing path |

## 5. Build & Deployment

### 5.1 Build prerequisites

| Platform | Toolchain | Extra deps |
|----------|-----------|------------|
| **Linux (dev, .240)** | Rust stable (rustup) | `libusb-1.0-0-dev` (for ar-drivers/rusb) |
| **Windows (`mini` @ 192.168.0.231)** | Rust 1.97.1 (MSVC target) + VS Build Tools 2022 — **already installed & verified** | libusb runtime (WinUSB driver for Nreal Light — see 5.4) |

Both platforms build from the **same workspace** — no platform-specific code
outside the USB source backend (`ar-drivers` handles the OS differences).

### 5.2 Build & test commands (canonical)

```bash

# Linux dev build
cargo build --release
cargo test                                   # ahrs crate tests (9 + new)

# Windows build (on mini, native — preferred) or cross-compile from Linux
cargo build --release --target x86_64-pc-windows-msvc

# HUD-only run (no UDP output) — requires glasses on USB
cargo run --release -- --no-udp --hud
```

Cross-compiling `ar-drivers`/`rusb` from Linux to Windows is possible but
fiddly (libusb sys crates); **prefer a native build on `mini`** — that is
also the P4 acceptance target.

### 5.3 Deployment topology

```
.dev box (Linux .240)                mini (Windows .231)          Gaming/VR PC
┌──────────────────────┐        ┌──────────────────────┐     ┌──────────────────┐
│ repo + codex + tests │  git   │ build + run tracker  │ UDP │ Opentrack + game │
│ (dev/build/cross)    │ ─────▶ │ (glasses via USB)    │────▶│ (head-look)      │
└──────────────────────┘        └──────────────────────┘     └──────────────────┘
```

- **Development & test:** Linux (.240) — Codex works here
- **Phase-1 runtime:** Linux (.240) with glasses plugged in via USB
- **Phase-4 runtime:** Windows (`mini`, .231) — native build, glasses via USB,
  UDP out to the gaming PC (LAN) or local Opentrack
- **Wake-on-LAN:** `mini` is woken from .240 (`wakeonlan <MAC>`) when a build
  or run is needed; sleeps otherwise

### 5.4 USB on Windows (P4 dependency risk — flagged)

| Item | Detail |
|------|--------|
| Library | ar-drivers + rusb (libusb bindings) |
| Driver | Nreal Light needs the **WinUSB** driver bound to its USB interface (via Zadig) on Windows |
| Risk | rusb/libusb on Windows is well-trodden, but the Nreal Light's specific interfaces must be verified on `mini` |
| Mitigation | Validate in P4 week 1; fallback: WinUSB via Zadig + documented setup script |
| Impact if blocked | P4 USB input fails → tracker cannot run on Windows at all (input is USB-exclusive, §2.3) |

This is the **single biggest P4 risk** — Codex should treat it as a
dependency to de-risk early, not a final-week surprise.

### 5.5 Full phase roadmap (detail)

| Phase | Scope | Exit criteria | Where |
|-------|-------|---------------|-------|
| **P1 — 3-DoF core** | USB input (Linux), Mahony, YPR, UDP out, HUD, logs, CLI | T1–T8 (§3.7) pass; HUD verified by visual capture | Linux .240 |
| **P2 — 6-DoF** | Positional tracking source (visual/inertial TBD), `--mode 3dof\|6dof`, position in frames/HUD/logs | 6-DoF pose tracks position with acceptable drift; mode switch works | Linux .240 |
| **P3 — Opentrack polish** | Protocol variant confirm (OQ1), interactive calibration (reset origin, axis wizard), drift correction UX | Calibration flow usable; Opentrack integration solid across games | Linux .240 |
| **P4 — Windows + GUI** | Native Windows build on `mini`, WinUSB driver setup, GUI (tracker control, calibration, mode switch, live plot) | Runs on `mini` with glasses; GUI wraps all CLI features | mini .231 |

### 5.6 Release & distribution

- **Versioning:** semver; `neuromancer-tracker vX.Y.Z` + `neuromancer-ahrs vX.Y.Z`
- **Artifacts per release:**
  - `neuromancer-tracker-linux-x86_64` (static-ish binary)
  - `neuromancer-tracker-windows-x86_64.exe` (built on `mini`)
  - optional: `neuromancer-ahrs` crate (crates.io or vendored path dep)
- **Install (Linux):** copy binary to `~/.local/bin/`
- **Install (Windows, P4):** installer or portable exe + WinUSB setup script
- **Docs:** README with quick start (USB plug → run → Opentrack), CLI reference,
  troubleshooting (device not found, driver issues, drift)

### 5.7 Operational notes

- Tracker is **user-launched** (no daemon/service in P1–P3); P4 GUI adds
  optional auto-start
- Resource envelope: < 10% core, < 50 MB RAM, no GPU — runs alongside games
  and Opentrack without contention
- Logs are append-only and bounded by disk; `--log-imu` long sessions should
  be rotated manually or by OS tools (no built-in rotation in P1)

---

## Appendix A — Open questions tracker

| ID | Question | Status |
|----|----------|--------|
| OQ1 | Opentrack UDP protocol variant (6×f64 vs 10-double)? | **Resolved — both**: `--protocol classic|extended` (default classic). Wire units **degrees** by default, `--units deg|rad` (stock Opentrack reads degrees — verified in opentrack source, §4.2) |
| OQ2 | Rust toolchain on `mini`? | **Resolved — yes**: Rust 1.97.1 (MSVC target) + VS Build Tools 2022 verified 2026-08-01 (see [[Development/environment/mini/README\|mini env docs]]) |
| OQ3 | Nreal Light IMU sample rate over USB? | **Resolved — measured**: ~**1650 Hz** on real glasses (V's first run, 2026-08-03); tracker prints `imu_rate=<Hz>` after the first 30 samples; dt is timestamp-derived with a 100 ms clamp, so any rate is handled automatically (C.1) |
| OQ4 | Raw IMU logging? | **Resolved — yes**, `--log-imu` (§3.4, §4.4) |

## Appendix B — Document status

- §1 Overview — **approved by V** (phased roadmap, HUD added)
- §2 Backend Technologies — **approved** (USB-exclusive input correction applied; ar-drivers made required dep)
- §3 Functionality — **approved**
- §4 Output Options — **approved**
- §5 Build & Deployment — **reviewed** (toolchain on mini verified — OQ2 resolved)
- Status: **complete and Codex-ready** — handoff next

## Appendix C — Phase 1 implementation handoff (2026-08-03)

Phase 1 (3-DoF, Linux) is implemented per this spec in the two-crate workspace
(`crates/neuromancer-ahrs`, `crates/neuromancer-tracker`). 46 tests pass,
clippy clean, `cargo build --release` verified. Commits: `5ae2119`, `0d51687`,
`0bf47c5`. See `README.md` for quick start + full CLI reference.

### C.1 Research findings resolved during implementation

- **OQ1 — Opentrack UDP protocol.** Stock Opentrack's "UDP over network"
  tracker reads exactly 6×f64 and copies them raw into its pose, then offers
  ±90/±180° offsets — verified in `tracker-udp/ftnoir_tracker_udp.cpp`
  (opentrack master). ⇒ rotation on the wire is **degrees**, not radians.
  The 10-double "extended" variant is NOT an opentrack protocol: it is a
  third-party (smartphone-app) convention — the same 6 doubles + 4 reserved
  doubles; stock opentrack truncates the datagram to 48 bytes and ignores
  them. Shipped: `--protocol classic|extended` (default classic); extended
  fields 6–9 = `[1.0, 0, 0, 0]` (pose-valid default + reserved).
- **Wire units.** The original spec text said "radians on the wire"; sending
  radians to stock opentrack overshoots ~57× (breaks T5). Resolved: **degrees
  by default**, `--units deg|rad` switch. The pose log follows the selected
  wire units. Spec §2.4/§3.3/§3.5/§4.2/§4.4 updated accordingly.
- **OQ3 — IMU sample rate.** Not fixed in advance; the tracker measures and
  prints `imu_rate=<Hz>` after the first 30 samples. Synthetic replay showed
  200 Hz; **real glasses measured ~1650 Hz** (V's first hardware run,
  2026-08-03) — the Nreal Light IMU delivers one report per USB read at that
  rate. `dt` is timestamp-derived with a 100 ms clamp, so the filter handles
  it without changes.
- **ar-drivers 0.4.3 quirk.** Its `nreal` feature map omits `rusb`, but the
  Nreal Light backend (`nreal_light.rs`) uses rusb/libusb unconditionally —
  the crate only compiles with `features = ["nreal", "rusb"]`. Spec §2.3's
  "ar-drivers via libusb" is confirmed correct.
- **Mahony convention.** Canonical kinematics `q̇ = ½ q ⊗ ω` (world→body,
  body-frame gyro; Madgwick/Mahony reference impl, Solà) with error term
  `v × a` (estimated × measured gravity). Convergent from 30–40° tilts;
  a dedicated discriminating test (`mahony_body_frame_gyro_composes_right`)
  guards the operand order. Math is f64 (spec §2.4 mentioned f32) for
  deterministic long-run drift behavior; the crate stays dependency-free.
- **Yaw drift — root cause found (2026-08-04, V's hardware).** Glasses steady
  on a table drifted **+15°/60 s** — a constant ≈ 0.25°/s, the classic
  signature of residual **gyro turn-on bias** (left after `ar-drivers`' static
  device calibration). With no magnetometer, yaw has no reference, so any
  bias integrates linearly. Fix: **startup stationary gyro-bias calibration**
  (`--gyro-calib`, default 2 s) measures the mean gyro while still and
  subtracts it. Verified in a reproduction test: same 0.25°/s bias → 15°/60 s
  uncalibrated, < 2.5° after calibration; live replay shows the calibrator
  recovering `gy=0.004363 rad/s` exactly. The bias also drifts with
  temperature mid-session, so the tracker **refreshes it in-run**: whenever
  the glasses rest still for the window, the bias is re-measured and swapped
  in (verified live: `gy=0.004370 → 0.008724` after an injected 0.25→0.5°/s
  drift). Persisting calibration across boots is deliberately NOT done —
  turn-on bias is random per power cycle, so a fresh 2 s measurement beats a
  saved value.

### C.2 Deliberate deviations / additions (vs spec text)

| # | Item | Why |
|---|------|-----|
| 1 | `--units deg|rad`, **degrees default** on the wire | Opentrack reads degrees (C.1); spec's "radians" would fail T5 |
| 2 | `--protocol classic|extended` switch | OQ1 — both variants, user switch |
| 3 | `--udp-rate <HZ>` flag (default 60) | G2 "≥ 60 Hz" vs §4.2 "60 Hz max" inconsistency; user chose flag |
| 4 | `--replay <PATH>` dev input | Hardware-free testing (deviation from §2.3 "no file input" — production input remains USB-exclusive) |
| 5 | Pose log = wire units, one line per filter frame | "Matches wire format" per §4.4; full rate for latency analysis |
| 6 | Rate decimator emits 50 Hz from a 200 Hz stream | Every 4th sample ≥ 16.7 ms interval — "60 Hz max, no minimum" §4.2; `--udp-rate` can raise it |
| 7 | HUD fixed-width columns (`{:6.1}`) | Stable columns for video capture (spec §4.3 intent) |
| 8 | `--log-level` (default `error`) — stderr warnings hidden by default | User request: no UDP warning spam unless `--log-level warning` (2026-08-04) |
| 9 | `--gyro-calib <SEC>` (default 2.0) — startup stationary gyro-bias calibration **+ in-run refresh on stillness** | Fixes residual-bias yaw drift measured at ~15°/60 s on real glasses (§C.1); 0 disables. In-run refresh tracks thermal bias drift mid-session; no persistence across boots (turn-on bias is random per power cycle) |

### C.3 Acceptance-test status (spec §3.7)

| Test | Status | Where |
|------|--------|-------|
| T1 `cargo build --release` (Linux) | **PASSED** | dev box (.240) |
| T4 `--no-udp --hud`, no UDP packets | **PASSED** (replay); re-run with glasses | verified via `--replay` — listener on 4242 saw no packets |
| T7 `--log-imu` + replay through filter | **PASSED** | integration test `replay_through_filter_tracks_yaw` |
| T8 `cargo test` (ahrs crate) | **PASSED** — 14 tests | `cargo test --workspace` (46 total) |
| T2 headset rotation → HUD sign/magnitude | **TOMORROW (needs glasses)** | hardware on .240 |
| T3 60 s still → yaw drift < 5° | **RE-TEST tomorrow** — 15°/60 s drift found (2026-08-04); root cause = residual gyro bias, fixed via startup calibration (§C.1) | hardware on .240 |
| T5 Opentrack receives pose, head-look works | **TOMORROW (needs glasses + Opentrack)** | hardware on .240 |
| T6 USB unplug mid-run → exit 3 | **TOMORROW (needs glasses)** | hardware on .240 |

### C.4 V's test checklist for tomorrow

```bash
cargo build --release && cargo test --workspace     # sanity
target/release/neuromancer-tracker --hud            # default UDP 127.0.0.1:4242 + HUD
target/release/neuromancer-tracker --no-udp --hud   # HUD only
target/release/neuromancer-tracker --log-imu /tmp/imu.jsonl --hud
```

1. **T2 sign check:** rotate the headset left/right (yaw), up/down (pitch),
   and tilt (roll). If an axis reads inverted vs head motion, use
   `--invert-yaw/--invert-pitch/--invert-roll` (or `--sensitivity` to scale).
   Note the convention: +yaw = turn left ("turning left is positive y" per
   ar-drivers). Calibrate with the flags and report the mapping back.
2. **T3 drift:** place the headset still on a table for 60 s; the tracker now
   calibrates the gyro bias at startup (default 2 s — keep it still), then
   watch HUD yaw — expect **well under 5°** now (was ~15° before the fix;
   the startup line shows `calib=2s`). If you lift the glasses and set them
   back down mid-run, the bias refreshes again on the next 2 s of stillness
   ("in-run gyro bias refreshed" at `--log-level info`). Run
   `--log-level info` to see the measured bias (`gy=0.004363 rad/s` ≈
   0.25°/s on the first run), and `--log-pose /tmp/pose.jsonl` for a numeric
   record. If drift is still large, capture `--log-imu /tmp/imu.jsonl` and
   report it — that tells us whether the residual is bias, scale, or
   temperature.
3. **T5 Opentrack:** start Opentrack, add the "UDP over network" tracker on
   127.0.0.1:4242, start the tracker, check head-look in a game or the
   Opentrack preview. The wire is 6×f64 **degrees**, native endianness.
   Note: if Opentrack is not running, the tracker prints a throttled
   "UDP send failed … Connection refused (is Opentrack listening?)" warning
   at most once per 5 s — that is expected and harmless.
4. **T6 unplug:** pull the USB cable mid-run — expect a clear error and exit
   code 3 (restart to reconnect).
5. Report back: HUD captures (phone video), the measured `imu_rate` line,
   drift numbers, exit codes. Any sign mismatch gets fixed in code or via the
   flags.

### C.5 Known caveats

- Yaw has no absolute reference (no magnetometer) — slow yaw drift is expected
  and only partially corrected; T3's 5°/60 s budget is the pass bar.
- Second Ctrl-C forces exit and may drop buffered log lines (flushing from a
  signal handler is not async-signal-safe — accepted trade-off).
- Windows USB (P4) risk unchanged: ar-drivers/libusb + WinUSB via Zadig (§5.4).
- IMU/pose logs are append-only with no rotation (§5.7).

## Appendix D — P2 visual-only plan (approved 2026-08-04)

Approved by V: **pure-Rust stereo visual odometry** as the P2a position source,
with an `--input imu|visual` switch. `imu` = Phase-1 3-DoF behavior exactly;
`visual` = 6-DoF pose fully from vision (the "turn the IMU off" switch — P1
had none); `imu+visual` fusion is P2b and reserved.

### D.1 Feasibility (verified in ar-drivers 0.4.3 source)

- `NrealLightSlamCamera::new()` + `get_frame(timeout)` → `NrealLightSlamCameraFrame
  { left: Vec<u8> (640×480 grayscale), right: Vec<u8>, timestamp: u64 }`,
  ~30 fps (UVC frame interval 333333×100 ns). "Only one instance can be
  alive at a time" — IMU+camera coexistence is a spike item.
- `cameras()` → `CameraDescriptor` per camera: intrinsic matrix, distortion
  (k1..k5), `leftcam_q_rightcam` stereo extrinsics, `imu_to_camera` — all
  needed for rectification, triangulation, and **metric scale** (stereo
  baseline), which directly addresses the X/Z drift that killed the original
  SLAM attempt (§1.1).

### D.2 Architecture

```
SlamCamera ──▶ [VisualSource] ──▶ rectify ──▶ FAST ──▶ KLT ──▶ stereo match
    640×480 stereo @30fps                                   │
                                              triangulate (metric depth)
                                                        ▼
                              PnP + RANSAC ──▶ incremental 6-DoF pose
                                                        │
                                     ┌───────────────────┴──────────┐
                                     ▼  UDP TX/TY/TZ real  HUD X/Y/Z  pose log position
```

- New crate `neuromancer-vo` (pure Rust, nalgebra — already in the tree via
  ar-drivers; no C++ deps, keeps P4 Windows cross-compile sane).
- Pose source abstraction so `imu` and `visual` share the output layer.
- Motion estimation uses **RANSAC + 3-point Umeyama (3D–3D) with Gauss–Newton
  reprojection refinement** instead of DLT-PnP: DLT-PnP degenerates on planar
  scenes, and the pipeline has metric stereo depth in both frames (M4 note).

### D.3 Scope & milestones

| M | Deliverable | Verification | Status |
|---|-------------|--------------|--------|
| M1 | `neuromancer-vo`: CameraModel + rectification + **synthetic stereo generator**; `--input imu|visual` switch | unit tests (projection round-trip, rectified epipolar geometry) | **DONE 2026-08-04** |
| M2 | FAST corner detection + KLT optical flow | synthetic frame pair tests | **DONE** — single-level LK (pyramid removed after testing: blurred coarse level biased estimates); 1.33/3.3/6.67 px motions at ≥90/≥90/≥70% inliers |
| M3 | Epipolar stereo matching + triangulation (metric depth) | depth vs known scene error | **DONE** — median relative depth error < 2% |
| M4 | Motion estimation + incremental 6-DoF pose | synthetic trajectory: pose error vs ground truth | **DONE** — RANSAC + Umeyama 3D-3D + Gauss–Newton reprojection refinement (replaces DLT-PnP: planar degeneracy, and stereo depth exists in both frames); < 5 mm/< 5 mrad well-constrained regime |
| M5 | Outputs per §4.6 (position in frames/UDP/HUD/pose log); `--input visual` end-to-end | integration tests + live replay | **DONE** — position in meters; UDP TX/TY/TZ in **cm** (Opentrack convention); HUD X/Y/Z; pose log x/y/z |
| M6 | `--record-visual` / `--replay-visual` (mirror `--log-imu`/`--replay`) | recorded frame replay | **DONE** — round-trip verified |
| M7 | Hardware spike on .240: frames @30 fps, intrinsics sanity, IMU+camera coexistence | physical | **DONE 2026-08-04 — see D.5 findings; intrinsics/rectification wiring deferred to M8** |

### D.4 Risks (flagged, per spec "honest expectation")

### D.5 M7 hardware-spike findings (2026-08-04, V on .240 with glasses)

All hardware checks passed (clean exit, no hangs); the spike exposed two
things that shape P2a's remaining work:

- **The SLAM camera itself delivers a rock-solid 30 fps** — the earlier
  "~6 fps device" reading was wrong. A dedicated probe
  (`cam_probe`, `crates/neuromancer-tracker/examples/cam_probe.rs`) that
  times every `get_frame` call shows a steady 33.3 ms cadence (p50 33.4,
  p99 33.8 ms, no jitter) with only the first frame paying a ~395 ms
  warm-up. The spec's 30 fps UVC config (`bFrameInterval = 333333`) holds.
- **The bottleneck is `ar-drivers` 0.4.3 `get_frame`, and it is
  alignment-dependent (bimodal, not a stable rate).** The read loop only
  accepts a bulk read where `recvd == 615908 && bulk_data[0] != 0`,
  otherwise it retries with the **full timeout** (`read_bulk(..., timeout)`
  instead of the computed `actual_timeout`). When the first bulk read
  lands mid-frame, every subsequent read consumes ~2 frame intervals
  (~74 ms) — a permanent ~13 fps state; when it lands on a frame
  boundary, reads are ~33 ms (30 fps). Probe runs were consistently
  either 29–30 fps or exactly ~10 fps, never in between. The tracker's
  old first-30-frame snapshot reported 6 fps when the 395 ms warm-up
  frame plus ~74 ms reads and ~26 ms VO landed inside the window.
- **VO pose estimation costs ~26 ms/frame on real frames (fast-fail is
  ~0 ms).** `vo.process` returns `None` (no pose) in ~0 ms, but a
  successful motion estimate (RANSAC + Umeyama + GN on ≤1500 features)
  costs ~26 ms. So end-to-end rate is: `33 ms read + 26 ms VO = ~59 ms`
  → **~17 fps best case with a pose every frame**, or the misaligned
  `74 + 26 = 100 ms` → **~10 fps**. Sustained-rate reporting was added
  to the tracker's visual mode so future runs distinguish warm-up noise
  from steady-state.
- **`ar-drivers` device timestamps are unusable for pacing analysis**
  (all deltas 0 — quantized by the `/1000 + 37600` offset path); wall
  clock is the only reliable timing.
- **Intrinsics are not yet wired** (known): VO runs on the hardcoded
  rectified rig (`fx=fy=500`, baseline 0.12 m), producing km-scale pose
  garbage on real scenes — expected, and the reason M8 (CameraDescriptor
  intrinsics + fisheye rectification + `imu_to_camera`) is the next
  milestone. 357 raw frames from the spike are preserved in
  `/tmp/spike_seq` for offline tuning.
- **IMU+camera coexistence confirmed** in both orders (camera→IMU and
  IMU→camera) — the D.1 "one source alive at a time" risk did not
  materialize.
- **Real IMU rate measured at 1000 Hz** (not the ~200 Hz assumed when
  OQ3 was closed) — dt handling is timestamp-derived and clamps, so this
  is handled automatically; note it for UDP decimation expectations.
- **Ctrl-C race found & fixed (commits `4b11637`+, root cause in
  `196f6bf`-era analysis):** SIGINT landing while the USB IMU read is
  blocked makes hidapi report "unplugged?" and the tracker wrongly exit 3.
  The first attempt (check the `CTRL_C` flag in the error path) was still
  racy — `ctrlc` increments the flag from a separate waiter thread, so the
  flag may not be set when the interrupted read returns. The real fix
  replaces `ctrlc` with a direct `sigaction` handler that increments the
  flag **in signal context** (async-signal-safe lock-free atomic; `_exit`
  on the second press; SA_RESTART deliberately unset so the blocking read
  returns EINTR). The flag is therefore always visible to the main thread
  before the read error is observed. Regression test
  `cli_sigint_mid_run_exits_0` (SIGINT to a running process must exit 0);
  `ctrlc` dependency removed.

**M7 action item for M8 — DONE 2026-08-04:** `ar-drivers` 0.4.3 is now
vendored (`vendor/ar-drivers`) with `get_frame` fixed: reads accumulate
into a persistent buffer and exactly one 615908-byte frame is extracted,
with the over-read tail (start of the next frame) carried over instead of
discarded, and retries using `actual_timeout`. The old code discarded any
read that was not exactly one frame, which — because a single `read_bulk`
often returns a frame *plus* the start of the next (both ending on short
packets) — forced a second blocking read (~74 ms/frame) permanently:
the ~13 fps alignment state. Probe-verified: 10/10 runs at 29.8 fps, no
13 fps state; record→replay round-trip intact. The VO pose cost (~26 ms
on the spike scene, up to ~108 ms on high-texture scenes) is now the
binding constraint in pose-heavy conditions (~6–10 fps end-to-end) and
is an M9 tuning item (RANSAC budget, keyframe thinning). Note: the
vendored crate's upstream `examples/` and dev-deps (clap, opencv — whose
build script fails on this toolchain) were removed; clippy is silenced
at the crate root as third-party code.

### D.6 Remaining P2a milestones

| M | Deliverable | Verification | Status |
|---|-------------|--------------|--------|
| M8 | Wire `ar-drivers` `CameraDescriptor` intrinsics + fisheye rectification into the rig; head-frame alignment (`imu_to_camera`) | intrinsics sanity on recorded spike frames; synthetic tests unchanged | **DONE 2026-08-04 — see D.7; real-scene pose stability is M9** |
| M9 | Real-scene VO tuning on `/tmp/spike_seq` (feature starvation, depth scale, drift) | pose plausibility on recorded session | **IN PROGRESS — see D.8/D.9: max-depth rejection + RANSAC gate + keyframe advance + RANSAC budget tuning landed; residual jitter tuning open** |

### D.7 M8 calibration wiring (2026-08-04)

M8 replaces the hardcoded rectified rig (`fx=fy=500`, cx=320, cy=240,
baseline 0.12 m) with the **glasses' on-device factory calibration**
(`NrealLight::cameras()` → `CameraDescriptor` from the `SLAM_camera`
config JSON) and rectifies each raw frame before the VO pipeline:

- **Intrinsics** (per camera): left `fx=234.46 fy=234.47 cx=325.16
  cy=245.73`, right `fx=234.40 fy=234.87 cx=318.99 cy=214.08`; distortion
  `kc` (k1..p2, OpenCV order) from device config. `cam_probe --calib`
  dumps them; `cam_probe --depth-check`/`--rect-check` verify on recorded
  frames.
- **Stereo extrinsics**: `left_t_right = imu_to_camera_left⁻¹ ∘
  imu_to_camera_right` — verified to reproduce the raw `leftcam_q_rightcam`
  rotation (0.0119 rad) and give a **+X baseline of 0.103 m** (the old
  hardcoded 0.12 was ~20% off).
- **Rectification** (`neuromancer-vo::rectify`): Bouguet-style rotations +
  radial/tangential undistortion, precomputed inverse remaps applied with
  bilinear sampling per frame. `Rectifier::apply` runs before
  FAST/KLT/stereo in both live and replay paths.
- **Head-frame alignment**: the visual pose is expressed in the IMU/head
  frame via `world_T_head = world_T_cam · imu_to_camera_left` — the first
  pose is exactly the IMU offset, so visual YPR is comparable to the P1 IMU
  YPR (cross-checked in M9).
- Fallback: replay mode (no device) uses this unit's captured constants
  (`cam_probe --calib`, 2026-08-04); the tracker only probes the live
  device in hardware mode (avoids MCU contention in parallel tests).
- Vendored `ar-drivers` got a defensive fix in the config parser
  (`config.len().saturating_sub(4)` — an empty config read previously
  panicked when the SLAM camera held the interface).

**Verification (M8 acceptance — PASSED):**
- **Rectification: epipolar lines horizontal** — stereo-match vertical
  offset on a recorded rectified pair: median **0.00 px**, mean 0.21 px
  (the raw pair is vertically misaligned by ~30 px: cy differs 245.7 vs
  214.1).
- **Metric depth sanity**: triangulated depths on two recorded pairs
  (`/tmp/spike_seq`, `/tmp/m8_seq`) are 100% within [0.3, 20] m, median
  ~0.6–1.0 m — plausible indoor scale (the old rig mis-scaled).
- Synthetic tests unchanged (20 vo tests incl. 3 new rectifier tests;
  full workspace 85 tests green; clippy clean).
- Live: `camera calibration: live device (baseline 0.103 m, coverage
  97.1%)` at startup; ~7 fps sustained (rectification adds per-frame
  remap cost).

**Remaining (M9):** real-scene pose stability. RANSAC still returns a
pose even when the inlier set is degenerate (0 inliers), producing
explosive inter-frame motion on low-texture/motion-blur frames — the
median inter-pose delta on a *still* headset should be ~0 but is
meter-scale on some frames. First M9 candidates: minimum-inlier gate in
`ransac_motion`, tighter feature/quality filters, and keyframe
management.

### D.8 M9 real-scene VO tuning (2026-08-04, first tranche)

Three changes land the biggest real-scene stability win; all verified on
live hardware (glasses, j35ter-A5):

- **Max triangulation distance (enclosed-space constraint).**
  `StereoMatcher::triangulate` now rejects any point whose depth falls
  outside `[min_depth, max_depth]` — the tracker uses `[0.5, 10] m`.
  Before this, subpixel disparity refinement could push points past the
  window (measured max 16.1 m); now `cam_probe --depth-check` reports
  **max = 10.0 m exactly** (4967 corners, median 0.95 m). The glasses are
  an indoor device; beyond 10 m the ~2.4 px disparity is noise, not
  geometry.
- **RANSAC minimum-inlier gate.** `ransac_motion` takes `min_inliers` and
  returns `None` when the refined inlier set is below it; `estimate_motion`
  uses `(src.len() / 40).max(20)` (~2.5% of correspondences, floor 20).
  Measured on real scenes: garbage poses have inliers 1–29, valid poses
  ≥30, so the gate cleanly separates them. Degenerate frames (bas-relief,
  motion blur, low texture) now return `None` instead of injecting
  explosive drift.
- **Keyframe always advances.** `VoPipeline::process` previously did
  `estimate_motion(...)?` — on failure the keyframe stayed stuck on the
  first frame. The first frame is the dark auto-exposure warm-up (13
  features), so the tracker could never recover: every subsequent frame
  tracked against the same bad keyframe and failed the gate. Now the
  keyframe advances every frame regardless of motion success; only the
  accumulated pose is held back on failure.

**Measured live (before → after):** poses sane (< 10 m) **9% → 100%**
(two 10–12 s runs: 20/20 and 23/23 sane); |pos| median 4.1e14 m → ~0.24 m;
inter-pose delta median 0.04–0.08 m, max 0.2–0.5 m (bounded, no
explosions).

### D.9 M9 drift tuning (2026-08-04, second tranche)

The first tranche bounded pose explosions but left drift: on a *still*
headset the accumulated path was 4.8–4.9 m over 20 s with endpoint drift
1.26–1.39 m, a systematic +z bias (mean +0.027 m/step, t = 2.0) and ~15°
systematic yaw drift. Cause: the loose RANSAC budget (300 iterations,
3.0 px reprojection threshold) admitted near-degenerate solutions that
GN refinement then polished. Tuning verified on the same live still
headset (j35ter-A5):

- **RANSAC budget 300 → 2000 iterations, reprojection threshold
  3.0 → 1.5 px** (both `estimate_motion` in `motion.rs` and
  `cam_probe --stats`). After: **path 1.46 m (−70%)**, **endpoint drift
  0.21 m (−83%)**, **+z bias gone** (mean −0.0012 m/step, t = −0.2),
  per-step median 0.042 m, max 0.17 m; yaw now only wanders (±12.4°)
  instead of drifting systematically.
- **Trade-off: fewer posed frames** (26 vs 45 per 20 s) — the tighter
  threshold rejects more frames. This is safe by design: rejected frames
  hit the D.8 min-inlier gate, return `None`, and the pipeline keeps the
  prior keyframe pose. Starvation is safe; looseness is not.
- Motion tests still pass (3, ~163 s — the iteration budget dominates
  runtime; a probabilistic early-exit remains a future lever).

**Remaining (M9):** residual per-frame jitter on a still headset, and
motion-heavy use of the 1.5 px threshold is untested (measured on a
still headset). Further levers: feature quality filters (FAST score),
keyframe thinning, tighter min-inlier gate. The real cure for residual
drift is IMU fusion (P2b, ESKF as originally planned).


- **Incremental VO drifts** (the old SLAM's killer); P2a targets the spec's
  "acceptable drift" bar as a stepping stone — the real cure is IMU fusion
  (P2b, ESKF as originally planned).
- Fisheye distortion/rectification quality (intrinsics from device config —
  validate in M7).
- Low-texture scenes starve FAST/KLT.
- CPU envelope: VO will exceed P1's "<10% of one core" target; acceptable
  for P2, revisit in P4.
- `ar-drivers` camera API maturity and IMU+camera exclusivity.
