# Nreal Light Head Tracker — Development Specification

**Document owner:** Neuromancer (Hermes, default profile)
**Prepared for:** Codex CLI (DeepSeek V4 Flash backend)
**Status:** Complete — reviewed by Neuromancer 2026-08-01, approved by V (sections 1–4), Codex-ready
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
| Output | Unit quaternion → yaw/pitch/roll | YXZ Tait-Bryan convention, RUB frame, degrees for HUD, radians on the wire |
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
  --log-pose <PATH>    Append filtered pose to file (JSONL, radians)
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
- X/Y/Z = 0 (3-DoF, P1); YXZ Tait-Bryan; **radians** on the wire
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
24      8     Yaw        yaw   (radians, YXZ Tait-Bryan)
32      8     Pitch      pitch (radians, YXZ Tait-Bryan)
40      8     Roll       roll  (radians, YXZ Tait-Bryan)
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
| OQ3 | Nreal Light IMU sample rate over USB? | Open — measure in P1 (affects dt handling) |
| OQ4 | Raw IMU logging? | **Resolved — yes**, `--log-imu` (§3.4, §4.4) |

## Appendix B — Document status

- §1 Overview — **approved by V** (phased roadmap, HUD added)
- §2 Backend Technologies — **approved** (USB-exclusive input correction applied; ar-drivers made required dep)
- §3 Functionality — **approved**
- §4 Output Options — **approved**
- §5 Build & Deployment — **reviewed** (toolchain on mini verified — OQ2 resolved)
- Status: **complete and Codex-ready** — handoff next
