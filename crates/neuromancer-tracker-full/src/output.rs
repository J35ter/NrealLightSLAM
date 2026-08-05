//! Output layer (spec §4): independent, combinable, rate-gated sinks.
//! Every sink is a write-only consumer and never feeds back into the filter.

use std::fs::{File, OpenOptions};
use std::io::{self, BufWriter, IsTerminal, Write};
use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::path::Path;
use std::time::{Duration, Instant};

use crate::cli::{Protocol, Units};
use crate::jsonl::{self, ImuSample};

/// One filtered pose handed to the sinks: `t` = stream time, `ypr_deg` =
/// yaw/pitch/roll **in degrees, post axis-mapping**, `position` = X/Y/Z in
/// meters (zero for 3-DoF / IMU mode; real for 6-DoF / visual mode).
#[derive(Debug, Clone, Copy)]
pub struct Frame {
    pub t: f64,
    pub ypr_deg: [f64; 3],
    pub position: [f64; 3],
}

impl Frame {
    /// 3-DoF frame (position fixed at origin).
    pub fn new_3dof(t: f64, ypr_deg: [f64; 3]) -> Self {
        Frame { t, ypr_deg, position: [0.0; 3] }
    }
}

/// Rate gate: allow an action at most every `interval`. First call passes.
#[derive(Debug, Clone)]
pub struct RateGate {
    interval: Duration,
    last: Option<Instant>,
}

impl RateGate {
    pub fn new(hz: f64) -> Self {
        RateGate {
            interval: Duration::from_secs_f64(1.0 / hz.max(1e-9)),
            last: None,
        }
    }

    /// `true` when the gate fires at `now` (updates the last-fire time).
    pub fn due(&mut self, now: Instant) -> bool {
        match self.last {
            None => {
                self.last = Some(now);
                true
            }
            Some(prev) if now.duration_since(prev) >= self.interval => {
                self.last = Some(now);
                true
            }
            Some(_) => false,
        }
    }
}

/// A write-only output consumer.
pub trait Sink {
    fn write(&mut self, frame: &Frame);
    fn name(&self) -> &'static str;
}

/// Convert degrees to the configured wire units.
fn wire_value(v_deg: f64, units: Units) -> f64 {
    match units {
        Units::Deg => v_deg,
        Units::Rad => v_deg.to_radians(),
    }
}

/// How often a failing UDP sink re-warns, at most (throttles the
/// "Connection refused" spam when the destination is not listening).
const UDP_WARN_INTERVAL: Duration = Duration::from_secs(5);

/// Opentrack UDP sink (spec §4.2): 48-byte 6×f64 (classic) or 80-byte
/// 10×f64 (extended), native endianness, stateless datagrams, rate-gated.
pub struct UdpSink {
    socket: UdpSocket,
    dest: SocketAddr,
    gate: RateGate,
    protocol: Protocol,
    units: Units,
    /// Last time a send failure was reported — warnings are throttled to at
    /// most one per `UDP_WARN_INTERVAL`. On Linux a connected socket to an
    /// unbound port alternates Ok/Err per send (ICMP port-unreachable is
    /// reported once, then consumed), so a simple warn-once-per-streak flag
    /// would flap and spam.
    last_warn: Option<Instant>,
}

impl UdpSink {
    pub fn bind(
        host: &str,
        port: u16,
        protocol: Protocol,
        units: Units,
        rate_hz: f64,
    ) -> io::Result<Self> {
        // Resolve host once (hostname or IP, v4/v6) and use the same address
        // for both the connect() and the reported destination.
        let dest: SocketAddr = (host, port)
            .to_socket_addrs()?
            .next()
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidInput, "cannot resolve host"))?;
        let socket = if dest.is_ipv4() {
            UdpSocket::bind("0.0.0.0:0")?
        } else {
            UdpSocket::bind("[::]:0")?
        };
        socket.connect(dest)?;
        Ok(UdpSink {
            socket,
            dest,
            gate: RateGate::new(rate_hz),
            protocol,
            units,
            last_warn: None,
        })
    }

    fn payload(&self, frame: &Frame) -> ([u8; 80], usize) {
        let mut buf = [0u8; 80];
        let yaw = wire_value(frame.ypr_deg[0], self.units);
        let pitch = wire_value(frame.ypr_deg[1], self.units);
        let roll = wire_value(frame.ypr_deg[2], self.units);
        // [TX, TY, TZ, Yaw, Pitch, Roll, 1.0, 0, 0, 0]
        // X/Y/Z in CENTIMETERS (Opentrack's FreeTrack-derived translation
        // convention — verified in opentrack source; see spec §4.2 note);
        // field 6 = pose-valid/confidence default; fields 7-9 reserved.
        let vals = [
            frame.position[0] * 100.0,
            frame.position[1] * 100.0,
            frame.position[2] * 100.0,
            yaw,
            pitch,
            roll,
            1.0,
            0.0,
            0.0,
            0.0,
        ];
        let n = match self.protocol {
            Protocol::Classic => 6,
            Protocol::Extended => 10,
        };
        for (i, v) in vals.iter().take(n).enumerate() {
            buf[i * 8..i * 8 + 8].copy_from_slice(&v.to_ne_bytes());
        }
        (buf, n * 8)
    }
}

impl Sink for UdpSink {
    fn write(&mut self, frame: &Frame) {
        if !self.gate.due(Instant::now()) {
            return;
        }
        let (buf, len) = self.payload(frame);
        match self.socket.send(&buf[..len]) {
            Ok(_) => {}
            Err(e) => {
                if crate::log::enabled(crate::log::Level::Debug) {
                    // Full verbosity: report every failure.
                    crate::log_debug!("udp send to {} failed: {e}", self.dest);
                } else if crate::log::enabled(crate::log::Level::Warn) {
                    // Throttled: at most one per interval (see struct docs).
                    let now = Instant::now();
                    let should_warn = self.last_warn.is_none()
                        || now.duration_since(self.last_warn.unwrap()) >= UDP_WARN_INTERVAL;
                    if should_warn {
                        self.last_warn = Some(now);
                        crate::log_warn!(
                            "UDP send to {} failed: {e} (is Opentrack listening on port {}?) — further warnings throttled to once/{:.0}s",
                            self.dest,
                            self.dest.port(),
                            UDP_WARN_INTERVAL.as_secs_f64()
                        );
                    }
                }
            }
        }
    }

    fn name(&self) -> &'static str {
        "udp"
    }
}

/// 2 Hz heads-up readout (spec §4.3): in-place `\r` redraw on a TTY,
/// newline fallback otherwise. Values are live filtered degrees.
pub struct HudSink<W: Write> {
    out: W,
    tty: bool,
    gate: RateGate,
}

impl<W: Write> HudSink<W> {
    pub fn new(out: W, tty: bool, hz: f64) -> Self {
        HudSink {
            out,
            tty,
            gate: RateGate::new(hz),
        }
    }

    /// HUD on stdout, TTY detection done once at construction.
    pub fn stdout(hz: f64) -> HudSink<std::io::Stdout> {
        HudSink::new(io::stdout(), io::stdout().is_terminal(), hz)
    }
}

impl<W: Write> Sink for HudSink<W> {
    fn write(&mut self, frame: &Frame) {
        if !self.gate.due(Instant::now()) {
            return;
        }
        let sep = if self.tty { '\r' } else { '\n' };
        let mut line = format!(
            "YAW {:6.1}°  PITCH {:6.1}°  ROLL {:6.1}°",
            frame.ypr_deg[0], frame.ypr_deg[1], frame.ypr_deg[2]
        );
        // 6-DoF: append the position readout (P1 output stays identical).
        if frame.position != [0.0; 3] {
            line.push_str(&format!(
                "  X {:6.2}m  Y {:6.2}m  Z {:6.2}m",
                frame.position[0], frame.position[1], frame.position[2]
            ));
        }
        line.push(sep);
        let _ = write!(self.out, "{line}");
        let _ = self.out.flush();
    }

    fn name(&self) -> &'static str {
        "hud"
    }
}

/// Pose log (spec §4.4): JSONL, append, best-effort. Values use the wire
/// units (`--units`), matching the UDP payload. Written once per filter
/// frame (full IMU rate) for latency/drift analysis.
pub struct PoseLogSink {
    out: BufWriter<File>,
    units: Units,
    failing: bool,
}

impl PoseLogSink {
    pub fn open(path: &Path, units: Units) -> io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(PoseLogSink {
            out: BufWriter::new(file),
            units,
            failing: false,
        })
    }
}

impl Sink for PoseLogSink {
    fn write(&mut self, frame: &Frame) {
        if self.failing {
            return;
        }
        let ypr = [
            wire_value(frame.ypr_deg[0], self.units),
            wire_value(frame.ypr_deg[1], self.units),
            wire_value(frame.ypr_deg[2], self.units),
        ];
        if let Err(e) = jsonl::write_pose(&mut self.out, frame.t, ypr, frame.position) {
            self.failing = true;
            crate::log_warn!("pose log write failed, disabled: {e}");
        }
    }

    fn name(&self) -> &'static str {
        "pose-log"
    }
}

impl Drop for PoseLogSink {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

/// Raw IMU log (spec §4.4): consumes samples at source, not filter frames.
/// Best-effort — a failing sink is disabled with a warning, never crashes
/// the loop.
pub struct ImuLogSink {
    out: BufWriter<File>,
    failing: bool,
}

impl ImuLogSink {
    pub fn open(path: &Path) -> io::Result<Self> {
        let file = OpenOptions::new().create(true).append(true).open(path)?;
        Ok(ImuLogSink {
            out: BufWriter::new(file),
            failing: false,
        })
    }

    pub fn write_sample(&mut self, s: &ImuSample) {
        if self.failing {
            return;
        }
        if let Err(e) = jsonl::write_imu(&mut self.out, s) {
            self.failing = true;
            crate::log_warn!("IMU log write failed, disabled: {e}");
        }
    }
}

impl Drop for ImuLogSink {
    fn drop(&mut self) {
        let _ = self.out.flush();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn frame(yaw: f64, pitch: f64, roll: f64) -> Frame {
        Frame::new_3dof(1.0, [yaw, pitch, roll])
    }

    /// Read an n-f64 native-endian payload from a byte slice.
    fn read_f64s(buf: &[u8]) -> Vec<f64> {
        buf.chunks_exact(8).map(|c| f64::from_ne_bytes(c.try_into().unwrap())).collect()
    }

    #[test]
    fn rate_gate_decimates() {
        let mut g = RateGate::new(4.0); // 250 ms interval
        let t0 = Instant::now();
        assert!(g.due(t0));
        assert!(!g.due(t0 + Duration::from_millis(100)));
        assert!(g.due(t0 + Duration::from_millis(250)));
        assert!(!g.due(t0 + Duration::from_millis(300)));
        assert!(g.due(t0 + Duration::from_millis(500)));
    }

    #[test]
    fn udp_classic_payload_bytes() {
        let sink = UdpSink {
            socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
            dest: "127.0.0.1:9".parse().unwrap(),
            gate: RateGate::new(60.0),
            protocol: Protocol::Classic,
            units: Units::Deg,
            last_warn: None,
        };
        let (buf, len) = sink.payload(&frame(30.0, -10.0, 5.0));
        assert_eq!(len, 48);
        let vals = read_f64s(&buf[..len]);
        assert_eq!(vals.len(), 6);
        assert_eq!(vals[0..3], [0.0, 0.0, 0.0]); // 3-DoF: translation fixed at 0
        assert_eq!(vals[3], 30.0);
        assert_eq!(vals[4], -10.0);
        assert_eq!(vals[5], 5.0);
    }

    #[test]
    fn udp_extended_payload_bytes_and_radians() {
        let sink = UdpSink {
            socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
            dest: "127.0.0.1:9".parse().unwrap(),
            gate: RateGate::new(60.0),
            protocol: Protocol::Extended,
            units: Units::Rad,
            last_warn: None,
        };
        let (buf, len) = sink.payload(&frame(180.0, 0.0, 0.0));
        assert_eq!(len, 80);
        let vals = read_f64s(&buf[..len]);
        assert_eq!(vals.len(), 10);
        assert!((vals[3] - std::f64::consts::PI).abs() < 1e-9);
        assert_eq!(vals[6], 1.0); // pose-valid/confidence default
        assert_eq!(vals[7..10], [0.0, 0.0, 0.0]); // reserved
    }

    #[test]
    fn udp_bind_accepts_hostname_and_ipv6() {
        assert!(UdpSink::bind("localhost", 9, Protocol::Classic, Units::Deg, 60.0).is_ok());
        assert!(UdpSink::bind("::1", 9, Protocol::Classic, Units::Deg, 60.0).is_ok());
        assert!(UdpSink::bind("does-not-exist.invalid", 9, Protocol::Classic, Units::Deg, 60.0)
            .is_err());
    }

    #[test]
    fn udp_position_is_centimeters() {
        let sink = UdpSink {
            socket: UdpSocket::bind("127.0.0.1:0").unwrap(),
            dest: "127.0.0.1:9".parse().unwrap(),
            gate: RateGate::new(60.0),
            protocol: Protocol::Classic,
            units: Units::Deg,
            last_warn: None,
        };
        // 0.25 m forward, 0.1 m right → 25 cm, 10 cm on the wire (Opentrack
        // FreeTrack-derived translation convention).
        let (buf, len) = sink.payload(&Frame {
            t: 0.0,
            ypr_deg: [0.0; 3],
            position: [0.1, 0.0, 0.25],
        });
        let vals = read_f64s(&buf[..len]);
        assert_eq!(vals[0], 10.0);
        assert_eq!(vals[1], 0.0);
        assert_eq!(vals[2], 25.0);
    }

    #[test]
    fn hud_appends_position_when_nonzero() {
        let mut buf = Vec::new();
        let mut hud = HudSink::new(&mut buf, false, 60.0);
        hud.write(&Frame {
            t: 0.0,
            ypr_deg: [1.0, 2.0, 3.0],
            position: [0.12, -0.03, 0.5],
        });
        let s = String::from_utf8(buf).unwrap();
        assert!(s.contains("X   0.12m  Y  -0.03m  Z   0.50m"), "got {s:?}");
        assert!(s.ends_with('\n'));

        // Zero position (3-DoF): P1 output stays unchanged (no X/Y/Z).
        let mut buf = Vec::new();
        let mut hud = HudSink::new(&mut buf, false, 60.0);
        hud.write(&Frame::new_3dof(0.0, [1.0, 2.0, 3.0]));
        let s = String::from_utf8(buf).unwrap();
        assert!(!s.contains("X "), "got {s:?}");
    }

    #[test]
    fn pose_log_includes_position() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!("nt-posepos-{}.jsonl", std::process::id()));
        let mut sink = PoseLogSink::open(&path, Units::Deg).unwrap();
        sink.write(&Frame {
            t: 0.0,
            ypr_deg: [0.0; 3],
            position: [0.1, 0.0, 0.25],
        });
        drop(sink);
        let content = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(content.contains("\"x\": 0.1000"), "got {content}");
        assert!(content.contains("\"z\": 0.2500"));
    }

    #[test]
    fn hud_format_tty_and_pipe() {
        // Piped (non-TTY): newline-terminated, readable line.
        let mut buf = Vec::new();
        let mut hud = HudSink::new(&mut buf, false, 60.0);
        hud.write(&frame(12.3, -4.1, 0.8));
        let s = String::from_utf8(buf).unwrap();
        // Fixed-width columns keep the readout stable on screen (spec §4.3).
        assert!(s.contains("YAW") && s.contains("PITCH") && s.contains("ROLL"));
        assert!(s.contains("12.3") && s.contains("-4.1") && s.contains("0.8"));
        assert!(s.contains('°'));
        assert!(s.ends_with('\n'));

        // TTY: carriage-return redraw in place.
        let mut buf = Vec::new();
        let mut hud = HudSink::new(&mut buf, true, 60.0);
        hud.write(&frame(1.0, 2.0, 3.0));
        let s = String::from_utf8(buf).unwrap();
        assert!(s.ends_with('\r'));
    }

    #[test]
    fn pose_log_uses_wire_units() {
        let dir = std::env::temp_dir();
        let path = dir.join(format!("nt-pose-{}.jsonl", std::process::id()));
        let mut sink = PoseLogSink::open(&path, Units::Rad).unwrap();
        sink.write(&frame(180.0, -90.0, 45.0));
        drop(sink);
        let content = std::fs::read_to_string(&path).unwrap();
        let _ = std::fs::remove_file(&path);
        assert!(content.contains("\"yaw\": 3.141593"), "got {content}");
        assert!(content.contains("\"roll\": 0.785398"));
    }
}
