//! Raw IMU sample type and the JSONL reader/writer used by `--log-imu`,
//! `--log-pose` and `--replay`. Hand-rolled, no serde — minimal deps by design.

use std::fmt::Write as _;
use std::io::{self, Write};

/// One IMU sample: accelerometer (m/s²) + gyroscope (rad/s), `t` = monotonic
/// seconds since stream start.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub t: f64,
    pub ax: f64,
    pub ay: f64,
    pub az: f64,
    pub gx: f64,
    pub gy: f64,
    pub gz: f64,
}

/// Write one IMU sample as a JSONL line (spec §4.4 format).
pub fn write_imu(out: &mut impl Write, s: &ImuSample) -> io::Result<()> {
    let mut line = String::with_capacity(160);
    write!(
        line,
        "{{\"t\": {:.6}, \"ax\": {:.6}, \"ay\": {:.6}, \"az\": {:.6}, \"gx\": {:.6}, \"gy\": {:.6}, \"gz\": {:.6}}}",
        s.t, s.ax, s.ay, s.az, s.gx, s.gy, s.gz
    )
    .expect("formatting to String cannot fail");
    writeln!(out, "{line}")
}

/// Write one filtered pose as a JSONL line. `ypr` uses the selected wire
/// units (matches `--units`).
pub fn write_pose(out: &mut impl Write, t: f64, ypr: [f64; 3]) -> io::Result<()> {
    let mut line = String::with_capacity(96);
    write!(
        line,
        "{{\"t\": {:.6}, \"yaw\": {:.6}, \"pitch\": {:.6}, \"roll\": {:.6}}}",
        t, ypr[0], ypr[1], ypr[2]
    )
    .expect("formatting to String cannot fail");
    writeln!(out, "{line}")
}

/// Parse one IMU JSONL line.
///
/// - `Ok(None)`: blank / whitespace-only line (skip).
/// - `Err(msg)`: malformed line (missing or unparseable required field).
/// - `Ok(Some(sample))`: valid sample. Unknown extra keys are ignored
///   (forward compatibility).
pub fn parse_imu_line(line: &str) -> Result<Option<ImuSample>, String> {
    let line = line.trim();
    if line.is_empty() {
        return Ok(None);
    }
    let inner = line
        .strip_prefix('{')
        .and_then(|l| l.strip_suffix('}'))
        .ok_or_else(|| "line is not a JSON object".to_string())?;

    let mut vals: [Option<f64>; 7] = [None; 7]; // t, ax, ay, az, gx, gy, gz
    let keys: [&str; 7] = ["t", "ax", "ay", "az", "gx", "gy", "gz"];
    for part in inner.split(',') {
        let (k, v) = part
            .split_once(':')
            .ok_or_else(|| format!("malformed key/value pair: {part:?}"))?;
        let key = k.trim().trim_matches('"');
        let value: f64 = v
            .trim()
            .parse()
            .map_err(|_| format!("invalid number for {key:?}: {v:?}"))?;
        if let Some(idx) = keys.iter().position(|&k| k == key) {
            vals[idx] = Some(value);
        }
    }
    let all: Vec<f64> = vals
        .into_iter()
        .enumerate()
        .map(|(i, v)| v.ok_or_else(|| format!("missing field {:?}", keys[i])))
        .collect::<Result<_, _>>()?;
    Ok(Some(ImuSample {
        t: all[0],
        ax: all[1],
        ay: all[2],
        az: all[3],
        gx: all[4],
        gy: all[5],
        gz: all[6],
    }))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn imu_roundtrip() {
        let s = ImuSample {
            t: 1.5,
            ax: 0.02,
            ay: -9.81,
            az: 0.05,
            gx: 0.001,
            gy: -0.002,
            gz: 0.0005,
        };
        let mut buf = Vec::new();
        write_imu(&mut buf, &s).unwrap();
        let line = String::from_utf8(buf).unwrap();
        assert!(line.starts_with("{\"t\":"));
        assert!(line.ends_with("}\n"));
        let parsed = parse_imu_line(&line).unwrap().unwrap();
        assert_eq!(parsed, s);
    }

    #[test]
    fn parse_skips_blank_and_tolerates_extra_keys() {
        assert!(parse_imu_line("").unwrap().is_none());
        assert!(parse_imu_line("  \n").unwrap().is_none());
        let line = r#"{"t": 1.0, "ax": 0.1, "ay": 0.2, "az": 0.3, "gx": 0.4, "gy": 0.5, "gz": 0.6, "extra": 99}"#;
        let s = parse_imu_line(line).unwrap().unwrap();
        assert_eq!(s.gx, 0.4);
    }

    #[test]
    fn parse_rejects_malformed() {
        assert!(parse_imu_line("not json").is_err());
        assert!(parse_imu_line(r#"{"t": 1.0, "ax": nope}"#).is_err());
        assert!(parse_imu_line(r#"{"t": 1.0}"#).is_err()); // missing fields
    }

    #[test]
    fn pose_roundtrip() {
        let mut buf = Vec::new();
        write_pose(&mut buf, 12.5, [0.21, -0.07, 0.01]).unwrap();
        let line = String::from_utf8(buf).unwrap();
        assert!(line.contains("\"yaw\": 0.210000"));
        assert!(line.contains("\"roll\": 0.010000"));
    }
}
