//! Hand-rolled CLI parser (spec §2.6: "may keep hand-rolled parser for zero
//! deps"). Zero-dependency by design; validation errors exit with code 2.

use std::path::PathBuf;
use std::str::FromStr;

/// Opentrack UDP protocol variant (OQ1 — resolved: both, user switch).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Protocol {
    /// 48-byte 6×f64 `[TX,TY,TZ,Yaw,Pitch,Roll]` — stock Opentrack.
    Classic,
    /// 80-byte 10×f64 — 6 pose doubles + 4 reserved doubles.
    Extended,
}

/// Wire units for UDP output and pose log (OQ1a — resolved: degrees default).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Units {
    Deg,
    Rad,
}

/// Effective tracker configuration (spec §3.4 + resolved open questions).
#[derive(Debug, Clone)]
pub struct Config {
    pub host: String,
    pub port: u16,
    pub no_udp: bool,
    pub hud: bool,
    pub hud_rate: f64,
    pub kp: f64,
    pub ki: f64,
    pub invert_yaw: bool,
    pub invert_pitch: bool,
    pub invert_roll: bool,
    pub sensitivity: f64,
    pub log_imu: Option<PathBuf>,
    pub log_pose: Option<PathBuf>,
    pub protocol: Protocol,
    pub udp_rate: f64,
    pub units: Units,
    /// Dev/testing aid: read IMU samples from a JSONL file instead of USB
    /// (deviation from the USB-exclusive input path, documented in README).
    pub replay: Option<PathBuf>,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            host: "127.0.0.1".to_string(),
            port: 4242,
            no_udp: false,
            hud: false,
            hud_rate: 2.0,
            kp: 1.0,
            ki: 0.005,
            invert_yaw: false,
            invert_pitch: false,
            invert_roll: false,
            sensitivity: 1.0,
            log_imu: None,
            log_pose: None,
            protocol: Protocol::Classic,
            udp_rate: 60.0,
            units: Units::Deg,
            replay: None,
        }
    }
}

/// Result of a successful parse.
pub enum ParseOutcome {
    Run(Config),
    Help,
    Version,
}

/// Arg iterator with `--flag=value` support: an inline value is parked and
/// consumed by the next `take()`.
struct ArgIter {
    inner: std::vec::IntoIter<String>,
    pending: Option<String>,
}

impl ArgIter {
    fn new(args: Vec<String>) -> Self {
        ArgIter {
            inner: args.into_iter(),
            pending: None,
        }
    }

    fn next_flag(&mut self) -> Option<String> {
        self.inner.next()
    }

    fn take(&mut self, name: &str) -> Result<String, String> {
        if let Some(v) = self.pending.take() {
            return Ok(v);
        }
        let v = self
            .inner
            .next()
            .ok_or_else(|| format!("option {name} requires a value"))?;
        if v.starts_with('-') {
            // Don't swallow a following flag as this option's value.
            return Err(format!("option {name} requires a value (got {v:?})"));
        }
        Ok(v)
    }
}

fn parse_f64(raw: &str, name: &str) -> Result<f64, String> {
    let v: f64 = f64::from_str(raw.trim())
        .map_err(|_| format!("invalid value for {name}: {raw:?}"))?;
    if !v.is_finite() {
        return Err(format!("invalid value for {name}: {raw:?} (must be finite)"));
    }
    Ok(v)
}

fn parse_nonneg(raw: &str, name: &str) -> Result<f64, String> {
    let v = parse_f64(raw, name)?;
    if v < 0.0 {
        return Err(format!("invalid value for {name}: {raw:?} (must be ≥ 0)"));
    }
    Ok(v)
}

fn parse_pos(raw: &str, name: &str) -> Result<f64, String> {
    let v = parse_f64(raw, name)?;
    if v <= 0.0 {
        return Err(format!("invalid value for {name}: {raw:?} (must be > 0)"));
    }
    Ok(v)
}

/// Parse command-line arguments (excluding argv[0]).
pub fn parse(args: Vec<String>) -> Result<ParseOutcome, String> {
    let mut cfg = Config::default();
    let mut it = ArgIter::new(args);

    while let Some(arg) = it.next_flag() {
        let (flag, inline) = match arg.split_once('=') {
            Some((f, v)) => (f.to_string(), Some(v.to_string())),
            None => (arg, None),
        };
        if let Some(v) = inline {
            it.pending = Some(v);
        }

        // Value-less flags: reject any inline value (`--hud=1`) so it can't
        // be stolen by the next value-taking flag (`--hud=1 --host x`).
        let flagless = matches!(
            flag.as_str(),
            "--no-udp" | "--hud" | "--invert-yaw" | "--invert-pitch" | "--invert-roll"
        );
        if flagless {
            if it.pending.take().is_some() {
                return Err(format!("option {flag} takes no value"));
            }
            match flag.as_str() {
                "--no-udp" => cfg.no_udp = true,
                "--hud" => cfg.hud = true,
                "--invert-yaw" => cfg.invert_yaw = true,
                "--invert-pitch" => cfg.invert_pitch = true,
                "--invert-roll" => cfg.invert_roll = true,
                _ => unreachable!("flagless set is exhaustive"),
            }
            continue;
        }

        match flag.as_str() {
            "-h" | "--help" => return Ok(ParseOutcome::Help),
            "-V" | "--version" => return Ok(ParseOutcome::Version),
            "--host" => cfg.host = it.take("--host")?,
            "--port" => {
                let raw = it.take("--port")?;
                cfg.port = u16::from_str(raw.trim())
                    .map_err(|_| format!("invalid value for --port: {raw:?} (1-65535)"))?;
                if cfg.port == 0 {
                    return Err("invalid value for --port: must be 1-65535".to_string());
                }
            }
            "--no-udp" => { /* handled by the flagless branch above */ }
            "--hud" => { /* handled by the flagless branch above */ }
            "--hud-rate" => {
                let raw = it.take("--hud-rate")?;
                cfg.hud_rate = parse_pos(&raw, "--hud-rate")?;
            }
            "--kp" => {
                let raw = it.take("--kp")?;
                cfg.kp = parse_nonneg(&raw, "--kp")?;
            }
            "--ki" => {
                let raw = it.take("--ki")?;
                cfg.ki = parse_nonneg(&raw, "--ki")?;
            }
            "--invert-yaw" => { /* handled by the flagless branch above */ }
            "--invert-pitch" => { /* handled by the flagless branch above */ }
            "--invert-roll" => { /* handled by the flagless branch above */ }
            "--sensitivity" => {
                let raw = it.take("--sensitivity")?;
                cfg.sensitivity = parse_nonneg(&raw, "--sensitivity")?;
            }
            "--log-imu" => cfg.log_imu = Some(PathBuf::from(it.take("--log-imu")?)),
            "--log-pose" => cfg.log_pose = Some(PathBuf::from(it.take("--log-pose")?)),
            "--protocol" => {
                let raw = it.take("--protocol")?;
                cfg.protocol = match raw.trim().to_ascii_lowercase().as_str() {
                    "classic" | "6x64" | "6xf64" => Protocol::Classic,
                    "extended" | "10d" | "10-double" | "10xf64" => Protocol::Extended,
                    other => {
                        return Err(format!(
                            "invalid value for --protocol: {other:?} (classic|extended)"
                        ))
                    }
                };
            }
            "--udp-rate" => {
                let raw = it.take("--udp-rate")?;
                cfg.udp_rate = parse_pos(&raw, "--udp-rate")?;
            }
            "--units" => {
                let raw = it.take("--units")?;
                cfg.units = match raw.trim().to_ascii_lowercase().as_str() {
                    "deg" | "degree" | "degrees" => Units::Deg,
                    "rad" | "radian" | "radians" => Units::Rad,
                    other => {
                        return Err(format!("invalid value for --units: {other:?} (deg|rad)"))
                    }
                };
            }
            "--replay" => cfg.replay = Some(PathBuf::from(it.take("--replay")?)),
            other => return Err(format!("unknown option: {other}")),
        }
    }
    Ok(ParseOutcome::Run(cfg))
}

/// Full usage/help text (spec §3.4 + resolved additions).
pub fn usage() -> &'static str {
    concat!(
        "neuromancer-tracker — Nreal Light head tracker (Phase 1: 3-DoF)\n",
        "\n",
        "USAGE:\n",
        "    neuromancer-tracker [OPTIONS]\n",
        "\n",
        "OPTIONS:\n",
        "    --host <IP>          Opentrack host (default 127.0.0.1)\n",
        "    --port <PORT>        Opentrack UDP port (default 4242)\n",
        "    --no-udp             Disable UDP output (HUD-only mode)\n",
        "    --protocol <P>       UDP protocol variant: classic|extended (default classic)\n",
        "    --udp-rate <HZ>      UDP output rate (default 60)\n",
        "    --units <U>          Wire units for UDP + pose log: deg|rad (default deg)\n",
        "    --hud                Enable 2 Hz on-screen orientation readout\n",
        "    --hud-rate <HZ>      HUD update rate (default 2)\n",
        "    --kp <FLOAT>         Mahony proportional gain (default 1.0)\n",
        "    --ki <FLOAT>         Mahony integral gain (default 0.005)\n",
        "    --invert-yaw         Invert yaw axis\n",
        "    --invert-pitch       Invert pitch axis\n",
        "    --invert-roll        Invert roll axis\n",
        "    --sensitivity <F>    Output sensitivity multiplier (default 1.0)\n",
        "    --log-imu <PATH>     Append raw IMU samples to file (JSONL)\n",
        "    --log-pose <PATH>    Append filtered pose to file (JSONL)\n",
        "    --replay <PATH>      Dev/testing: read IMU from a JSONL file instead of USB\n",
        "    --version            Print version and exit\n",
        "    --help               Print help and exit\n",
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(args: &[&str]) -> Result<ParseOutcome, String> {
        parse(args.iter().map(|s| s.to_string()).collect())
    }

    #[test]
    fn defaults() {
        let ParseOutcome::Run(c) = run(&[]).unwrap() else {
            panic!("expected config");
        };
        assert_eq!(c.host, "127.0.0.1");
        assert_eq!(c.port, 4242);
        assert!(!c.no_udp);
        assert!(!c.hud);
        assert_eq!(c.hud_rate, 2.0);
        assert_eq!(c.kp, 1.0);
        assert_eq!(c.ki, 0.005);
        assert_eq!(c.sensitivity, 1.0);
        assert_eq!(c.protocol, Protocol::Classic);
        assert_eq!(c.udp_rate, 60.0);
        assert_eq!(c.units, Units::Deg);
        assert!(c.replay.is_none());
    }

    #[test]
    fn full_flag_set() {
        let ParseOutcome::Run(c) = run(&[
            "--host", "192.168.0.10", "--port", "5555", "--protocol=extended", "--udp-rate",
            "90", "--units", "rad", "--hud", "--hud-rate", "4", "--kp", "0.5", "--ki", "0.01",
            "--invert-yaw", "--invert-roll", "--sensitivity", "1.5", "--log-imu", "/tmp/i.jsonl",
            "--log-pose", "/tmp/p.jsonl", "--replay", "/tmp/r.jsonl", "--no-udp",
        ])
        .unwrap()
        else {
            panic!("expected config");
        };
        assert_eq!(c.host, "192.168.0.10");
        assert_eq!(c.port, 5555);
        assert_eq!(c.protocol, Protocol::Extended);
        assert_eq!(c.udp_rate, 90.0);
        assert_eq!(c.units, Units::Rad);
        assert!(c.hud);
        assert_eq!(c.hud_rate, 4.0);
        assert_eq!(c.kp, 0.5);
        assert_eq!(c.ki, 0.01);
        assert!(c.invert_yaw);
        assert!(!c.invert_pitch);
        assert!(c.invert_roll);
        assert_eq!(c.sensitivity, 1.5);
        assert_eq!(c.log_imu.as_deref(), Some(std::path::Path::new("/tmp/i.jsonl")));
        assert_eq!(c.log_pose.as_deref(), Some(std::path::Path::new("/tmp/p.jsonl")));
        assert!(c.replay.is_some());
        assert!(c.no_udp);
    }

    #[test]
    fn help_and_version() {
        assert!(matches!(run(&["--help"]).unwrap(), ParseOutcome::Help));
        assert!(matches!(run(&["-h"]).unwrap(), ParseOutcome::Help));
        assert!(matches!(run(&["--version"]).unwrap(), ParseOutcome::Version));
    }

    #[test]
    fn rejects_inline_value_on_value_less_flags() {
        for f in ["--hud", "--no-udp", "--invert-yaw", "--invert-pitch", "--invert-roll"] {
            assert!(run(&[&format!("{f}=1")]).is_err(), "{f}=1 must error");
        }
        // The parked inline value must never be stolen by the next flag.
        assert!(run(&["--hud=1", "--host", "x"]).is_err());
    }

    #[test]
    fn rejects_flag_as_value() {
        assert!(run(&["--host", "--hud"]).is_err());
        assert!(run(&["--port", "--no-udp"]).is_err());
        assert!(run(&["--kp", "--invert-roll"]).is_err());
    }

    #[test]
    fn rejects_unknown_flag() {
        assert!(run(&["--bogus"]).is_err());
    }

    #[test]
    fn rejects_bad_values() {
        assert!(run(&["--port", "0"]).is_err());
        assert!(run(&["--port", "70000"]).is_err());
        assert!(run(&["--port", "abc"]).is_err());
        assert!(run(&["--udp-rate", "0"]).is_err());
        assert!(run(&["--udp-rate", "-5"]).is_err());
        assert!(run(&["--hud-rate", "0"]).is_err());
        assert!(run(&["--kp", "-1"]).is_err());
        assert!(run(&["--ki", "nan"]).is_err());
        assert!(run(&["--protocol", "bogus"]).is_err());
        assert!(run(&["--units", "bogus"]).is_err());
        assert!(run(&["--host"]).is_err()); // missing value
    }

    #[test]
    fn protocol_aliases() {
        for alias in ["classic", "6x64", "6xf64", "CLASSIC"] {
            let ParseOutcome::Run(c) = run(&["--protocol", alias]).unwrap() else {
                panic!();
            };
            assert_eq!(c.protocol, Protocol::Classic);
        }
        for alias in ["extended", "10d", "10-double", "10xf64"] {
            let ParseOutcome::Run(c) = run(&["--protocol", alias]).unwrap() else {
                panic!();
            };
            assert_eq!(c.protocol, Protocol::Extended);
        }
    }
}
