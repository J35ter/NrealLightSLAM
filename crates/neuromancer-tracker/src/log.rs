//! Minimal leveled logging to stderr (no external deps).
//!
//! Level semantics: a message of severity `l` is printed when
//! `l <= current level` in the order `error < warn < info < debug`.
//! Default level is [`Level::Error`] — only errors show unless the user
//! raises verbosity with `--log-level`.

use std::sync::atomic::{AtomicU8, Ordering};

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[repr(u8)]
pub enum Level {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
}

impl Level {
    /// Parse a CLI value (`error|warn|warning|info|debug`).
    pub fn parse(s: &str) -> Option<Level> {
        match s.trim().to_ascii_lowercase().as_str() {
            "error" => Some(Level::Error),
            "warn" | "warning" => Some(Level::Warn),
            "info" => Some(Level::Info),
            "debug" => Some(Level::Debug),
            _ => None,
        }
    }
}

static LEVEL: AtomicU8 = AtomicU8::new(Level::Error as u8);

/// Set the global log level (once, from the CLI config).
pub fn set_level(level: Level) {
    LEVEL.store(level as u8, Ordering::Relaxed);
}

/// Is `level` currently enabled (visible)?
pub fn enabled(level: Level) -> bool {
    (level as u8) <= LEVEL.load(Ordering::Relaxed)
}

/// Print a message to stderr when the given level is enabled.
/// Usage: `log!(Level::Warn, "udp send to {} failed: {e}", dest)`.
pub fn log(level: Level, args: std::fmt::Arguments<'_>) {
    if enabled(level) {
        eprintln!("{args}");
    }
}

#[macro_export]
macro_rules! log_error {
    ($($arg:tt)*) => {
        $crate::log::log($crate::log::Level::Error, format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_warn {
    ($($arg:tt)*) => {
        $crate::log::log($crate::log::Level::Warn, format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        $crate::log::log($crate::log::Level::Info, format_args!($($arg)*))
    };
}

#[macro_export]
macro_rules! log_debug {
    ($($arg:tt)*) => {
        $crate::log::log($crate::log::Level::Debug, format_args!($($arg)*))
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn level_parse() {
        assert_eq!(Level::parse("error"), Some(Level::Error));
        assert_eq!(Level::parse("warn"), Some(Level::Warn));
        assert_eq!(Level::parse("warning"), Some(Level::Warn));
        assert_eq!(Level::parse("info"), Some(Level::Info));
        assert_eq!(Level::parse("debug"), Some(Level::Debug));
        assert_eq!(Level::parse("bogus"), None);
    }

    #[test]
    fn default_is_error_only() {
        set_level(Level::Error);
        assert!(enabled(Level::Error));
        assert!(!enabled(Level::Warn));
        assert!(!enabled(Level::Info));
        assert!(!enabled(Level::Debug));
        set_level(Level::Debug);
        assert!(enabled(Level::Error) && enabled(Level::Warn) && enabled(Level::Info) && enabled(Level::Debug));
    }
}
