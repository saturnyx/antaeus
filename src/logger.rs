//! File-based logger implementation for the V5 Brain.
//!
//! This module implements the [`log`] crate's logging facade, writing log
//! messages to both the console (terminal/debug output) and a file on the
//! V5 Brain's SD card.
//!
//! # Usage
//!
//! Initialize the logger once at the start of your program:
//!
//! ```ignore
//! use antaeus::fs::logger;
//! use log::{info, warn, error, LevelFilter};
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     logger::init(LevelFilter::Debug).expect("Logger init failed");
//!
//!     info!("Program started");
//!     warn!("This is a warning");
//!     error!("This is an error");
//! }
//! ```
//!
//! # Log Output
//!
//! On VexOS, logs are written to `log.txt` in the root of the SD card. On
//! other targets, logs are written to `test-artifacts/log.txt`. Each log entry
//! includes:
//! - Log level (TRACE, DEBUG, INFO, WARN, ERROR)
//! - Timestamp (time since program start)
//! - Target (module path)
//! - Message
//!
//! Example output:
//! ```text
//! INFO [2m 5s 123ms] antaeus::motion::feedback_control::legacy_pid::linear_pid - PID Control Loop Started
//! WARN [2m 5s 456ms] antaeus::peripherals::drivetrain - Controller State Error: Disconnected
//! ```

use std::{
    fs::{OpenOptions, create_dir_all},
    io::{BufWriter, Write},
    path::PathBuf,
    sync::Mutex,
    time::Duration,
};

use humantime::{FormattedDuration, format_duration};
use log::{LevelFilter, Metadata, Record, SetLoggerError};
use vexide::time::user_uptime;

/// A dual-output logger for the Antaeus framework.
///
/// Writes log messages to both the console and a file.
/// The file is created/truncated when the logger is initialized.
#[derive(Debug)]
pub struct AntLogger {
    /// Buffered file writer for log output.
    ///
    /// Wrapped in a mutex for thread-safe access. May be `None` if
    /// the file could not be opened (e.g., no SD card present).
    file_writer: Mutex<Option<BufWriter<std::fs::File>>>,
}

impl AntLogger {
    fn new() -> Self {
        let file_writer = Self::log_path()
            .and_then(|path| {
                if let Some(parent) = path.parent() {
                    if !parent.as_os_str().is_empty() {
                        create_dir_all(parent).ok()?;
                    }
                }

                OpenOptions::new()
                    .create(true)
                    .write(true)
                    .truncate(true)
                    .open(path)
                    .ok()
            })
            .map(BufWriter::new);

        Self {
            file_writer: Mutex::new(file_writer),
        }
    }

    fn log_path() -> Option<PathBuf> {
        if cfg!(target_os = "vexos") {
            Some(PathBuf::from("log.txt"))
        } else {
            Some(PathBuf::from("test-artifacts/log_test.txt"))
        }
    }
}

impl log::Log for AntLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool { metadata.level() <= log::max_level() }

    fn log(&self, record: &Record<'_>) {
        if self.enabled(record.metadata()) {
            let log_line = format!(
                "{} [{}] {} - {}\n",
                record.level(),
                get_time(),
                record.target(),
                record.args()
            );

            // Print to console
            print!("{}", log_line);

            if let Ok(mut writer_guard) = self.file_writer.lock() {
                if let Some(ref mut writer) = *writer_guard {
                    let _ = writer.write_all(log_line.as_bytes());
                }
            }
        }
    }

    fn flush(&self) {
        if let Ok(mut writer_guard) = self.file_writer.lock() {
            if let Some(ref mut writer) = *writer_guard {
                let _ = writer.flush();
            }
        }
    }
}

static LOGGER: std::sync::OnceLock<AntLogger> = std::sync::OnceLock::new();

/// Initializes the Antaeus logger.
///
/// This function must be called once before any logging macros are used.
/// It sets up the global logger to write to both the console and the target-
/// specific log file path.
///
/// # Arguments
///
/// * `level` - The minimum log level to record. Messages below this level
///   will be ignored. Use [`LevelFilter::Trace`] for maximum verbosity or
///   [`LevelFilter::Error`] for critical messages only.
///
/// # Errors
///
/// Returns [`SetLoggerError`] if a logger has already been set.
///
/// # Example
///
/// ```ignore
/// use antaeus::fs::logger;
/// use log::LevelFilter;
///
/// // Initialize with debug level logging
/// logger::init(LevelFilter::Debug)?;
/// ```
pub fn init(level: LevelFilter) -> Result<(), SetLoggerError> {
    let logger = LOGGER.get_or_init(|| AntLogger::new());
    log::set_logger(logger).map(|()| log::set_max_level(level))
}

/// Returns the formatted duration since the user program started.
///
/// On VexOS, this returns the actual uptime. On other platforms (for testing),
/// returns a placeholder value.
fn get_time() -> FormattedDuration {
    let dur;
    if !cfg!(target_os = "vexos") {
        dur = Duration::from_millis(123432);
    } else {
        dur = user_uptime();
    }
    format_duration(dur)
}

#[cfg(test)]
mod tests {
    use log::{LevelFilter, debug, error, info, trace, warn};

    #[test]
    fn log_full_test() {
        super::init(LevelFilter::Trace).expect("Failed to initialize logger");

        trace!("This is a trace message");
        debug!("This is a debug message");
        info!("This is an info message");
        warn!("This is a warning message");
        error!("This is an error message");

        log::logger().flush();

        assert!(
            log::logger().enabled(
                &log::Metadata::builder()
                    .level(log::Level::Error)
                    .target("test")
                    .build()
            )
        );
    }
}
