use std::{
    fs::OpenOptions,
    io::{BufWriter, Write},
    sync::Mutex,
    time::Duration,
};

use humantime::{FormattedDuration, format_duration};
use log::{LevelFilter, Metadata, Record, SetLoggerError};
use vexide::time::user_uptime;

/// A logger for Antaeus
pub struct AntLogger {
    file_writer: Mutex<Option<BufWriter<std::fs::File>>>,
}

impl AntLogger {
    fn new() -> Self {
        let file_writer = OpenOptions::new()
            .create(true)
            .write(true)
            .truncate(true)
            .open("log.txt")
            .ok()
            .map(BufWriter::new);

        Self {
            file_writer: Mutex::new(file_writer),
        }
    }
}

impl log::Log for AntLogger {
    fn enabled(&self, metadata: &Metadata) -> bool { metadata.level() <= log::max_level() }

    fn log(&self, record: &Record) {
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

/// Initialize logger
pub fn init(level: LevelFilter) -> Result<(), SetLoggerError> {
    let logger = LOGGER.get_or_init(|| AntLogger::new());
    log::set_logger(logger).map(|()| log::set_max_level(level))
}

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
    #[ignore = "filesystem access needed (file write)"]
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
