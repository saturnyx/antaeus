use std::{
    fs::OpenOptions,
    io::{BufWriter, Write},
    sync::Mutex,
};

use log::{Level, LevelFilter, Metadata, Record, SetLoggerError};

/// A logger for Antaeus
pub struct AntLogger {
    file_writer: Mutex<Option<BufWriter<std::fs::File>>>,
}

impl AntLogger {
    fn new() -> Self {
        let file_writer = OpenOptions::new()
            .create(true)
            .append(true)
            .open("log.txt")
            .ok()
            .map(BufWriter::new);

        Self {
            file_writer: Mutex::new(file_writer),
        }
    }
}

impl log::Log for AntLogger {
    fn enabled(&self, metadata: &Metadata) -> bool { metadata.level() <= Level::Info }

    fn log(&self, record: &Record) {
        if self.enabled(record.metadata()) {
            let log_line =
                format!("[{}] {} - {}\n", record.level(), record.target(), record.args());

            // Print to console
            print!("{}", log_line);

            // Write to file (don't use warn! to avoid recursion)
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
