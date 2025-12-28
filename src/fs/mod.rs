//! Filesystem utilities for the V5 Brain.
//!
//! This module provides utilities for interacting with the V5 Brain's
//! filesystem, including logging functionality for recording telemetry
//! and debug information.
//!
//! # Logging
//!
//! The `logger` submodule provides a file-based logger that writes to
//! `log.txt` on the SD card. This is useful for debugging issues that
//! only occur on the robot.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::fs::logger;
//! use log::{info, LevelFilter};
//!
//! // Initialize the logger at program start
//! logger::init(LevelFilter::Debug).expect("Failed to initialize logger");
//!
//! // Now you can use standard logging macros
//! info!("Robot initialized successfully");
//! ```

/// File-based logging for the V5 Brain.
///
/// Provides a logger implementation that writes to both the console
/// and a file on the SD card.
pub mod logger;
