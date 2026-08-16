//! Use Ratatui to create Terminal User Interfaces on the VEX V5 Brain's Display
//!
//! # Example
//! ```
#![doc = include_str!("../../../examples/ratatui.rs")]
//! ```
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig, prelude::Rgb888};
use ratatui::Terminal;

use super::embedded_graphics::DisplayDriver;

/// VEX V5 Terminal Driver
pub trait TerminalDisplay<'a> {
    /// Create a new `Terminal` from a `DisplayDriver`
    fn from_driver(driver: &'a mut DisplayDriver) -> Self;
}

impl<'a> TerminalDisplay<'a> for Terminal<EmbeddedBackend<'a, DisplayDriver, Rgb888>> {
    fn from_driver(driver: &'a mut DisplayDriver) -> Self {
        let config = EmbeddedBackendConfig::default();
        let backend = EmbeddedBackend::new(driver, config);

        Terminal::new(backend).unwrap()
    }
}
