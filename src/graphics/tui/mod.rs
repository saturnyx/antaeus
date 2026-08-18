//! Use Ratatui to create Terminal User Interfaces on the VEX V5 Brain's Display
//!
//! # Example
//! ```
#![doc = include_str!("../../../examples/ratatui.rs")]
//! ```
// src/graphics/tui.rs — untouched, works for both backends unmodified
use mousefood::{EmbeddedBackend, EmbeddedBackendConfig, prelude::Rgb888};
use ratatui::Terminal;

use super::embedded_graphics::DisplayDriver;

/// Terminal Display Backend Trait
pub trait TerminalDisplay<'a> {
    /// Create a backend using a `DisplayDriver`
    fn from_driver(driver: &'a mut DisplayDriver) -> Self;
}

impl<'a> TerminalDisplay<'a> for Terminal<EmbeddedBackend<'a, DisplayDriver, Rgb888>> {
    fn from_driver(driver: &'a mut DisplayDriver) -> Self {
        let config = EmbeddedBackendConfig::default();
        let backend = EmbeddedBackend::new(driver, config);
        Terminal::new(backend).unwrap()
    }
}
