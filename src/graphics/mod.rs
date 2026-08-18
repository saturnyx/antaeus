//! Graphics and Display Related Modules
//!
//! Structure
//! [`embedded_graphics`]: V5 Brain display graphics using `embedded-graphics` and `ratatui`.
//! [`tui`]: TUI rendering support for the V5 Brain display.

// src/graphics/mod.rs
#[cfg(target_os = "vexos")]
mod embedded_graphics;
#[cfg(target_os = "vexos")]
pub use embedded_graphics::DisplayDriver;

#[cfg(not(target_os = "vexos"))]
pub mod sim;
#[cfg(not(target_os = "vexos"))]
pub use sim as embedded_graphics;

pub mod tui;
