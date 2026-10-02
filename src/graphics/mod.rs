//! Graphics and Display Related Modules
//!
//! Structure
//! [`embedded_graphics`]: V5 Brain display graphics using `embedded-graphics` and `ratatui`.
//! [`tui`]: TUI rendering support for the V5 Brain display.

// Brain Display Sim
#[cfg(target_os = "vexos")]
mod embedded_graphics;
#[cfg(target_os = "vexos")]
pub use embedded_graphics::DisplayDriver;

#[cfg(not(target_os = "vexos"))]
pub mod sim;
#[cfg(not(target_os = "vexos"))]
pub use sim as embedded_graphics;
// LED Sim
#[cfg(target_os = "vexos")]
pub use vexide::adi::addrled;
#[cfg(not(target_os = "vexos"))]
pub mod addrled;
#[cfg(not(target_os = "vexos"))]
pub mod tui;
