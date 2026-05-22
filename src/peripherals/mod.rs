//! Operator control utilities for driver control periods.
//!
//! This module simplifies mapping controller inputs to robot actions
//! during the driver-controlled portion of a match.
//!
//! # Features
//!
//! - **Button-to-ADI mapping**: Control pneumatics with button presses.
//! - **Button-to-Motor mapping**: Control mechanisms with button holds.
//! - **Control button modifiers**: Combine buttons for extended controls.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::peripherals::controller::{ControllerControl, ControllerButton};
//!
//! let controller = Controller::new(ControllerId::Primary);
//! let control = ControllerControl::new(&controller, ControllerButton::ButtonA);
//!
//! // Button B toggles a piston
//! control.button_to_adi_toggle(
//!     ControllerButton::ButtonB,
//!     vec![&mut piston],
//!     false,
//! );
//!
//! // L1 runs intake forward, L2 runs it backward
//! control.dual_button_to_motors(
//!     ControllerButton::ButtonL1,
//!     ControllerButton::ButtonL2,
//!     vec![&mut intake],
//!     12.0, -12.0, 0.0, false,
//! );
//! ```

pub mod drivetrain;

pub mod motorgroup;

pub mod range_sensor;

pub mod mapper;

// LEGACY ---------------------------------------------------------------------+

#[cfg(feature = "legacy")]
pub mod controller;
