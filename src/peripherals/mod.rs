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
//! use antaeus::opcontrol::controller::{ControllerControl, ControllerButton};
//!
//! let controller = Controller::new(ControllerId::Primary);
//! let control = ControllerControl::new(&controller, ControllerButton::ButtonA);
//!
//! // Button B toggles a piston
//! control.button_to_adi_toggle(
//!     ControllerButton::ButtonB,
//!     heapless::Vec::from_array([&mut piston]),
//!     false,
//! );
//!
//! // L1 runs intake forward, L2 runs it backward
//! control.dual_button_to_motors(
//!     ControllerButton::ButtonL1,
//!     ControllerButton::ButtonL2,
//!     heapless::Vec::from_array([&mut intake]),
//!     12.0, -12.0, 0.0, false,
//! );
//! ```

/// Controller input mapping utilities.
///
/// Provides [`ControllerControl`](controller::ControllerControl) for
/// mapping buttons to motors and ADI devices.
pub mod controller;

/// Differential drivetrain control module.
///
/// Provides the [`Differential`](drivetrain::Differential) struct for controlling
/// robots with left and right motor groups. Supports multiple control schemes:
///
/// - **Tank**: Each joystick controls one side of the robot.
/// - **Arcade**: Left stick for forward/backward, right stick for turning.
/// - **Reverse**: Inverted controls for driving in reverse.
pub mod drivetrain;

pub mod motorgroup;
