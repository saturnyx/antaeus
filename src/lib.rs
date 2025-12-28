//! # Antaeus
//!
//! Antaeus is a versatile robotics framework built on top of [Vexide](https://vexide.dev).
//! It provides a comprehensive set of tools for VEX V5 robot programming, including:
//!
//! - **Drivetrain Control**: Support for differential drivetrains with tank, arcade, and
//!   reverse control schemes.
//! - **Motion Control**: PID-based movement systems, odometry tracking, and path following
//!   using the Candidate-Based Pursuit algorithm.
//! - **Display Graphics**: An [`embedded-graphics`](https://crates.io/crates/embedded-graphics)
//!   compatible driver for the V5 Brain display, with pre-loaded fonts and logo rendering.
//! - **Operator Control**: Utilities for mapping controller buttons to motors and ADI devices.
//! - **Logging**: A file-based logger for debugging and telemetry.
//!
//! ## Quick Start
//!
//! ```ignore
//! use antaeus::drivetrain::Differential;
//! use vexide::prelude::*;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let drivetrain = Differential::new(
//!         [
//!             Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
//!             Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
//!         ],
//!         [
//!             Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
//!             Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
//!         ],
//!     );
//!
//!     let controller = Controller::new(ControllerId::Primary);
//!     loop {
//!         drivetrain.tank(&controller);
//!     }
//! }
//! ```
//!
//! ## Modules
//!
//! - [`drivetrain`]: Differential drivetrain control with multiple drive modes.
//! - [`motion`]: Autonomous motion algorithms including PID, odometry, and pursuit.
//! - [`display`]: V5 Brain display graphics using `embedded-graphics`.
//! - [`opcontrol`]: Controller input mapping to motors and pneumatics.
//! - [`fs`]: Filesystem utilities including logging.

/// Differential drivetrain control module.
///
/// Provides the [`Differential`](drivetrain::Differential) struct for controlling
/// robots with left and right motor groups. Supports multiple control schemes:
///
/// - **Tank**: Each joystick controls one side of the robot.
/// - **Arcade**: Left stick for forward/backward, right stick for turning.
/// - **Reverse**: Inverted controls for driving in reverse.
pub mod drivetrain;

/// Filesystem utilities module.
///
/// Contains logging functionality for recording robot telemetry and debug
/// information to files on the V5 Brain's SD card.
pub mod fs;

/// Autonomous motion control module.
///
/// Provides algorithms for precise robot movement during autonomous periods:
///
/// - **PID Control**: Proportional-Integral-Derivative controllers for linear
///   and rotational movement.
/// - **Odometry**: Position tracking using tracking wheels and an inertial sensor.
/// - **Pursuit**: Path following using the Candidate-Based Pursuit algorithm,
///   a robust variant of pure pursuit.
pub mod motion;

/// Operator control utilities module.
///
/// Simplifies controller input handling during driver control periods.
/// Maps controller buttons to motor voltages and ADI digital outputs
/// with support for toggle, momentary, and dual-button controls.
pub mod opcontrol;

/// V5 Brain display graphics module.
///
/// Provides an [`embedded-graphics`](https://crates.io/crates/embedded-graphics)
/// compatible [`DrawTarget`](embedded_graphics_core::draw_target::DrawTarget)
/// for rendering graphics on the V5 Brain display. Includes:
///
/// - Pre-loaded TTF fonts for text rendering.
/// - Antaeus logo and badge display utilities.
pub mod display;
