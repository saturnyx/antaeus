//! Drivetrain Control
//!
//! This module defines traits and enums related to drivetrain types
// TODO: Add Docs

use std::cell::BorrowMutError;

use snafu::Snafu;
use vexide::{
    controller::ControllerError,
    math::Angle,
    prelude::Controller,
    smart::{PortError, motor::BrakeMode},
};

use crate::utils::error::Report;

pub mod differential;

/// Any form of Drivable Drivetrain that can be controlled using a controller
pub trait Drivable {
    /// Controls a tank-style drivetrain using the input from a controller.
    ///
    /// In tank drive mode, each joystick directly controls one side of the
    /// drivetrain. The left stick Y-axis controls the left motors, and the
    /// right stick Y-axis controls the right motors.
    fn tank(&mut self, controller: &Controller) -> Result<(), DrivetrainError>;
    /// Drive the robot using arcade controls (single-stick forward/back + single-stick turn).
    ///
    /// Behavior:
    /// - Forward/backward is read from the left stick Y axis.
    /// - Turning is read from the right stick X axis.
    /// - The two values are mixed into left/right voltages as:
    ///   - left = (fwd + turn) * 12.0
    ///   - right = (fwd - turn) * 12.0
    fn arcade(&mut self, controller: &Controller) -> Result<(), DrivetrainError>;
    /// Drive the robot using reversed tank controls (sticks swapped and inverted).
    ///
    /// Behavior:
    /// - Left motors take input from the RIGHT stick Y axis, inverted.
    /// - Right motors take input from the LEFT stick Y axis, inverted.
    /// - Computation:
    ///   - left = (-right_y) * 12.0
    ///   - right = (-left_y) * 12.0
    /// - This is useful when the robot is driving backwards but you want the sticks
    ///   to maintain an intuitive left/right mapping relative to the robot's new front.
    fn reverse_tank(&mut self, controller: &Controller) -> Result<(), DrivetrainError>;
    /// Drive the robot using reversed arcade controls (forward/turn both inverted).
    ///
    /// Behavior:
    /// - Forward/backward is read from the left stick Y axis, but inverted (fwd = -left_y).
    /// - Turning is read from the right stick X axis, also inverted (turn = -right_x).
    /// - Mixed into left/right voltages as:
    ///   - left = (fwd + turn) * 12.0
    ///   - right = (fwd - turn) * 12.0
    /// - This inversion preserves intuitive steering when the robot is driving backwards
    ///   (pushing the right stick right still causes a clockwise turn relative to the driver).
    /// - On controller read error, zeroed inputs are used and a warning is logged.
    ///
    /// Notes:
    /// * Inputs are assumed to be in the range [-1.0, 1.0] and are scaled to volts by 12.0.
    fn reverse_arcade(&mut self, controller: &Controller) -> Result<(), DrivetrainError>;
}

/// A Differential Drivetrain (or tank drive) is a drivetrain that has 2
/// separate sides that move independently to moe the robot.
pub trait Differential {
    /// Sets the brake mode for all motors in the drivetrain.
    ///
    /// The brake mode determines how motors behave when no voltage is applied:
    ///
    /// - [`BrakeMode::Coast`]: Motors spin freely.
    /// - [`BrakeMode::Brake`]: Motors actively resist rotation.
    /// - [`BrakeMode::Hold`]: Motors actively hold their position.
    fn set_brakemode(&self, brakemode: BrakeMode) -> Result<(), DrivetrainError>;
    /// Returns the average encoder position of all motors (left + right).
    ///
    /// The position is read from each motor’s integrated encoder and averaged.
    /// The result is returned as an [`Angle`].
    fn position(&self) -> Report<Angle, Vec<DrivetrainError>>;
    /// Returns the average encoder position of all left motors.
    fn left_position(&self) -> Report<Angle, Vec<DrivetrainError>>;
    /// Returns the average encoder position of all right motors.
    fn right_position(&self) -> Report<Angle, Vec<DrivetrainError>>;
    /// Resets the integrated encoder position on all drivetrain motors.
    ///
    /// This will attempt to reset both left and right motor groups.
    fn reset_position(&self) -> Result<(), DrivetrainError>;
    /// Sets the integrated encoder position on all drivetrain motors.
    fn set_position(&self, position: Angle) -> Result<(), DrivetrainError>;
    /// Sets the same voltage on all drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_voltage(&self, voltage: f64) -> Result<(), DrivetrainError>;
    /// Sets the same voltage on all *left* drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_left_voltage(&self, voltage: f64) -> Result<(), DrivetrainError>;
    /// Sets the same voltage on all *right* drivetrain motors.
    ///
    /// `voltage` is in volts.
    fn set_right_voltage(&self, voltage: f64) -> Result<(), DrivetrainError>;
}

/// Errors that can occur while commanding or reading from the drivetrain.
#[derive(Debug, Snafu)]
pub enum DrivetrainError {
    /// An error occurred while accessing a motor port (e.g. invalid port
    /// number, hardware failure, etc.).
    #[snafu(transparent)]
    PortError {
        /// The underlying error from the when trying to access a motor port.
        source: PortError,
    },
    /// An error occurred while reading the controller state (e.g. disconnected
    /// controller, communication error, etc.).
    #[snafu(transparent)]
    ControllerError {
        /// The underlying error from the when trying to read the controller
        /// state.
        source: ControllerError,
    },
    /// Failed to borrow the motor group mutably (e.g. already borrowed
    /// elsewhere).
    #[snafu(transparent)]
    BorrowMutError {
        /// The underlying error from trying to borrow the motor group mutably.
        source: BorrowMutError,
    },
    /// An unknown error occurred (catch-all for unexpected issues).
    Unknown {
        /// A string describing the unknown error.
        string: String,
    },
}
