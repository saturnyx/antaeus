//! PID controller implementations for robot motion control.
//!
//! This module provides PID (Proportional-Integral-Derivative) controllers
//! for precise drivetrain control during autonomous periods.
//!
//! # Available Controllers
//!
//! - `pid`: Standard PID for controlling a differential drivetrain.
//!   Supports linear movement, rotation, and swing turns.
//! - `arcpid`: Arc PID for curved movements where the robot doesn't
//!   stop to turn.
//! - `singlepid`: Standalone PID for controlling a single motor group
//!   (e.g., an arm or lift).
//!
//! # How PID Works
//!
//! PID control calculates motor output based on three terms:
//!
//! - **P (Proportional)**: Output proportional to the error (distance from target).
//! - **I (Integral)**: Output proportional to accumulated error over time.
//! - **D (Derivative)**: Output proportional to the rate of error change.
//!
//! The formula is: `output = Kp*error + Ki*integral + Kd*derivative`
//!
//! # Tuning
//!
//! Start with Kp and increase until the robot reaches the target.
//! Add Kd to reduce overshoot. Only add Ki if the robot consistently
//! undershoots.

/// Arc PID controller for curved movements.
///
/// Allows the robot to move in arcs rather than stopping to turn.
/// This is less precise than standard PID but faster for some maneuvers.
pub mod arcpid;

/// Standard PID controller for differential drivetrains.
///
/// Provides linear movement, rotation, and swing turn capabilities
/// with configurable PID gains.
pub mod pid;

/// Standalone PID controller for single motor groups.
///
/// Useful for controlling mechanisms like arms, lifts, or flywheels
/// independently from the drivetrain.
pub mod singlepid;
