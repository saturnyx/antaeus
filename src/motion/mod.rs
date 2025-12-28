//! Autonomous motion control algorithms.
//!
//! This module provides tools for precise robot movement during autonomous
//! periods. It includes:
//!
//! - **Odometry**: Position tracking using tracking wheels and an inertial sensor.
//! - **PID Control**: Proportional-Integral-Derivative controllers for accurate
//!   linear and rotational movement.
//! - **Path Following**: The Candidate-Based Pursuit algorithm for smooth path
//!   tracking.
//!
//! # Architecture
//!
//! The motion system is built around asynchronous control loops that run
//! independently from your main autonomous routine. You initialize the
//! controllers, then call movement methods that set targets and wait for
//! completion.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::pid::pid::{PIDMovement, PIDValues, DrivetrainConfig};
//!
//! // Create and initialize PID controller
//! let pid = PIDMovement { /* ... */ };
//! pid.init();
//!
//! // Execute movements
//! pid.travel(24.0, 2000, 100).await;  // Move 24 inches
//! pid.rotate(90.0, 2000, 100).await;  // Turn 90 degrees
//! ```

/// Odometry tracking for position estimation.
///
/// Provides the [`OdomMovement`](odom::OdomMovement) struct for tracking
/// the robot's global position using tracking wheels and an inertial sensor.
pub mod odom;

/// PID control algorithms.
///
/// Contains multiple PID implementations:
/// - [`pid`](pid::pid): Standard PID for drivetrain control.
/// - [`arcpid`](pid::arcpid): PID that allows arc movements.
/// - [`singlepid`](pid::singlepid): PID for single motor groups.
pub mod pid;

/// Candidate-Based Pursuit path following algorithm.
///
/// A more robust variant of pure pursuit that handles edge cases
/// like the robot being off the path or near path endpoints.
pub mod pusuit;
