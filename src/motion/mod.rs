//! Autonomous motion control algorithms.
//!
//! This module provides tools for precise robot movement during autonomous
//! periods. It includes:
//!
//! * **Odometry**: Position tracking using tracking wheels and an inertial sensor.
//! * **PID Control**: Proportional-Integral-Derivative controllers for accurate
//!   linear and rotational movement.
//! * **Path Following**: The Candidate-Based Pursuit algorithm for smooth path
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
//! use antaeus::motion::feedback_control::legacy_pid::linear_pid::{PIDMovement, PIDValues};
//! use antaeus::motion::feedback_control::legacy_pid::DrivetrainConfig;
//!
//! // Create and initialize PID controller
//! let pid = PIDMovement { /* ... */ };
//! pid.init();
//!
//! // Execute movements
//! pid.travel(24.0, 2000, 100).await;  // Move 24 inches
//! pid.rotate(90.0, 2000, 100).await;  // Turn 90 degrees
//! ```

pub mod localization;

pub mod pursuit;

pub mod feedback_control;
