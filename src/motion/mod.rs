//! Autonomous motion control algorithms.
//!
//! This module provides tools for precise robot movement during autonomous
//! periods. It includes:
//!
//! # Structure
//!
//! * **Localization**: Position tracking / Odometry / Dead Reckoning
//! * **PID Control**: Proportional-Integral-Derivative controllers for accurate
//!   linear and rotational movement.
//! * **Path Following**: The Candidate-Based Pursuit algorithm for smooth path
//!   tracking.
//!
//! # Architecture
//!
//! The motion system is built around a tick-based architecture. This allows the
//! entire program to run on just a single thread. But this also means that a
//! `.tick()` function will have to be called once every cycle.

pub mod localization;

pub mod pursuit;

pub mod feedback_control;
