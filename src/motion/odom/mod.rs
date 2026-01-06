//! Odometry tracking for robot position estimation.
//!
//! This module provides odometry tracking using perpendicular tracking wheels
//! and an inertial sensor to estimate the robot's global position on the field.
//!
//! # Module Structure
//!
//! - **[`devices`]**: Sensor abstractions and position types.
//! - **[`tracker`]**: The main odometry tracking controller.
//! - **[`ptsht`]**: Point-and-shoot navigation using odometry.
//! - **[`legacy`]**: Legacy odometry implementation (deprecated).
//!
//! # How It Works
//!
//! Odometry uses wheel encoders to measure how far the robot has traveled
//! and an inertial sensor (IMU) to measure rotation. By combining these
//! measurements over time, we can estimate the robot's (x, y) position
//! and heading on the field.
//!
//! # Hardware Requirements
//!
//! - **Vertical tracking wheel**: Measures forward/backward movement.
//! - **Horizontal tracking wheel**: Measures sideways (strafing) movement.
//! - **Inertial sensor (IMU)**: Measures rotation for accurate heading.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::odom::tracker::OdomTracker;
//! use antaeus::motion::odom::devices::{TrackerMech, Tracker, TrackingSensor};
//! use vexide::prelude::*;
//! use std::sync::Arc;
//! use vexide::sync::Mutex;
//!
//! // Create tracking sensors
//! let v_sensor = TrackingSensor::new_rotation_sensor(
//!     RotationSensor::new(peripherals.port_5, Direction::Forward)
//! );
//! let h_sensor = TrackingSensor::new_rotation_sensor(
//!     RotationSensor::new(peripherals.port_6, Direction::Forward)
//! );
//!
//! // Create trackers (2.75" wheel, 1:1 ratio, offset from center)
//! let vertical = Tracker::new(v_sensor, 2.75, 1.0, 1.0, 2.0);
//! let horizontal = Tracker::new(h_sensor, 2.75, 1.0, 1.0, 3.5);
//!
//! // Create mechanism with IMU
//! let imu = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_10)));
//! let mechanism = TrackerMech::new(vertical, horizontal, imu);
//!
//! // Create and start odometry
//! let odom = OdomTracker::new(mechanism);
//! odom.init();
//! ```

mod algorithm;

/// Tracking devices and position types.
pub mod devices;

/// Legacy odometry implementation.
///
/// **Deprecated**: Use [`tracker`] for new projects.
pub mod legacy;

/// Point-and-shoot navigation.
pub mod ptsht;

/// Main odometry tracking controller.
pub mod tracker;
