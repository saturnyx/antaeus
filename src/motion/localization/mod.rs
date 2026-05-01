//! Localization (odometry) tracking for robot position estimation.
//!
//! This module provides odometry tracking using perpendicular tracking wheels
//! and an inertial sensor to estimate the robot's global position on the field.
//!
//! # Module Structure
//!
//! * **[`tracker`]**: The main odometry tracking controller.
//! * **[`tracker::devices`]**: Sensor abstractions and pose types.
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
//! use antaeus::motion::localization::tracker::Tracker;
//! use antaeus::motion::localization::tracker::devices::{TrackerMech, TrackerPod, TrackingSensor};
//! use antaeus::misc::units::Length;
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
//! // Create tracker pods (2.75" wheel, 1:1 ratio, offset from center)
//! let vertical = TrackerPod::new(
//!     v_sensor,
//!     Length::from_inches(2.75),
//!     1.0,
//!     1.0,
//!     Length::from_inches(2.0),
//! );
//! let horizontal = TrackerPod::new(
//!     h_sensor,
//!     Length::from_inches(2.75),
//!     1.0,
//!     1.0,
//!     Length::from_inches(3.5),
//! );
//!
//! // Create mechanism with IMU
//! let imu = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_10)));
//! let mechanism = TrackerMech::new(vertical, horizontal, imu);
//!
//! // Create the tracker (call `tick` in your control loop)
//! let odom = Tracker::new(mechanism);
//! ```

use crate::motion::localization::tracker::devices::Pose;

pub mod tracker;

#[allow(async_fn_in_trait)]
pub trait Localizer {
    fn get_coords(&self) -> Pose;
    async fn tick(&mut self);
}
