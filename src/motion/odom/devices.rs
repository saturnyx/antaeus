//! Tracking devices and position types for odometry.
//!
//! This module provides the sensor abstractions and data types used by the
//! odometry tracking system. It includes:
//!
//! - **TrackingSensor**: An abstraction over different encoder types.
//! - **Tracker**: Configuration for a tracking wheel with gear ratios.
//! - **TrackerMech**: The complete tracking mechanism with vertical/horizontal
//!   trackers and an IMU.
//! - **Pose**: A 2D position with heading.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::odom::devices::{TrackingSensor, Tracker, TrackerMech, Pose};
//! use vexide::prelude::*;
//! use std::sync::Arc;
//! use vexide::sync::Mutex;
//!
//! // Create a tracking sensor from a rotation sensor
//! let sensor = TrackingSensor::new_rotation_sensor(
//!     RotationSensor::new(peripherals.port_5, Direction::Forward)
//! );
//!
//! // Create a tracker with wheel diameter, gear ratio, and offset
//! let tracker = Tracker::new(sensor, 2.75, 1.0, 1.0, 0.0);
//! ```

use std::sync::Arc;

use log::warn;
use vexide::{
    adi::encoder::AdiOpticalEncoder,
    math::Angle,
    smart::{PortError, imu::InertialSensor, rotation::RotationSensor},
    sync::Mutex,
};

use crate::peripherals::drivetrain::Differential;

/// An abstraction over different encoder types used for tracking.
///
/// This enum allows the odometry system to work with various sensor types
/// without needing separate code paths for each.
///
/// # Variants
///
/// - `AdiOpticalEncoder`: A 3-wire optical shaft encoder.
/// - `RotationSensor`: A V5 rotation sensor (high resolution).
/// - `Differential`: Uses drivetrain motor encoders as tracking sensors.
/// - `None`: Placeholder for optional tracking (returns zero).
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::devices::TrackingSensor;
/// use vexide::prelude::*;
///
/// // Using a rotation sensor
/// let sensor = TrackingSensor::new_rotation_sensor(
///     RotationSensor::new(peripherals.port_5, Direction::Forward)
/// );
///
/// // Using the drivetrain motors
/// let sensor = TrackingSensor::new_differential(drivetrain);
/// ```
#[derive(Clone)]
pub enum TrackingSensor {
    /// An ADI (3-wire) optical shaft encoder.
    AdiOpticalEncoder(Arc<Mutex<AdiOpticalEncoder>>),
    /// A V5 rotation sensor (high-resolution encoder).
    RotationSensor(Arc<Mutex<RotationSensor>>),
    /// Uses the drivetrain motors' integrated encoders.
    Differential(Differential),
    /// No tracking sensor (placeholder for optional tracking).
    None,
}

impl TrackingSensor {
    /// Creates a new TrackingSensor from an ADI optical encoder.
    ///
    /// # Arguments
    ///
    /// * `encoder` - The ADI optical encoder to use for tracking.
    pub fn new_adi_optical_encoder(encoder: AdiOpticalEncoder) -> Self {
        Self::AdiOpticalEncoder(Arc::new(Mutex::new(encoder)))
    }

    /// Creates a new TrackingSensor from a V5 rotation sensor.
    ///
    /// # Arguments
    ///
    /// * `sensor` - The rotation sensor to use for tracking.
    pub fn new_rotation_sensor(sensor: RotationSensor) -> Self {
        Self::RotationSensor(Arc::new(Mutex::new(sensor)))
    }

    /// Creates a new TrackingSensor from a differential drivetrain.
    ///
    /// # Arguments
    ///
    /// * `diff` - The differential drivetrain whose motor encoders will be used.
    pub fn new_differential(diff: Differential) -> Self { Self::Differential(diff) }

    /// Creates a placeholder TrackingSensor that always returns zero.
    pub fn new_none() -> Self { Self::None }

    /// Returns the current rotational position of the tracking sensor.
    ///
    /// # Returns
    ///
    /// The position as an [`Angle`]. Returns zero if the device
    /// encounters an error (a warning is logged).
    pub async fn position(&self) -> Angle {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => {
                encoder.lock().await.position().unwrap_or_else(|e| {
                    warn!("ADI Optical Encoder Position Error: {}", e);
                    Angle::from_radians(0.0)
                })
            }
            TrackingSensor::RotationSensor(encoder) => {
                encoder.lock().await.position().unwrap_or_else(|e| {
                    warn!("Rotation Sensor Position Error: {}", e);
                    Angle::from_radians(0.0)
                })
            }
            TrackingSensor::Differential(dt) => dt.position(),
            TrackingSensor::None => Angle::from_radians(0.0),
        }
    }

    /// Resets the tracking sensor position to zero.
    ///
    /// # Errors
    ///
    /// Returns a [`PortError`] if the sensor is disconnected or encounters an error.
    pub async fn reset_position(&self) -> Result<(), PortError> {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => encoder.lock().await.reset_position(),
            TrackingSensor::RotationSensor(encoder) => encoder.lock().await.reset_position(),
            TrackingSensor::Differential(dt) => dt.reset_position(),
            TrackingSensor::None => Ok(()),
        }
    }

    /// Sets the tracking sensor position to a specific angle.
    ///
    /// # Arguments
    ///
    /// * `position` - The angle to set as the current position.
    ///
    /// # Errors
    ///
    /// Returns a [`PortError`] if the sensor is disconnected or encounters an error.
    pub async fn set_position(&self, position: Angle) -> Result<(), PortError> {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => {
                encoder.lock().await.set_position(position)
            }
            TrackingSensor::RotationSensor(encoder) => encoder.lock().await.set_position(position),
            TrackingSensor::Differential(dt) => dt.set_position(position),
            TrackingSensor::None => Ok(()),
        }
    }
}

/// Configuration for a tracking wheel.
///
/// A tracking wheel is an unpowered wheel with an encoder used to measure
/// how far the robot has traveled. This struct combines the sensor with
/// physical wheel properties and gear ratios.
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::devices::{TrackingSensor, Tracker};
/// use vexide::prelude::*;
///
/// let sensor = TrackingSensor::new_rotation_sensor(
///     RotationSensor::new(peripherals.port_5, Direction::Forward)
/// );
///
/// // 2.75" wheel, 1:1 gear ratio, no offset
/// let tracker = Tracker::new(sensor, 2.75, 1.0, 1.0, 0.0);
/// ```
#[derive(Clone)]
pub struct Tracker {
    /// The sensor measuring wheel rotation.
    pub sensor:         TrackingSensor,
    /// The diameter of the tracking wheel in inches.
    pub wheel_diameter: f64,
    /// The number of teeth on the driven (wheel-side) gear.
    pub driven_gear:    f64,
    /// The number of teeth on the driving (encoder-side) gear.
    pub driving_gear:   f64,
    /// The perpendicular distance from the tracking center in inches.
    pub offset:         f64,
}

impl Tracker {
    /// Creates a new Tracker configuration.
    ///
    /// # Arguments
    ///
    /// * `sensor` - The tracking sensor to use.
    /// * `wheel_diameter` - The diameter of the tracking wheel in inches.
    /// * `driven_gear` - The number of teeth on the driven (wheel-side) gear.
    /// * `driving_gear` - The number of teeth on the driving (encoder-side) gear.
    /// * `offset` - The perpendicular distance from the tracking center in inches.
    pub fn new(
        sensor: TrackingSensor,
        wheel_diameter: f64,
        driven_gear: f64,
        driving_gear: f64,
        offset: f64,
    ) -> Self {
        Self {
            sensor,
            wheel_diameter,
            driven_gear,
            driving_gear,
            offset,
        }
    }

    /// Calculates the distance traveled by the tracking wheel.
    ///
    /// Takes into account the wheel diameter, gear ratio, and offset.
    ///
    /// # Returns
    ///
    /// The distance traveled in inches.
    pub async fn dist(&self) -> f64 {
        let angle = self.sensor.position().await;
        let gear_ratio = self.driving_gear as f64 / self.driven_gear as f64;
        let distance = angle.as_radians() * gear_ratio * (self.wheel_diameter / 2.0);
        distance + self.offset
    }
}

/// The complete tracking mechanism for odometry.
///
/// Groups together the vertical tracker, horizontal tracker, and IMU
/// needed for position estimation.
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::devices::{TrackingSensor, Tracker, TrackerMech};
/// use vexide::prelude::*;
/// use std::sync::Arc;
/// use vexide::sync::Mutex;
///
/// let vertical = Tracker::new(/* ... */);
/// let horizontal = Tracker::new(/* ... */);
/// let imu = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_10)));
///
/// let mechanism = TrackerMech::new(vertical, horizontal, imu);
/// ```
#[derive(Clone)]
pub struct TrackerMech {
    /// The vertical (forward/backward) tracking wheel.
    pub vertical_tracker:   Tracker,
    /// The horizontal (left/right) tracking wheel.
    pub horizontal_tracker: Tracker,
    /// The inertial sensor for heading measurement.
    pub imu:                Arc<Mutex<InertialSensor>>,
}

impl TrackerMech {
    /// Creates a new tracking mechanism.
    ///
    /// # Arguments
    ///
    /// * `vertical_tracker` - The vertical (forward/backward) tracking wheel.
    /// * `horizontal_tracker` - The horizontal (left/right) tracking wheel.
    /// * `imu` - The inertial sensor wrapped in a thread-safe Mutex.
    pub fn new(
        vertical_tracker: Tracker,
        horizontal_tracker: Tracker,
        imu: Arc<Mutex<InertialSensor>>,
    ) -> Self {
        Self {
            vertical_tracker,
            horizontal_tracker,
            imu,
        }
    }
}

/// A 2D position with heading.
///
/// Represents the robot's position on the field with x and y coordinates
/// and a heading angle.
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::devices::Pose;
/// use vexide::math::Angle;
///
/// // Start at origin facing forward
/// let pose = Pose::origin();
///
/// // Create a custom pose
/// let pose = Pose::new(24.0, 12.0, Angle::from_degrees(45.0));
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    /// The x-coordinate in inches.
    pub x: f64,
    /// The y-coordinate in inches.
    pub y: f64,
    /// The heading angle.
    pub t: Angle,
}

impl Pose {
    /// Creates a new Pose with the specified position and heading.
    ///
    /// # Arguments
    ///
    /// * `x` - The x-coordinate in inches.
    /// * `y` - The y-coordinate in inches.
    /// * `t` - The heading angle.
    pub fn new(x: f64, y: f64, t: Angle) -> Self { Self { x, y, t } }

    /// Creates a Pose at the origin (0, 0) with heading 0.
    pub fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            t: Angle::from_radians(0.0),
        }
    }
}
