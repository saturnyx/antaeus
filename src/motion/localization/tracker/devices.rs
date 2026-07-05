//! Tracking devices and position types for odometry.
//!
//! This module provides the sensor abstractions and data types used by the
//! odometry tracking system. It includes:
//!
//! * **TrackingSensor**: An abstraction over different encoder types.
//! * **TrackerPod**: Configuration for a tracking wheel with gear ratios.
//! * **TrackerMech**: The complete tracking mechanism with vertical/horizontal
//!   trackers and an IMU.
//! * **Pose**: A 2D position with heading.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::localization::tracker::devices::{TrackingSensor, TrackerPod, TrackerMech, Pose};
//! use antaeus::misc::units::Length;
//! use vexide::prelude::*;
//! use std::sync::Arc;
//! use vexide::sync::Mutex;
//!
//! // Create a tracking sensor from a rotation sensor
//! let sensor = TrackingSensor::new_rotation_sensor(
//!     RotationSensor::new(peripherals.port_5, Direction::Forward)
//! );
//!
//! // Create a tracker pod with wheel diameter, gear ratio, and offset
//! let tracker = TrackerPod::new(
//!     sensor,
//!     Length::from_inches(2.75),
//!     1.0,
//!     1.0,
//!     Length::zero(),
//! );
//! ```

use std::{cell::BorrowMutError, sync::Arc};

use snafu::Snafu;
use vexide::{
    adi::encoder::AdiOpticalEncoder,
    math::Angle,
    smart::{
        PortError,
        imu::{InertialError, InertialSensor},
        rotation::RotationSensor,
    },
    sync::Mutex,
};

use crate::{peripherals::drivetrain::DrivetrainError, utils::units::Length};

/// Errors that can occur while commanding or reading from the a tracking sensor.
#[derive(Debug, Snafu)]
pub enum TrackingSensorError {
    /// An error occurred while accessing a motor port (e.g. invalid port
    /// number, hardware failure, etc.).
    #[snafu(transparent)]
    PortError {
        /// The underlying error from the when trying to access a motor port.
        source: PortError,
    },
    /// Failed to borrow the motor group mutably (e.g. already borrowed
    /// elsewhere).
    #[snafu(transparent)]
    DrivetrainError {
        /// Errors that can occur while commanding or reading from the drivetrain.
        source: DrivetrainError,
    },

    /// Failed to retrieve IMU Reading
    #[snafu(transparent)]
    InertialError {
        /// IMU error
        source: InertialError,
    },
    /// An error returned by RefCell::try_borrow_mut.
    #[snafu(transparent)]
    BorrowMutError {
        /// An error returned by RefCell::try_borrow_mut.
        source: BorrowMutError,
    },
    /// An unknown error occurred (catch-all for unexpected issues).
    Unknown {
        /// A string describing the unknown error.
        string: String,
    },
}

/// Trackable Sensors
pub trait Trackable {
    /// Returns the current rotational position of the tracking sensor.
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError>;
    /// Resets the tracking sensor position to zero.
    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError>;
    /// Sets the tracking sensor position to a specific angle.
    fn set_track_position(&mut self, position: Angle) -> Result<(), TrackingSensorError>;
}

impl Trackable for AdiOpticalEncoder {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> { Ok(Self::position(self)?) }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> {
        Ok(Self::reset_position(self)?)
    }

    fn set_track_position(&mut self, position: Angle) -> Result<(), TrackingSensorError> {
        Ok(Self::set_position(self, position)?)
    }
}

impl Trackable for RotationSensor {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> { Ok(Self::position(self)?) }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> {
        Ok(Self::reset_position(self)?)
    }

    fn set_track_position(&mut self, position: Angle) -> Result<(), TrackingSensorError> {
        Ok(Self::set_position(self, position)?)
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
/// use antaeus::motion::localization::tracker::devices::{TrackingSensor, TrackerPod};
/// use antaeus::misc::units::Length;
/// use vexide::prelude::*;
///
/// let sensor = TrackingSensor::new_rotation_sensor(
///     RotationSensor::new(peripherals.port_5, Direction::Forward)
/// );
///
/// // 2.75" wheel, 1:1 gear ratio, no offset
/// let tracker = TrackerPod::new(
///     sensor,
///     Length::from_inches(2.75),
///     1.0,
///     1.0,
///     Length::zero(),
/// );
/// ```
pub struct TrackerPod<'s, S: Trackable> {
    /// The sensor measuring wheel rotation.
    pub sensor:         &'s mut S,
    /// The diameter of the tracking wheel in inches.
    pub wheel_diameter: Length,
    /// The number of teeth on the driven (wheel-side) gear.
    pub driven_gear:    f64,
    /// The number of teeth on the driving (encoder-side) gear.
    pub driving_gear:   f64,
    /// The perpendicular distance from the tracking center in inches.
    pub offset:         Length,
}

impl<'s, S: Trackable> TrackerPod<'s, S> {
    /// Creates a new tracker pod configuration.
    ///
    /// # Arguments
    ///
    /// * `sensor` - The tracking sensor to use.
    /// * `wheel_diameter` - The diameter of the tracking wheel in inches.
    /// * `driven_gear` - The number of teeth on the driven (wheel-side) gear.
    /// * `driving_gear` - The number of teeth on the driving (encoder-side) gear.
    /// * `offset` - The perpendicular distance from the tracking center in inches.
    pub fn new(
        sensor: &'s mut S,
        wheel_diameter: Length,
        driven_gear: f64,
        driving_gear: f64,
        offset: Length,
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
    pub fn dist(&mut self) -> Result<Length, TrackingSensorError> {
        let angle = self.sensor.track_position()?;
        let gear_ratio = self.driving_gear as f64 / self.driven_gear as f64;
        let distance = angle.as_radians() * gear_ratio * (self.wheel_diameter / 2.0);
        Ok(distance)
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
/// use antaeus::motion::localization::tracker::devices::{TrackingSensor, TrackerPod, TrackerMech};
/// use vexide::prelude::*;
/// use std::sync::Arc;
/// use vexide::sync::Mutex;
///
/// let vertical = TrackerPod::new(/* ... */);
/// let horizontal = TrackerPod::new(/* ... */);
/// let imu = Arc::new(Mutex::new(InertialSensor::new(peripherals.port_10)));
///
/// let mechanism = TrackerMech::new(vertical, horizontal, imu);
/// ```
pub struct TrackerMech<'v, 'h, V: Trackable, H: Trackable> {
    /// The vertical (forward/backward) tracking wheel.
    pub vertical_tracker:   TrackerPod<'v, V>,
    /// The horizontal (left/right) tracking wheel.
    pub horizontal_tracker: TrackerPod<'h, H>,
    /// The inertial sensor for heading measurement.
    pub imu:                Arc<Mutex<InertialSensor>>,
}

impl<'v, 'h, V: Trackable, H: Trackable> TrackerMech<'v, 'h, V, H> {
    /// Creates a new tracking mechanism.
    ///
    /// # Arguments
    ///
    /// * `vertical_tracker` - The vertical (forward/backward) tracking wheel.
    /// * `horizontal_tracker` - The horizontal (left/right) tracking wheel.
    /// * `imu` - The inertial sensor wrapped in a thread-safe Mutex.
    pub fn new(
        vertical_tracker: TrackerPod<'v, V>,
        horizontal_tracker: TrackerPod<'h, H>,
        imu: Arc<Mutex<InertialSensor>>,
    ) -> Self {
        Self {
            vertical_tracker,
            horizontal_tracker,
            imu,
        }
    }
}
