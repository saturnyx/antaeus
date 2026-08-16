//! Range sensor wrappers and Kalman-filtered range estimates.
//!
//! This module provides a unified [`RangeSensor`] interface for VEX range
//! hardware and a [`KalmanRangeSensor`] to smooth distance measurements and
//! estimate velocity using a constant-velocity model.
//!
//! # RangeSensor
//!
//! - [`RangeSensor::from_distance`] wraps a smart distance sensor.
//! - [`RangeSensor::from_adi`] wraps an ADI range finder (distance only).
//! - [`RangeSensor::distance`] returns the latest distance reading, if any.
//! - [`RangeSensor::velocity`] returns object velocity when supported.
//!
//! # KalmanRangeSensor
//!
//! Call [`KalmanRangeSensor::predict`] with a steady cadence, then
//! [`KalmanRangeSensor::tick`] to incorporate sensor readings. The filter
//! returns the current measurement, variance, and velocity estimate.
//!
//! # Example
//! ```
#![doc = include_str!("../../examples/range_sensor.rs")]
//! ```
use std::{
    cell::{BorrowError, RefCell},
    rc::Rc,
    time::Duration,
};

#[allow(unused_imports)]
use log::debug;
use snafu::Snafu;
use vexide::{
    adi::range_finder::AdiRangeFinder,
    smart::{
        PortError,
        distance::{DistanceObjectError, DistanceSensor},
    },
    time::user_uptime,
};

use crate::{
    make_cloneable,
    utils::units::{Length, Speed},
};

/// Unified range sensor interface for smart and ADI hardware.
#[derive(Debug, Clone)]
pub enum RangeSensor {
    /// Smart distance sensor with distance and velocity data.
    SmartDistSensor(Rc<RefCell<DistanceSensor>>),
    /// ADI range finder (distance only).
    AdiDistanceSensor(Rc<RefCell<AdiRangeFinder>>),
    /// Mock sensor for testing, allowing manual setting of distance and
    /// velocity.
    #[cfg(any(test, debug_assertions))]
    Mock {
        /// Optional distance value (None simulates no reading).
        distance: Option<Length>,
        /// Optional velocity value (None simulates no reading).
        velocity: Option<Speed>,
    },
}

/// Errors that can occur when reading from a range sensor.
#[derive(Debug, Snafu)]
pub enum RangeSensorError {
    /// Errors related to accessing the sensor port (e.g., disconnected,
    /// invalid port).
    #[snafu(display("Failed to access port: {port_error}"))]
    PortError {
        /// The underlying port error.
        port_error: PortError,
    },
    /// Errors indicating that the sensor could not be borrowed (e.g., already
    /// borrowed elsewhere).
    #[snafu(transparent)]
    BorrowError {
        /// The underlying borrow error when trying to access the sensor.
        source: BorrowError,
    },
    /// Errors related to retrieving the distance object from a smart distance
    /// sensor (e.g., sensor malfunction, communication failure).
    #[snafu(display("Failed to get object's distance : {distance_object_error}"))]
    DistanceObjectError {
        /// The underlying distance object error.
        distance_object_error: DistanceObjectError,
    },
    /// Errors indicating that the sensor did not detect a valid distance
    /// measurement.
    #[snafu(display("Sensor did not detect a distance"))]
    NoDistance,
    /// Errors indicating that the sensor did not detect a valid velocity
    /// measurement (applicable to smart distance sensors that support
    /// velocity).
    #[snafu(display("Sensor did not detect a velocity"))]
    NoVelocity,
}

impl RangeSensor {
    /// Wrap a smart distance sensor.
    pub fn from_distance(sensor: DistanceSensor) -> Self {
        Self::SmartDistSensor(make_cloneable(sensor))
    }

    /// Wrap an ADI range finder.
    pub fn from_adi(sensor: AdiRangeFinder) -> Self {
        Self::AdiDistanceSensor(make_cloneable(sensor))
    }

    /// Read the current distance measurement, if available.
    pub fn distance(&self) -> Result<Length, RangeSensorError> {
        match self {
            RangeSensor::AdiDistanceSensor(sensor) => match sensor.try_borrow()?.distance() {
                Ok(dist) => match dist {
                    Some(d) => Ok(Length::from_centimeters(d as f64)),
                    None => Err(RangeSensorError::NoDistance),
                },
                Err(e) => Err(RangeSensorError::PortError { port_error: e }),
            },
            RangeSensor::SmartDistSensor(sensor) => {
                let object = sensor.try_borrow()?.object();
                match object {
                    Ok(obj) => match obj {
                        Some(obj) => Ok(Length::from_centimetres(obj.distance as f64)),
                        None => Err(RangeSensorError::NoDistance),
                    },
                    Err(e) => Err(RangeSensorError::DistanceObjectError {
                        distance_object_error: e,
                    }),
                }
            }
            RangeSensor::Mock {
                distance,
                velocity: _,
            } => match distance {
                Some(dist) => Ok(*dist),
                None => Err(RangeSensorError::NoDistance),
            },
        }
    }

    /// Read the current velocity measurement when supported.
    ///
    /// ADI range finders do not report velocity, so this returns `None`.
    pub fn velocity(&self) -> Result<Speed, RangeSensorError> {
        match self {
            Self::SmartDistSensor(sensor) => {
                let obj = sensor.try_borrow()?.object();
                match obj {
                    Ok(Some(obj)) => Ok(Speed::from_metres_per_second(obj.velocity)),
                    Ok(None) => Err(RangeSensorError::NoVelocity),
                    Err(e) => Err(RangeSensorError::DistanceObjectError {
                        distance_object_error: e,
                    }),
                }
            }
            Self::AdiDistanceSensor(_) => Err(RangeSensorError::NoVelocity),
            Self::Mock {
                distance: _,
                velocity,
            } => match velocity {
                Some(vel) => Ok(*vel),
                None => Err(RangeSensorError::NoVelocity),
            },
        }
    }
}

/// Kalman-filtered range sensor wrapper.
///
/// Maintains a constant-velocity Kalman filter state over distance
/// measurements. Call [`KalmanRangeSensor::predict`] to advance the estimate,
/// then [`KalmanRangeSensor::tick`] to incorporate a new measurement.
#[derive(Debug)]
pub struct KalmanRangeSensor {
    sensor:          RangeSensor,
    process_var:     f64,
    measurement_var: f64,
    last_update:     Duration, // time_step
    prev_m:          Length,   // State
    prev_vel:        Speed,    // Velocity
    prev_var:        f64,
    est_m:           Length,
    est_var:         f64,
    new_m:           Length,
    new_var:         f64,
}
impl KalmanRangeSensor {
    /// Create a new Kalman range sensor.
    ///
    /// # Arguments
    ///
    /// * `sensor` - The range sensor to sample.
    /// * `process_var` - Process noise variance.
    /// * `measurement_var` - Measurement noise variance.
    /// * `initial_distance` - Initial distance estimate.
    /// * `initial_velocity` - Initial velocity estimate.
    pub fn new(
        sensor: RangeSensor,
        process_var: f64,
        measurement_var: f64,
        initial_distance: Length,
        initial_velocity: Speed,
    ) -> Self {
        Self {
            sensor,
            process_var,
            measurement_var,
            last_update: user_uptime(),
            prev_m: initial_distance,
            prev_vel: initial_velocity,
            prev_var: measurement_var, // Start with measurement variance
            est_m: initial_distance,
            est_var: measurement_var,
            new_m: initial_distance,
            new_var: measurement_var,
        }
    }

    /// Predict the next state using elapsed time since the last update.
    pub fn predict(&mut self) {
        let elapsed = user_uptime() - self.last_update;
        let dx = self.prev_vel * elapsed;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update = user_uptime();
    }

    /// Predict with an explicit time step (test-only helper).
    #[cfg(any(test, debug_assertions))]
    pub fn predict_with_dt(&mut self, dt: Duration) {
        let dx = self.prev_vel * dt;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update += dt;
    }

    /// Update the filter with the latest sensor measurement.
    ///
    /// If a measurement is unavailable, the predicted state is retained.
    pub fn tick(&mut self) {
        let m = match self.sensor.distance() {
            Ok(d) => d,
            Err(_) => return,
        };

        let residual = m - self.est_m;
        let kalman_gain = self.est_var / (self.est_var + self.measurement_var);
        self.new_m = self.est_m + (kalman_gain * residual);
        self.new_var = (1.0 - kalman_gain) * self.est_var;

        self.prev_m = self.new_m;
        self.prev_var = self.new_var;
        if let Ok(vel) = self.sensor.velocity() {
            self.prev_vel = vel;
        } else {
            let distance_change = self.new_m - self.prev_m;
            let elapsed = user_uptime() - self.last_update;
            self.prev_vel = distance_change / elapsed;
        }
    }

    /// Return the most recent filtered distance measurement.
    pub fn measurement(&self) -> Length { self.new_m }

    /// Return the current measurement variance.
    pub fn variance(&self) -> f64 { self.new_var }

    /// Return the current velocity estimate.
    pub fn velocity(&self) -> Speed { self.prev_vel }

    /// Return the predicted distance measurement.
    pub fn predicted_measurement(&self) -> Length { self.est_m }

    /// Return the predicted measurement variance.
    pub fn predicted_variance(&self) -> f64 { self.est_var }

    /// Reset the filter state to new initial values.
    pub fn reset(&mut self, initial_distance: Length, initial_velocity: Speed) {
        self.prev_m = initial_distance;
        self.prev_vel = initial_velocity;
        self.prev_var = self.measurement_var;
        self.est_m = initial_distance;
        self.est_var = self.measurement_var;
        self.new_m = initial_distance;
        self.new_var = self.measurement_var;
        self.last_update = user_uptime();
    }

    /// Update the sensor with mock data (test-only helper).
    #[cfg(any(test, debug_assertions))]
    pub fn set_sensor_mock(&mut self, distance: Length, velocity: Speed) {
        self.sensor = RangeSensor::Mock {
            distance: Some(distance),
            velocity: Some(velocity),
        };
    }
}

#[cfg(test)]
mod tests {
    use vexide::prelude::Peripherals;

    use super::*;

    /// Test data structure with proper Length and Speed types
    struct TestDataPoint {
        distance: Length,
        velocity: Speed,
        dt:       Duration,
    }

    #[vexide::test]
    async fn test_kalman_filter_predict_update(_p: Peripherals) {
        // Test Kalman filter predict/update cycle with perfect and noisy measurements
        // Scenario 1: Clean data - object at constant 1 m/s
        let test_data_clean = [
            TestDataPoint {
                distance: Length::from_metres(1.0),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Length::from_metres(1.1),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Length::from_metres(1.2),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Length::from_metres(1.3),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Length::from_metres(1.4),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
        ];

        let initial_dist = Length::from_metres(1.0);
        let initial_vel = Speed::from_metres_per_second(1.0);

        let mut kalman = KalmanRangeSensor::new(
            RangeSensor::Mock {
                distance: Some(initial_dist),
                velocity: Some(initial_vel),
            },
            0.01, // process variance
            0.05, // measurement variance
            initial_dist,
            initial_vel,
        );

        debug!("=== Kalman Filter Clean Data Test ===");

        for (i, data_point) in test_data_clean.iter().enumerate() {
            kalman.predict_with_dt(data_point.dt);
            let predicted_dist = kalman.predicted_measurement().as_metres();
            let predicted_var = kalman.predicted_variance();

            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.tick();
            let updated_dist = kalman.measurement().as_metres();
            let updated_var = kalman.variance();

            // Assertions for clean data
            assert!(updated_dist > 0.0, "Step {}: Length should be positive", i + 1);
            assert!(
                updated_var < predicted_var,
                "Step {}: Variance should decrease after update",
                i + 1
            );
            assert!(updated_var > 0.0, "Step {}: Variance should be positive", i + 1);

            let min_dist = predicted_dist.min(data_point.distance.as_metres());
            let max_dist = predicted_dist.max(data_point.distance.as_metres());
            assert!(
                updated_dist >= min_dist - 0.01 && updated_dist <= max_dist + 0.01,
                "Step {}: Updated distance should be between prediction and measurement",
                i + 1
            );
        }

        // Scenario 2: Noisy data - object at constant 2 m/s with measurement noise
        let test_data_noisy = [
            TestDataPoint {
                distance: Length::from_metres(2.0),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Length::from_metres(2.15),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Length::from_metres(2.05),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Length::from_metres(2.25),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Length::from_metres(2.20),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
        ];

        let initial_dist = Length::from_metres(2.0);
        let initial_vel = Speed::from_metres_per_second(2.0);

        let mut kalman = KalmanRangeSensor::new(
            RangeSensor::Mock {
                distance: Some(initial_dist),
                velocity: Some(initial_vel),
            },
            0.02, // process variance
            0.1,  // measurement variance (high for noisy sensor)
            initial_dist,
            initial_vel,
        );

        debug!("=== Kalman Filter Noisy Data Test ===");
        let mut prev_var = kalman.variance();

        for (i, data_point) in test_data_noisy.iter().enumerate() {
            kalman.predict_with_dt(data_point.dt);
            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.tick();

            let current_var = kalman.variance();

            // Assertions for noisy data
            assert!(current_var > 0.0, "Step {}: Variance should be positive", i + 1);
            assert!(
                current_var <= prev_var + 0.01, // small tolerance for process noise
                "Step {}: Variance should not increase significantly",
                i + 1
            );

            prev_var = current_var;
        }

        debug!("=== Kalman Filter Tests Complete ===");
    }

    #[test]
    fn test_reset_functionality() {
        let initial_dist = Length::from_metres(1.0);
        let initial_vel = Speed::from_metres_per_second(0.0);

        let mut kalman = KalmanRangeSensor::new(
            RangeSensor::Mock {
                distance: Some(initial_dist),
                velocity: Some(initial_vel),
            },
            0.01,
            0.1,
            initial_dist,
            initial_vel,
        );

        // Verify initial state
        assert_eq!(kalman.measurement().as_metres(), 1.0);

        // Reset to different values
        let new_dist = Length::from_metres(5.0);
        let new_vel = Speed::from_metres_per_second(0.5);
        kalman.reset(new_dist, new_vel);

        // Verify reset worked
        assert_eq!(kalman.measurement().as_metres(), 5.0);
        assert_eq!(kalman.velocity().as_metres_per_second(), 0.5);
    }
}
