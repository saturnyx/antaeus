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
//! [`KalmanRangeSensor::update`] to incorporate sensor readings. The filter
//! returns the current measurement, variance, and velocity estimate.
//!
//! # Example
//!
//! ```ignore
//! use crate::peripherals::range_sensor::{KalmanRangeSensor, RangeSensor};
//! use measurements::{Distance, Speed};
//! use vexide::smart::distance::DistanceSensor;
//!
//! let sensor = DistanceSensor::new(1);
//! let range = RangeSensor::from_distance(sensor);
//! let mut filter = KalmanRangeSensor::new(
//!     range,
//!     0.02,
//!     0.1,
//!     Distance::from_metres(0.5),
//!     Speed::from_metres_per_second(0.0),
//! );
//!
//! filter.predict().await;
//! filter.update().await;
//! let filtered = filter.measurement();
//! ```
use std::{sync::Arc, time::Duration};

#[allow(unused_imports)]
use log::debug;
use log::warn;
use measurements::{Distance, Speed};
use vexide::{
    adi::range_finder::AdiRangeFinder,
    smart::distance::DistanceSensor,
    sync::Mutex,
    time::user_uptime,
};

/// Unified range sensor interface for smart and ADI hardware.
pub enum RangeSensor {
    /// Smart distance sensor with distance and velocity data.
    SmartDistSensor(Arc<Mutex<DistanceSensor>>),
    /// ADI range finder (distance only).
    AdiDistanceSensor(Arc<Mutex<AdiRangeFinder>>),
    #[cfg(any(test, debug_assertions))]
    Mock {
        distance: Option<Distance>,
        velocity: Option<Speed>,
    },
}

impl RangeSensor {
    /// Wrap a smart distance sensor.
    pub fn from_distance(sensor: DistanceSensor) -> Self {
        Self::SmartDistSensor(Arc::new(Mutex::new(sensor)))
    }

    /// Wrap an ADI range finder.
    pub fn from_adi(sensor: AdiRangeFinder) -> Self {
        Self::AdiDistanceSensor(Arc::new(Mutex::new(sensor)))
    }

    /// Read the current distance measurement, if available.
    pub async fn distance(&self) -> Option<Distance> {
        match self {
            RangeSensor::AdiDistanceSensor(sensor) => match sensor.lock().await.distance() {
                Ok(dist) => match dist {
                    Some(d) => Some(Distance::from_centimeters(d as f64)),
                    None => None,
                },
                Err(e) => {
                    warn!("Error getting distance: {}", e);
                    None
                }
            },
            RangeSensor::SmartDistSensor(sensor) => {
                let object = sensor.lock().await.object();
                match object {
                    Ok(obj) => match obj {
                        Some(obj) => Some(Distance::from_micrometres(obj.distance as f64)),
                        None => None,
                    },
                    Err(e) => {
                        warn!("Error getting object from distance sensor: {}", e);
                        None
                    }
                }
            }
            RangeSensor::Mock {
                distance,
                velocity: _,
            } => distance.clone(),
        }
    }

    /// Read the current velocity measurement when supported.
    ///
    /// ADI range finders do not report velocity, so this returns `None`.
    pub async fn velocity(&self) -> Option<Speed> {
        match self {
            Self::SmartDistSensor(sensor) => {
                let obj = sensor.lock().await.object();
                match obj {
                    Ok(Some(obj)) => Some(Speed::from_metres_per_second(obj.velocity)),
                    Ok(None) => None,
                    Err(e) => {
                        warn!("Error getting velocity {}", e);
                        None
                    }
                }
            }
            Self::AdiDistanceSensor(_) => None,
            Self::Mock {
                distance: _,
                velocity,
            } => velocity.clone(),
        }
    }
}

/// Kalman-filtered range sensor wrapper.
///
/// Maintains a constant-velocity Kalman filter state over distance
/// measurements. Call [`KalmanRangeSensor::predict`] to advance the estimate,
/// then [`KalmanRangeSensor::update`] to incorporate a new measurement.
pub struct KalmanRangeSensor {
    sensor:          RangeSensor,
    process_var:     f64,
    measurement_var: f64,
    last_update:     Duration, // time_step
    prev_m:          Distance, // State
    prev_vel:        Speed,    // Velocity
    prev_var:        f64,
    est_m:           Distance,
    est_var:         f64,
    new_m:           Distance,
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
        initial_distance: Distance,
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
    pub async fn predict(&mut self) {
        let elapsed = user_uptime() - self.last_update;
        let dx = self.prev_vel * elapsed;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update = user_uptime();
    }

    /// Predict with an explicit time step (test-only helper).
    #[cfg(any(test, debug_assertions))]
    pub async fn predict_with_dt(&mut self, dt: Duration) {
        let dx = self.prev_vel * dt;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update = self.last_update + dt;
    }

    /// Update the filter with the latest sensor measurement.
    ///
    /// If a measurement is unavailable, the predicted state is retained.
    pub async fn update(&mut self) {
        let m = match self.sensor.distance().await {
            Some(val) => val,
            None => {
                warn!("Error getting distance");
                self.new_m = self.est_m;
                self.new_var = self.est_var;
                return; // Exit fn
            }
        };

        let residual = m - self.est_m;
        let kalman_gain = self.est_var / (self.est_var + self.measurement_var);
        self.new_m = self.est_m + (kalman_gain * residual);
        self.new_var = (1.0 - kalman_gain) * self.est_var;

        self.prev_m = self.new_m;
        self.prev_var = self.new_var;
        if let Some(vel) = self.sensor.velocity().await {
            self.prev_vel = vel;
        } else {
            let distance_change = self.new_m - self.prev_m;
            let elapsed = user_uptime() - self.last_update;
            self.prev_vel = distance_change / elapsed;
        }
    }

    /// Return the most recent filtered distance measurement.
    pub fn measurement(&self) -> Distance { self.new_m }

    /// Return the current measurement variance.
    pub fn variance(&self) -> f64 { self.new_var }

    /// Return the current velocity estimate.
    pub fn velocity(&self) -> Speed { self.prev_vel }

    /// Return the predicted distance measurement.
    pub fn predicted_measurement(&self) -> Distance { self.est_m }

    /// Return the predicted measurement variance.
    pub fn predicted_variance(&self) -> f64 { self.est_var }

    /// Reset the filter state to new initial values.
    pub fn reset(&mut self, initial_distance: Distance, initial_velocity: Speed) {
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
    pub fn set_sensor_mock(&mut self, distance: Distance, velocity: Speed) {
        self.sensor = RangeSensor::Mock {
            distance: Some(distance),
            velocity: Some(velocity),
        };
    }
}

#[cfg(test)]
mod tests {
    use rand_distr::Normal;
    use vexide::prelude::Peripherals;

    use super::*;

    /// Test data structure with proper Distance and Speed types
    struct TestDataPoint {
        distance: Distance,
        velocity: Speed,
        dt:       Duration,
    }

    /// Helper struct to collect test data for visualization
    struct FilterTestData {
        time_steps:   Vec<f64>,
        measurements: Vec<f64>,
        filtered:     Vec<f64>,
        predicted:    Vec<f64>,
        variance:     Vec<f64>,
    }

    /// Generate random noisy test data following a linear trajectory
    ///
    /// # Parameters
    /// - `num_samples`: Number of data points to generate
    /// - `start_distance`: Starting distance in meters
    /// - `velocity`: Constant velocity in m/s
    /// - `dt_ms`: Time step in milliseconds
    /// - `noise_amplitude`: Noise standard deviation in meters
    fn generate_random_test_data(
        num_samples: usize,
        start_distance: f64,
        velocity: f64,
        dt_ms: u64,
        noise_amplitude: f64,
    ) -> Vec<TestDataPoint> {
        use rand_distr::Distribution;

        let mut rng = rand::thread_rng();
        let mut data = Vec::new();
        let dt = Duration::from_millis(dt_ms);
        let dt_secs = dt_ms as f64 / 1000.0;
        let normal =
            Normal::new(0.0, noise_amplitude).expect("Failed to create normal distribution");

        for i in 0..num_samples {
            // True distance following linear trajectory
            let true_distance = start_distance + velocity * (i as f64) * dt_secs;

            // Add Gaussian noise
            let measurement: f64 = true_distance + normal.sample(&mut rng);

            data.push(TestDataPoint {
                distance: Distance::from_metres(measurement.max(0.0)), // Ensure non-negative
                velocity: Speed::from_metres_per_second(velocity),
                dt,
            });
        }

        data
    }

    impl FilterTestData {
        fn new() -> Self {
            Self {
                time_steps:   Vec::new(),
                measurements: Vec::new(),
                filtered:     Vec::new(),
                predicted:    Vec::new(),
                variance:     Vec::new(),
            }
        }

        fn add_step(&mut self, time: f64, measured: f64, filtered: f64, predicted: f64, var: f64) {
            self.time_steps.push(time);
            self.measurements.push(measured);
            self.filtered.push(filtered);
            self.predicted.push(predicted);
            self.variance.push(var);
        }

        /// Generate an HTML plot using plotly
        fn plot_to_html(&self, filename: &str) -> std::io::Result<()> {
            use plotly::{Plot, Scatter};

            let measured = Scatter::new(self.time_steps.clone(), self.measurements.clone())
                .name("Measured (Mock)")
                .mode(plotly::common::Mode::LinesMarkers);

            let filtered = Scatter::new(self.time_steps.clone(), self.filtered.clone())
                .name("Filtered (Kalman)")
                .mode(plotly::common::Mode::Lines);

            let predicted = Scatter::new(self.time_steps.clone(), self.predicted.clone())
                .name("Predicted")
                .mode(plotly::common::Mode::Lines);

            let mut plot = Plot::new();
            plot.add_trace(measured);
            plot.add_trace(predicted);
            plot.add_trace(filtered);

            plot.set_layout(
                plotly::Layout::new()
                    .title(plotly::common::Title::new("Kalman Filter: Raw vs Filtered Data"))
                    .x_axis(
                        plotly::layout::Axis::new()
                            .title(plotly::common::Title::new("Time (steps)")),
                    )
                    .y_axis(
                        plotly::layout::Axis::new()
                            .title(plotly::common::Title::new("Distance (m)")),
                    ),
            );

            plot.write_html(filename);
            Ok(())
        }

        /// Fallback: print table format for debugging
        fn print_table(&self) {
            println!("\n{:=^80}", " Kalman Filter Test Data ");
            println!(
                "{:<8} {:<15} {:<15} {:<15} {:<15}",
                "Step", "Measured (m)", "Filtered (m)", "Predicted (m)", "Variance"
            );
            println!("{:-<80}", "");

            for i in 0..self.time_steps.len() {
                println!(
                    "{:<8} {:<15.4} {:<15.4} {:<15.4} {:<15.6}",
                    i, self.measurements[i], self.filtered[i], self.predicted[i], self.variance[i]
                );
            }
            println!("{:=<80}\n", "");
        }
    }

    #[vexide::test]
    async fn test_kalman_filter_predict_update(_p: Peripherals) {
        // Test Kalman filter predict/update cycle with perfect and noisy measurements
        // Scenario 1: Clean data - object at constant 1 m/s
        let test_data_clean = vec![
            TestDataPoint {
                distance: Distance::from_metres(1.0),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Distance::from_metres(1.1),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Distance::from_metres(1.2),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Distance::from_metres(1.3),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
            TestDataPoint {
                distance: Distance::from_metres(1.4),
                velocity: Speed::from_metres_per_second(1.0),
                dt:       Duration::from_millis(100),
            },
        ];

        let initial_dist = Distance::from_metres(1.0);
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
            kalman.predict_with_dt(data_point.dt).await;
            let predicted_dist = kalman.predicted_measurement().as_metres();
            let predicted_var = kalman.predicted_variance();

            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.update().await;
            let updated_dist = kalman.measurement().as_metres();
            let updated_var = kalman.variance();

            // Assertions for clean data
            assert!(updated_dist > 0.0, "Step {}: Distance should be positive", i + 1);
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
        let test_data_noisy = vec![
            TestDataPoint {
                distance: Distance::from_metres(2.0),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.15),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.05),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.25),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.20),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
        ];

        let initial_dist = Distance::from_metres(2.0);
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
            kalman.predict_with_dt(data_point.dt).await;
            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.update().await;

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

    #[vexide::test]
    #[ignore = "manual verification needed (graph)"]
    async fn test_kalman_filter_visualization(_p: Peripherals) {
        // Generate random noisy test data
        // Object moving at constant 2 m/s with Gaussian noise (σ = 0.15m)
        let test_data = generate_random_test_data(
            100,  // 100 samples for more comprehensive visualization
            2.0,  // Start at 2.0m
            2.0,  // Velocity: 2.0 m/s
            50,   // Time step: 50ms
            0.15, // Noise amplitude (standard deviation): 0.15m
        );

        let initial_dist = Distance::from_metres(2.0);
        let initial_vel = Speed::from_metres_per_second(2.0);

        let mut kalman = KalmanRangeSensor::new(
            RangeSensor::Mock {
                distance: Some(initial_dist),
                velocity: Some(initial_vel),
            },
            0.02, // process variance
            0.15, // measurement variance (high for noisy sensor)
            initial_dist,
            initial_vel,
        );

        let mut test_data_viz = FilterTestData::new();
        test_data_viz.add_step(
            0.0,
            initial_dist.as_metres(),
            kalman.measurement().as_metres(),
            kalman.predicted_measurement().as_metres(),
            kalman.variance(),
        );

        let mut prev_var = kalman.variance();

        for (i, data_point) in test_data.iter().enumerate() {
            kalman.predict_with_dt(data_point.dt).await;
            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.update().await;

            let current_var = kalman.variance();
            debug!(
                "Noisy Step {}: Measured: {:.3}m, Filtered: {:.3}m, Variance: {:.6}",
                i + 1,
                data_point.distance.as_metres(),
                kalman.measurement().as_metres(),
                current_var
            );

            test_data_viz.add_step(
                (i + 1) as f64,
                data_point.distance.as_metres(),
                kalman.measurement().as_metres(),
                kalman.predicted_measurement().as_metres(),
                current_var,
            );

            // Variance should generally decrease (filter converging)
            assert!(
                current_var <= prev_var + 0.01, // small tolerance for process noise
                "Step {}: Variance should not increase significantly",
                i + 1
            );

            prev_var = current_var;
        }

        // Print table for debugging
        test_data_viz.print_table();
        let temp_dir = std::env::temp_dir();
        // Generate plot
        if let Some(strpath) = temp_dir.join("test_plot.html").to_str() {
            match test_data_viz.plot_to_html(strpath) {
                Ok(_) => println!("Plot saved to target/kalman_filter_test.html"),
                Err(e) => eprintln!("Failed to create plot: {}", e),
            }
        }
    }

    #[test]
    fn test_reset_functionality() {
        let initial_dist = Distance::from_metres(1.0);
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
        let new_dist = Distance::from_metres(5.0);
        let new_vel = Speed::from_metres_per_second(0.5);
        kalman.reset(new_dist, new_vel);

        // Verify reset worked
        assert_eq!(kalman.measurement().as_metres(), 5.0);
        assert_eq!(kalman.velocity().as_metres_per_second(), 0.5);
    }
}
