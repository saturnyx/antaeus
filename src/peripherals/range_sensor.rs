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
pub enum RangeSensor {
    SmartDistSensor(Arc<Mutex<DistanceSensor>>),
    AdiDistanceSensor(Arc<Mutex<AdiRangeFinder>>),
    #[cfg(any(test, debug_assertions))]
    Mock {
        distance: Option<Distance>,
        velocity: Option<Speed>,
    },
}

impl RangeSensor {
    pub fn from_distance(sensor: DistanceSensor) -> Self {
        Self::SmartDistSensor(Arc::new(Mutex::new(sensor)))
    }

    pub fn from_adi(sensor: AdiRangeFinder) -> Self {
        Self::AdiDistanceSensor(Arc::new(Mutex::new(sensor)))
    }

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

    pub async fn predict(&mut self) {
        let elapsed = user_uptime() - self.last_update;
        let dx = self.prev_vel * elapsed;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update = user_uptime();
    }

    /// For testing: predict with explicit time step
    #[cfg(any(test, debug_assertions))]
    pub async fn predict_with_dt(&mut self, dt: Duration) {
        let dx = self.prev_vel * dt;
        self.est_m = self.prev_m + dx;
        self.est_var = self.prev_var + self.process_var;
        self.last_update = self.last_update + dt;
    }

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

    pub fn measurement(&self) -> Distance { self.new_m }

    pub fn variance(&self) -> f64 { self.new_var }

    pub fn velocity(&self) -> Speed { self.prev_vel }

    pub fn predicted_measurement(&self) -> Distance { self.est_m }

    pub fn predicted_variance(&self) -> f64 { self.est_var }

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

    /// For testing: update the sensor with new mock data
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
            plot.add_trace(filtered);
            plot.add_trace(predicted);

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
    async fn test_kalman_filter_with_data_sequence(_p: Peripherals) {
        // Simulate object moving at constant 1 m/s
        let test_data = vec![
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

        debug!("=== Kalman Filter Test ===");
        debug!(
            "Initial state - Distance: {:.3}m, Variance: {:.6}",
            kalman.measurement().as_metres(),
            kalman.variance()
        );

        for (i, data_point) in test_data.iter().enumerate() {
            // Step 1: Predict next state
            kalman.predict_with_dt(data_point.dt).await;
            let predicted_dist = kalman.predicted_measurement().as_metres();
            let predicted_var = kalman.predicted_variance();

            debug!(
                "Step {}: After predict - Distance: {:.3}m, Variance: {:.6}",
                i + 1,
                predicted_dist,
                predicted_var
            );

            // Step 2: Update with new measurement
            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.update().await;
            let updated_dist = kalman.measurement().as_metres();
            let updated_var = kalman.variance();

            debug!(
                "Step {}: After update - Distance: {:.3}m, Variance: {:.6}",
                i + 1,
                updated_dist,
                updated_var
            );

            // Assertions
            assert!(
                updated_dist > 0.0,
                "Step {}: Distance should be positive, got {:.3}m",
                i + 1,
                updated_dist
            );

            assert!(
                updated_var < predicted_var,
                "Step {}: Variance should decrease after update ({:.6} -> {:.6})",
                i + 1,
                predicted_var,
                updated_var
            );

            assert!(
                updated_var > 0.0,
                "Step {}: Variance should be positive, got {:.6}",
                i + 1,
                updated_var
            );

            // Distance should be reasonable (between predicted and measured)
            let min_dist = predicted_dist.min(data_point.distance.as_metres());
            let max_dist = predicted_dist.max(data_point.distance.as_metres());
            assert!(
                updated_dist >= min_dist - 0.01 && updated_dist <= max_dist + 0.01,
                "Step {}: Updated distance {:.3}m should be between prediction {:.3}m and \
                 measurement {:.3}m",
                i + 1,
                updated_dist,
                predicted_dist,
                data_point.distance.as_metres()
            );
        }

        debug!("=== Test Complete ===");
    }

    #[vexide::test]
    #[ignore = "manual verification needed (graph)"]
    async fn test_kalman_with_noisy_measurements_visualized(_p: Peripherals) {
        // Simulate noisy sensor with linear motion (2 m/s) with significant noise
        // The object moves in a straight line but measurements oscillate around the true path
        let test_data = vec![
            // Start at 2.0m, moving at 2.0 m/s
            TestDataPoint {
                distance: Distance::from_metres(2.0),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.18),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.08),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.25),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.12),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 2.25m
            TestDataPoint {
                distance: Distance::from_metres(2.35),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.20),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.40),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.26),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.38),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 2.50m
            TestDataPoint {
                distance: Distance::from_metres(2.55),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.48),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.68),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.52),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.72),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 2.75m
            TestDataPoint {
                distance: Distance::from_metres(2.60),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.80),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.70),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.88),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.75),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 3.00m
            TestDataPoint {
                distance: Distance::from_metres(3.05),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.95),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.15),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.02),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.18),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 3.25m
            TestDataPoint {
                distance: Distance::from_metres(3.12),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.28),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.18),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.32),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.22),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            // 3.50m - settle a bit
            TestDataPoint {
                distance: Distance::from_metres(3.50),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.48),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.55),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.52),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.60),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.58),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.63),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(3.62),
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

        // Generate plot
        match test_data_viz.plot_to_html("target/kalman_filter_test.html") {
            Ok(_) => println!("Plot saved to target/kalman_filter_test.html"),
            Err(e) => eprintln!("Failed to create plot: {}", e),
        }
    }

    #[vexide::test]
    async fn test_kalman_with_noisy_measurements(_p: Peripherals) {
        // Simulate noisy sensor with constant 2 m/s motion
        let test_data = vec![
            TestDataPoint {
                distance: Distance::from_metres(2.0),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            },
            TestDataPoint {
                distance: Distance::from_metres(2.15),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            }, // slightly off
            TestDataPoint {
                distance: Distance::from_metres(2.05),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            }, // noisy
            TestDataPoint {
                distance: Distance::from_metres(2.25),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            }, // noisy
            TestDataPoint {
                distance: Distance::from_metres(2.20),
                velocity: Speed::from_metres_per_second(2.0),
                dt:       Duration::from_millis(50),
            }, // converging
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

        let mut prev_var = kalman.variance();

        for (i, data_point) in test_data.iter().enumerate() {
            kalman.predict_with_dt(data_point.dt).await;
            kalman.set_sensor_mock(data_point.distance, data_point.velocity);
            kalman.update().await;

            let current_var = kalman.variance();
            debug!(
                "Noisy Step {}: Distance: {:.3}m, Variance: {:.6}",
                i + 1,
                kalman.measurement().as_metres(),
                current_var
            );

            // Variance should generally decrease (filter converging)
            assert!(
                current_var <= prev_var + 0.01, // small tolerance for process noise
                "Step {}: Variance should not increase significantly",
                i + 1
            );

            prev_var = current_var;
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
