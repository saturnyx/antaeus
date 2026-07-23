use std::time::Duration;

use antaeus::{
    peripherals::range_sensor::{KalmanRangeSensor, RangeSensor},
    utils::units::{Length, Speed},
};
use rand::rng;
use rand_distr::{Distribution, Normal};
use vexide::prelude::Peripherals;

struct TestDataPoint {
    distance: Length,
    velocity: Speed,
    dt:       Duration,
}

struct FilterTestData {
    time_steps:   Vec<f64>,
    measurements: Vec<f64>,
    filtered:     Vec<f64>,
    predicted:    Vec<f64>,
    variance:     Vec<f64>,
}

fn generate_random_test_data(
    num_samples: usize,
    start_distance: f64,
    velocity: f64,
    dt_ms: u64,
    noise_amplitude: f64,
) -> Vec<TestDataPoint> {
    let mut rng = rng();
    let mut data = Vec::with_capacity(num_samples);
    let dt = Duration::from_millis(dt_ms);
    let dt_secs = dt_ms as f64 / 1000.0;
    let normal = Normal::new(0.0, noise_amplitude).expect("failed to create normal distribution");

    for i in 0..num_samples {
        let true_distance = start_distance + velocity * (i as f64) * dt_secs;
        let measurement = true_distance + normal.sample(&mut rng);

        data.push(TestDataPoint {
            distance: Length::from_metres(measurement.max(0.0)),
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

    fn plot_to_html(&self, filename: &str) {
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
                .title(plotly::common::Title::with_text(
                    "Kalman Filter: Raw vs Filtered Data",
                ))
                .x_axis(
                    plotly::layout::Axis::new()
                        .title(plotly::common::Title::with_text("Time (steps)")),
                )
                .y_axis(
                    plotly::layout::Axis::new()
                        .title(plotly::common::Title::with_text("Length (m)")),
                ),
        );

        plot.write_html(filename);
    }

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
#[ignore = "manual verification needed (graph)"]
async fn test_kalman_filter_visualization(_p: Peripherals) {
    let test_data = generate_random_test_data(
        100,  // samples
        2.0,  // start distance
        2.0,  // velocity
        50,   // dt (ms)
        0.15, // gaussian noise std-dev
    );

    let initial_dist = Length::from_metres(2.0);
    let initial_vel = Speed::from_metres_per_second(2.0);

    let mut kalman = KalmanRangeSensor::new(
        RangeSensor::Mock {
            distance: Some(initial_dist),
            velocity: Some(initial_vel),
        },
        0.02, // process variance
        0.15, // measurement variance
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

        test_data_viz.add_step(
            (i + 1) as f64,
            data_point.distance.as_metres(),
            kalman.measurement().as_metres(),
            kalman.predicted_measurement().as_metres(),
            current_var,
        );

        assert!(
            current_var <= prev_var + 0.01,
            "Step {}: Variance should not increase significantly",
            i + 1
        );

        prev_var = current_var;
    }

    test_data_viz.print_table();

    std::fs::create_dir_all("test-artifacts/test-plots").expect("create plot dir failed");
    let out_file = "test-artifacts/test-plots/kalman_filter_test.html";
    test_data_viz.plot_to_html(out_file);

    println!("Plot saved to {out_file}");
}
