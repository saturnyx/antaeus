//! Integration test that logs a simulated Candidate-Based Pursuit run to CSV.
//!
//! Run explicitly to create `test-artifacts/test-traces/pursuit_motion_trace.csv`:
//! `cargo test --test pursuit_trace -- --include-ignored`
//!
//! Open the generated CSV with `tools/motion_trace_viewer.html`.

use std::{
    fs::{self, File},
    sync::Arc,
    time::Duration,
};

use antaeus::{
    motion::{
        localization::{
            Localizer,
            tracker::{
                Tracker,
                devices::{HeadingSensor, Trackable, TrackerMech, TrackerPod, TrackingSensorError},
            },
        },
        pursuit::{Pursuit, control::basic::BasicControl},
    },
    peripherals::drivetrain::Differential,
    utils::{
        geo::{Path, Point},
        units::Length,
    },
};
use vexide::{math::Angle, sync::Mutex};

mod dt;
mod trace_support;
use dt::SimDrive;
use trace_support::{TraceSample, write_trace_plot_html};

const WHEEL_DIAMETER_IN: f64 = 3.25;
const TRACK_WIDTH_IN: f64 = 13.0;
const SIMULATION_STEP: Duration = Duration::from_millis(10);
const STEP_SECONDS: f64 = 0.01;
const MAX_STEPS: usize = 10_000;
const TRACE_PATH: &str = "test-artifacts/test-traces/pursuit_motion_trace.csv";
const PLOT_PATH: &str = "test-artifacts/test-traces/pursuit_motion_trace.html";

/// Perfect simulated IMU updated from differential-drive wheel distances.
#[derive(Default)]
struct SimHeading {
    angle: Angle,
}

impl HeadingSensor for SimHeading {
    fn heading(&mut self) -> Result<Angle, TrackingSensorError> { Ok(self.angle) }

    fn reset_heading(&mut self) -> Result<(), TrackingSensorError> {
        self.angle = Angle::ZERO;
        Ok(())
    }

    fn set_heading(&mut self, heading: Angle) -> Result<(), TrackingSensorError> {
        self.angle = heading;
        Ok(())
    }
}

/// Reports the average drivetrain encoder angle as the forward tracking wheel.
impl Trackable for SimDrive {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> {
        Ok(self.position().value())
    }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> {
        self.reset_position().map_err(TrackingSensorError::from)
    }

    fn set_track_position(&mut self, position: Angle) -> Result<(), TrackingSensorError> {
        self.set_position(position)
            .map_err(TrackingSensorError::from)
    }
}

/// A differential-drive simulation has no lateral tracking-wheel displacement.
struct StationaryTracker;

impl Trackable for StationaryTracker {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> { Ok(Angle::ZERO) }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> { Ok(()) }

    fn set_track_position(&mut self, _position: Angle) -> Result<(), TrackingSensorError> { Ok(()) }
}

fn wheel_distance_in(angle: Angle) -> f64 { angle.as_radians() * WHEEL_DIAMETER_IN / 2.0 }

/// Writes the pose and wheel command state for a single pursuit control cycle.
fn write_sample(
    writer: &mut csv::Writer<File>,
    step: usize,
    pid: &SimDrive,
    pose_x_in: f64,
    pose_y_in: f64,
    pose_heading_rad: f64,
    imu_heading_rad: f64,
    path_point: Option<Point>,
) {
    writer
        .serialize((
            step,
            step as f64 * STEP_SECONDS,
            "candidate_based_pursuit",
            wheel_distance_in(pid.left_position().value()),
            wheel_distance_in(pid.right_position().value()),
            imu_heading_rad,
            pose_x_in,
            pose_y_in,
            pose_heading_rad,
            path_point.map(|point| point.x),
            path_point.map(|point| point.y),
        ))
        .expect("write pursuit trace sample");
}

/// Follows a gentle diagonal path using Candidate-Based Pursuit and records
/// the full odometry trace for the browser viewer.
///
/// The manual physics/IMU update occurs before `Pursuit::tick()`: it applies
/// the wheel voltages from the preceding pursuit cycle, then gives pursuit a
/// current odometry pose from which to select its next lookahead target.
#[vexide::test]
#[ignore = "writes a pursuit trace; run explicitly for visualization"]
async fn logs_candidate_based_pursuit_trace(_peripherals: vexide::prelude::Peripherals) {
    fs::create_dir_all("test-artifacts/test-traces").expect("create trace directory");
    let mut writer = csv::Writer::from_writer(File::create(TRACE_PATH).expect("create trace file"));
    writer
        .write_record([
            "step",
            "time_s",
            "segment",
            "left_distance_in",
            "right_distance_in",
            "imu_heading_rad",
            "pose_x_in",
            "pose_y_in",
            "pose_heading_rad",
            "path_x_in",
            "path_y_in",
        ])
        .expect("write trace header");

    // A single long diagonal segment produces a smooth steering arc without
    // introducing an unvalidated sharp-corner transition.
    let path = Path::from_vec(vec![
        Point::origin(),
        Point::new(Length::from_inches(18.0), Length::from_inches(84.0)),
        Point::new(Length::from_inches(50.0), Length::from_inches(56.0)),
        Point::new(Length::from_inches(20.0), Length::from_inches(42.0)),
        Point::new(Length::from_inches(64.0), Length::from_inches(10.0)),
    ]);
    let final_waypoint = *path.waypoints.last().expect("path has a final waypoint");

    let mut samples = Vec::new();

    let drivetrain = SimDrive::new(200.0 * std::f64::consts::TAU / 60.0);
    let mut vertical_sensor = drivetrain.clone();
    let mut horizontal_sensor = StationaryTracker;
    let imu = Arc::new(Mutex::new(SimHeading::default()));
    let vertical_tracker = TrackerPod::new(
        &mut vertical_sensor,
        Length::from_inches(WHEEL_DIAMETER_IN),
        1.0,
        1.0,
        Length::zero(),
    );
    let horizontal_tracker = TrackerPod::new(
        &mut horizontal_sensor,
        Length::from_inches(WHEEL_DIAMETER_IN),
        1.0,
        1.0,
        Length::zero(),
    );
    let mut odometry =
        Tracker::new(TrackerMech::new(vertical_tracker, horizontal_tracker, imu.clone()));

    let pursuit = Pursuit::new(Length::from_inches(4.0));
    let controller =
        BasicControl::new(Length::from_inches(TRACK_WIDTH_IN), Length::from_inches(0.75));

    let mut completed = false;
    for step in 0..MAX_STEPS {
        // Write each planned waypoint once. Later rows leave these optional
        // fields empty while the viewer retains the complete underlay.
        // Simulate one plant timestep with the voltages commanded by the
        // previous Candidate-Based Pursuit update.
        drivetrain.tick(SIMULATION_STEP);

        let left_distance_in = wheel_distance_in(drivetrain.left_position().value());
        let right_distance_in = wheel_distance_in(drivetrain.right_position().value());
        let imu_heading_rad = (right_distance_in - left_distance_in) / TRACK_WIDTH_IN;
        imu.lock()
            .await
            .set_heading(Angle::from_radians(imu_heading_rad))
            .expect("update simulated IMU");

        // Pursuit selects a lookahead target, commands wheel voltages, and
        // updates odometry from the sensor state established above.
        let should_continue = pursuit
            .tick(&mut odometry, &drivetrain, &controller, path.clone())
            .await
            .expect("run pursuit control cycle");

        let pose = odometry.get_coords();
        write_sample(
            &mut writer,
            step,
            &drivetrain,
            pose.x.as_inches(),
            pose.y.as_inches(),
            pose.t.as_radians(),
            imu_heading_rad,
            path.waypoints.get(step).copied(),
        );
        samples.push(TraceSample {
            step,
            time_s: step as f64 * STEP_SECONDS,
            segment: "candidate_based_pursuit".to_string(),
            pose_x_in: pose.x.as_inches(),
            pose_y_in: pose.y.as_inches(),
            heading_rad: pose.t.as_radians(),
        });

        if !should_continue {
            completed = true;
            break;
        }
    }
    writer.flush().expect("flush trace");
    write_trace_plot_html(
        &samples,
        &path
            .waypoints
            .iter()
            .map(|point| (point.x, point.y))
            .collect::<Vec<_>>(),
        PLOT_PATH,
        "Candidate-Based Pursuit Trace",
    );

    let final_pose = odometry.get_coords();
    let tolerance = Length::from_inches(1.0);
    assert!(completed, "pursuit did not settle within {MAX_STEPS} steps");
    assert!((final_pose.x - Length::from_inches(final_waypoint.x)).abs() <= tolerance);
    assert!((final_pose.y - Length::from_inches(final_waypoint.y)).abs() <= tolerance);

    println!("pursuit trace written to {TRACE_PATH}");
    println!("pursuit Plotly HTML written to {PLOT_PATH}");
}
