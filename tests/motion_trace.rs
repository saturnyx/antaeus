//! Integration test that records a simulated PID and odometry run to CSV.
//!
//! Run explicitly because this test creates `test-artifacts/test-traces/odometry_motion_trace.csv`:
//! `cargo test --test motion_trace -- --include-ignored`

use std::{
    fs::{self, File},
    num::NonZeroU32,
    time::Duration,
};

use antaeus::{
    make_cloneable,
    motion::{
        feedback_control::pid::drive_pid::DrivePID,
        localization::{
            Localizer,
            tracker::{
                Tracker,
                devices::{HeadingSensor, Trackable, TrackerMech, TrackerPod, TrackingSensorError},
            },
        },
    },
    peripherals::drivetrain::Differential,
    utils::units::Length,
};
use vexide::math::Angle;

mod dt;
mod trace_support;
use dt::SimDrive;
use trace_support::{TraceSample, write_trace_plot_html};

const WHEEL_DIAMETER_IN: f64 = 3.25;
const TRACK_WIDTH_IN: f64 = 13.0;
const SIMULATION_STEP: Duration = Duration::from_millis(1);
const STEP_SECONDS: f64 = 0.001;
const STEPS_PER_SEGMENT: usize = 5_000;
const TRACE_PATH: &str = "test-artifacts/test-traces/odometry_motion_trace.csv";
const PLOT_PATH: &str = "test-artifacts/test-traces/odometry_motion_trace.html";

/// A perfect simulated IMU. Its angle is updated from the differential-drive
/// wheel distances before every odometry update.
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

/// Lets a cloned `SimDrive` act as a forward tracking wheel. Clones share all
/// simulated state, so this reports the same average encoder position used by
/// the PID-controlled drivetrain.
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

/// A differential drive cannot strafe, so its lateral tracking wheel is still.
struct StationaryTracker;

impl Trackable for StationaryTracker {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> { Ok(Angle::ZERO) }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> { Ok(()) }

    fn set_track_position(&mut self, _position: Angle) -> Result<(), TrackingSensorError> { Ok(()) }
}

/// Records one sample in a stable, viewer-friendly CSV schema.
#[allow(clippy::too_many_arguments)]
fn write_sample(
    writer: &mut csv::Writer<File>,
    step: usize,
    segment: &str,
    pid: &DrivePID<SimDrive>,
    pose_x_in: f64,
    pose_y_in: f64,
    pose_heading_rad: f64,
    imu_heading_rad: f64,
) {
    let left_distance_in =
        pid.drivetrain.left_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
    let right_distance_in =
        pid.drivetrain.right_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;

    writer
        .serialize((
            step,
            step as f64 * STEP_SECONDS,
            segment,
            pid.pid_left.target,
            pid.pid_right.target,
            left_distance_in,
            right_distance_in,
            imu_heading_rad,
            pose_x_in,
            pose_y_in,
            pose_heading_rad,
        ))
        .expect("write trace sample");
}

/// Logs a mixed travel-and-turn run for inspection in `tools/motion_trace_viewer.html`.
///
/// The path is forward 10 in, left 90°, forward 6 in, right 90°, and forward
/// 4 in. Its ideal final pose is approximately `(-6 in, 14 in, 0 rad)`.
#[vexide::test]
#[ignore = "writes a simulation trace; run explicitly for visualization"]
async fn logs_multi_motion_odometry_trace(_peripherals: vexide::prelude::Peripherals) {
    fs::create_dir_all("test-artifacts/test-traces").expect("create trace directory");
    let trace_file = File::create(TRACE_PATH).expect("create trace file");
    let mut writer = csv::Writer::from_writer(trace_file);
    writer
        .write_record([
            "step",
            "time_s",
            "segment",
            "left_target_in",
            "right_target_in",
            "left_distance_in",
            "right_distance_in",
            "imu_heading_rad",
            "pose_x_in",
            "pose_y_in",
            "pose_heading_rad",
        ])
        .expect("write trace header");

    let mut samples = Vec::new();

    let drivetrain = SimDrive::new(200.0 * std::f64::consts::TAU / 60.0);
    let mut vertical_sensor = drivetrain.clone();
    let mut horizontal_sensor = StationaryTracker;
    let imu = make_cloneable(SimHeading::default());

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

    let mut pid = DrivePID::new(
        drivetrain,
        0.5,
        0.0,
        0.0,
        12.0,
        Length::from_inches(WHEEL_DIAMETER_IN),
        NonZeroU32::new(1).unwrap(),
        NonZeroU32::new(1).unwrap(),
        Length::from_inches(TRACK_WIDTH_IN),
        Length::zero(),
        Length::from_inches(0.1),
    );

    let quarter_turn_distance = TRACK_WIDTH_IN * std::f64::consts::FRAC_PI_4;
    let motions = [
        ("forward_10", 10.0, 10.0),
        ("turn_left_90", -quarter_turn_distance, quarter_turn_distance),
        ("forward_6", 6.0, 6.0),
        ("turn_right_90", quarter_turn_distance, -quarter_turn_distance),
        ("forward_4", 4.0, 4.0),
    ];

    let mut step = 0;
    for (segment, left_delta_in, right_delta_in) in motions {
        pid.set_relative_target(
            Length::from_inches(left_delta_in),
            Length::from_inches(right_delta_in),
        );

        for _ in 0..STEPS_PER_SEGMENT {
            // Advance plant state using the preceding iteration's PID output.
            pid.drivetrain.tick(SIMULATION_STEP);

            // Perfect IMU model derived from the two wheel distances.
            let left_distance_in =
                pid.drivetrain.left_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
            let right_distance_in =
                pid.drivetrain.right_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
            let imu_heading_rad = (right_distance_in - left_distance_in) / TRACK_WIDTH_IN;
            imu.borrow_mut()
                .set_heading(Angle::from_radians(imu_heading_rad))
                .expect("update simulated IMU");

            pid.tick();
            odometry.tick().expect("update odometry");

            let pose = odometry.get_coords();
            write_sample(
                &mut writer,
                step,
                segment,
                &pid,
                pose.x.as_inches(),
                pose.y.as_inches(),
                pose.t.as_radians(),
                imu_heading_rad,
            );
            samples.push(TraceSample {
                step,
                time_s: step as f64 * STEP_SECONDS,
                segment: segment.to_string(),
                pose_x_in: pose.x.as_inches(),
                pose_y_in: pose.y.as_inches(),
                heading_rad: pose.t.as_radians(),
            });
            step += 1;
        }
    }

    writer.flush().expect("flush trace");
    write_trace_plot_html(
        &samples,
        &[(0.0, 0.0), (0.0, 10.0), (-6.0, 10.0), (-6.0, 14.0)],
        PLOT_PATH,
        "Odometry Multi-Motion Trace",
    );

    let final_pose = odometry.get_coords();
    let tolerance = Length::from_inches(0.25);
    assert!((final_pose.x - Length::from_inches(-6.0)).abs() <= tolerance);
    assert!((final_pose.y - Length::from_inches(14.0)).abs() <= tolerance);
    assert!(final_pose.t.as_radians().abs() <= 0.03);

    println!("motion trace written to {TRACE_PATH}");
    println!("motion Plotly HTML written to {PLOT_PATH}");
}
