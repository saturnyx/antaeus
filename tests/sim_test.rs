use std::{num::NonZeroU32, time::Duration};

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

pub mod dt;
use dt::SimDrive;

/// Physical constants shared by the PID and odometry simulation.
const WHEEL_DIAMETER_IN: f64 = 3.25;
const TRACK_WIDTH_IN: f64 = 13.0;
const SIMULATION_STEP: Duration = Duration::from_millis(1);
const PID_SETTLE_STEPS: usize = 10_000;
const ODOMETRY_SETTLE_STEPS: usize = 5_000;
const DISTANCE_TOLERANCE_IN: f64 = 0.1;

/// Minimal heading source for odometry integration tests.
///
/// The test updates this value from the differential-drive wheel travel before
/// every odometry tick; it intentionally models a perfect IMU.
#[derive(Default)]
struct SimHeading {
    angle: Angle,
}

impl SimHeading {
    fn new() -> Self { Self::default() }
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

/// Treats the drivetrain's average encoder angle as a forward tracking wheel.
///
/// `SimDrive::clone` shares state, so this tracker observes the same simulated
/// encoder positions that the PID-controlled drivetrain moves.
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

/// A lateral tracking wheel for straight-line differential-drive tests.
///
/// The simulated drivetrain cannot strafe, so its horizontal tracking wheel
/// should remain stationary.
struct StationaryTracker;

impl Trackable for StationaryTracker {
    fn track_position(&mut self) -> Result<Angle, TrackingSensorError> { Ok(Angle::ZERO) }

    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> { Ok(()) }

    fn set_track_position(&mut self, _position: Angle) -> Result<(), TrackingSensorError> { Ok(()) }
}

/// Verifies that both drivetrain PID loops settle at a relative forward target.
#[test]
fn pid_test() {
    let drivetrain = SimDrive::new(200.0 * std::f64::consts::TAU / 60.0);
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
        Length::from_inches(DISTANCE_TOLERANCE_IN),
    );

    pid.set_relative_target(Length::from_inches(10.0), Length::from_inches(10.0));
    for _ in 0..PID_SETTLE_STEPS {
        // Physics advances using the commanded voltage from the preceding PID tick.
        pid.drivetrain.tick(SIMULATION_STEP);
        pid.tick();
    }

    let distance = pid.drivetrain.position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
    assert!((distance - 10.0).abs() <= DISTANCE_TOLERANCE_IN);
}

/// Verifies straight-line odometry using a drivetrain-backed vertical tracker.
#[vexide::test]
async fn odom_test(_peripherals: vexide::prelude::Peripherals) {
    let drivetrain = SimDrive::new(200.0 * std::f64::consts::TAU / 60.0);
    let mut vertical_sensor = drivetrain.clone();
    let mut horizontal_sensor = StationaryTracker;

    let vertical = TrackerPod::new(
        &mut vertical_sensor,
        Length::from_inches(3.25),
        1.0,
        1.0,
        Length::zero(),
    );
    let horizontal = TrackerPod::new(
        &mut horizontal_sensor,
        Length::from_inches(3.25),
        1.0,
        1.0,
        Length::zero(),
    );
    let mechanism = TrackerMech::new(vertical, horizontal, make_cloneable(SimHeading::new()));
    let mut odom = Tracker::new(mechanism);

    let mut pid = DrivePID::new(
        drivetrain,
        0.5,
        0.0,
        0.0,
        12.0,
        Length::from_inches(3.25),
        NonZeroU32::new(1).unwrap(),
        NonZeroU32::new(1).unwrap(),
        Length::from_inches(13.0),
        Length::zero(),
        Length::from_inches(0.1),
    );
    pid.set_relative_target(Length::from_inches(10.0), Length::from_inches(10.0));

    for _ in 0..PID_SETTLE_STEPS {
        pid.drivetrain.tick(SIMULATION_STEP);
        pid.tick();
        odom.tick().unwrap();
    }

    let pose = odom.get_coords();
    assert!(pose.x.abs() <= Length::from_inches(1e-6));
    assert!(
        (pose.y - Length::from_inches(10.0)).abs() <= Length::from_inches(DISTANCE_TOLERANCE_IN)
    );
}

/// Exercises odometry across a forward path with two in-place turns. The
/// simulated IMU is derived from differential wheel travel, so the tracker
/// receives independent wheel and heading measurements just as it would on a
/// robot.
#[vexide::test]
async fn odom_multiple_motions_test(_peripherals: vexide::prelude::Peripherals) {
    let drivetrain = SimDrive::new(200.0 * std::f64::consts::TAU / 60.0);
    let mut vertical_sensor = drivetrain.clone();
    let mut horizontal_sensor = StationaryTracker;
    let imu = make_cloneable(SimHeading::new());

    let vertical = TrackerPod::new(
        &mut vertical_sensor,
        Length::from_inches(WHEEL_DIAMETER_IN),
        1.0,
        1.0,
        Length::zero(),
    );
    let horizontal = TrackerPod::new(
        &mut horizontal_sensor,
        Length::from_inches(WHEEL_DIAMETER_IN),
        1.0,
        1.0,
        Length::zero(),
    );
    let mechanism = TrackerMech::new(vertical, horizontal, imu.clone());
    let mut odom = Tracker::new(mechanism);

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

    // Applies an incremental wheel-distance target. Heading is calculated
    // after physics advances, then supplied to odometry before its tick.
    macro_rules! run_motion {
        ($left_delta_in:expr, $right_delta_in:expr) => {
            pid.set_relative_target(
                Length::from_inches($left_delta_in),
                Length::from_inches($right_delta_in),
            );

            for _ in 0..ODOMETRY_SETTLE_STEPS {
                pid.drivetrain.tick(SIMULATION_STEP);

                // Differential-drive kinematics: right travel greater than left
                // travel is a positive (counter-clockwise) heading change.
                let left_distance =
                    pid.drivetrain.left_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
                let right_distance =
                    pid.drivetrain.right_position().value().as_radians() * WHEEL_DIAMETER_IN / 2.0;
                let heading =
                    Angle::from_radians((right_distance - left_distance) / TRACK_WIDTH_IN);
                imu.borrow_mut().set_heading(heading).unwrap();

                pid.tick();
                odom.tick().unwrap();
            }
        };
    }

    // Forward 10 in, turn counter-clockwise 90°, forward 6 in, turn back to
    // zero heading, then forward another 4 in. In this coordinate system a
    // positive 90° heading rotates forward travel toward negative X.
    let quarter_turn = TRACK_WIDTH_IN * std::f64::consts::FRAC_PI_4;
    run_motion!(10.0, 10.0);
    run_motion!(-quarter_turn, quarter_turn);
    run_motion!(6.0, 6.0);
    run_motion!(quarter_turn, -quarter_turn);
    run_motion!(4.0, 4.0);

    let pose = odom.get_coords();
    // Position tolerance accounts for the PID's 0.1 in settling band on each segment.
    let position_tolerance = Length::from_inches(0.25);
    assert!((pose.x - Length::from_inches(-6.0)).abs() <= position_tolerance);
    assert!((pose.y - Length::from_inches(14.0)).abs() <= position_tolerance);
    assert!(pose.t.as_radians().abs() <= 0.03);
}
