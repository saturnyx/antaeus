use antaeus::{
    motion::{
        odom::{self, devices::Pose},
        pusuit::geo,
        *,
    },
    to_mutex,
};

use crate::hardware::Robot;
pub async fn main_auton(robot: &mut Robot) {
    let mut path = pusuit::geo::Path::origin();
    path.add(geo::Point::new(20.0, 20.0));
    path.add(geo::Point::new(-20.0, 20.0));
    path.add(geo::Point::new(0.0, 0.0));

    let arcpid_val = pid::arcpid::ArcPIDValues {
        kp:        0.0,
        kd:        0.0,
        tolerance: 0.0,
        maxpwr:    0.0,
        active:    false,
        target:    0.0,
        offset:    0.0,
    };

    let dtc = pid::DrivetrainConfig {
        wheel_diameter: 3.25,
        driving_gear:   3.0,
        driven_gear:    4.0,
        track_width:    13.9,
    };

    let arc_pid = pid::arcpid::ArcPIDMovement {
        drivetrain:        robot.dt.clone(),
        drivetrain_config: dtc,
        arcpid_values:     std::sync::Arc::new(vexide::sync::Mutex::new(arcpid_val)),
    };

    let vertical = odom::devices::Tracker {
        sensor:         odom::devices::TrackingSensor::RotationSensor(robot.v_tracker.clone()),
        offset:         0.0,
        wheel_diameter: 3.25,
        driven_gear:    1.0,
        driving_gear:   1.0,
    };

    let horizontal = odom::devices::Tracker {
        sensor:         odom::devices::TrackingSensor::RotationSensor(robot.h_tracker.clone()),
        offset:         0.0,
        wheel_diameter: 3.25,
        driven_gear:    1.0,
        driving_gear:   1.0,
    };

    let trackers = odom::devices::TrackerMech::new(vertical, horizontal, robot.imu.clone());

    let odomtrack = odom::tracker::OdomTracker {
        trackermech: trackers,
        global_pose: to_mutex(Pose::origin()),
    };
    let pursuit = pusuit::Pursuit { lookahead: 10.0 };
    let _ = pursuit.follow(&odomtrack, &arc_pid, path.clone());
    let _ = pursuit.follow(&odomtrack, &arc_pid, path);
}
