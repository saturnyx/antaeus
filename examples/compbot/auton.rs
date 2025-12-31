use antaeus::motion::{pusuit::geo, *};

use crate::hardware::Robot;
pub fn main_auton(robot: &mut Robot) {
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

    let pid = pid::arcpid::ArcPIDMovement {
        drivetrain:        robot.dt.clone(),
        drivetrain_config: dtc,
        arcpid_values:     std::sync::Arc::new(vexide::sync::Mutex::new(arcpid_val)),
    };

    let vertical = odom::WheelTracker {
        device:         odom::TrackingDevice::RotationSensor(robot.v_tracker.clone()),
        offset:         0.0,
        wheel_diameter: 3.25,
        reverse:        false,
    };

    let horizontal = odom::WheelTracker {
        device:         odom::TrackingDevice::RotationSensor(robot.h_tracker.clone()),
        offset:         0.0,
        wheel_diameter: 3.25,
        reverse:        false,
    };

    let trackers = odom::Trackers {
        horizontal: horizontal,
        vertical:   vertical,
        imu:        robot.imu.clone(),
    };

    let odom_values = odom::OdomValues {
        global_x:       0.0,
        global_y:       0.0,
        global_heading: 0.0,
    };

    let odom = odom::OdomMovement {
        odometry_values: std::sync::Arc::new(vexide::sync::Mutex::new(odom_values)),
        trackers:        trackers,
        pid:             None,
        arc_pid:         Some(pid),
    };

    let pursuit = pusuit::Pursuit { lookahead: 10.0 };
    let _ = pursuit.follow(odom, path);
}
