use antaeus::{
    motion::{
        odom::{self, devices::Pose},
        pursuit::{control, geo},
        *,
    },
    to_mutex,
};
use measurements::Length;

use crate::hardware::Robot;
pub async fn main_auton(robot: &mut Robot) {
    let mut path = pursuit::geo::Path::origin();
    path.add(geo::Point::new(Length::from_inches(20.0), Length::from_inches(20.0)));
    path.add(geo::Point::new(Length::from_inches(-20.0), Length::from_inches(20.0)));
    path.add(geo::Point::origin());

    let basic_ctrl = control::basic::BasicControl {
        track_width: Length::from_inches(13.9),
        tolerance:   Length::from_inches(0.5),
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
    let pursuit = pursuit::Pursuit {
        lookahead: Length::from_inches(10.0),
    };
    let _ = pursuit.follow(&odomtrack, &robot.dt, &basic_ctrl, path.clone());
}
