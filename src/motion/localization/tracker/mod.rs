use std::sync::Arc;

use devices::{Pose, TrackerMech};
use log::warn;
use vexide::{math::Angle, prelude::InertialSensor, sync::Mutex};

use super::Localizer;
use crate::misc::units::Length;

pub mod devices;

pub struct TrackerState {
    pub prev_t: Angle,
    pub prev_v: Length,
    pub prev_h: Length,
}

pub struct Tracker {
    pub tracker_mech: TrackerMech,
    pub pose:         Pose,
    pub state:        TrackerState,
}

impl Tracker {
    pub fn new(tracker_mech: TrackerMech) -> Self {
        Self {
            tracker_mech,
            pose: Pose::origin(),
            state: TrackerState {
                prev_t: Angle::from_radians(0.0),
                prev_v: Length::from_inches(0.0),
                prev_h: Length::from_inches(0.0),
            },
        }
    }

    pub fn from_pose(tracker_mech: TrackerMech, pose: Pose) -> Self {
        Self {
            tracker_mech,
            pose,
            state: TrackerState {
                prev_t: Angle::from_radians(0.0),
                prev_v: Length::from_inches(0.0),
                prev_h: Length::from_inches(0.0),
            },
        }
    }

    pub fn reset_origin(&mut self, t: Option<Angle>, x: Option<Length>, y: Option<Length>) {
        self.pose.t = match t {
            Some(t) => t,
            None => Angle::ZERO,
        };
        self.pose.x = match x {
            Some(x) => x,
            None => Length::from_inches(0.0),
        };
        self.pose.y = match y {
            Some(y) => y,
            None => Length::from_inches(0.0),
        };
    }
}

impl Localizer for Tracker {
    async fn tick(&mut self) {
        // TODO: FIX THIS!!!
        let t = get_imu_angle(&self.tracker_mech.imu).await;
        let v = self.tracker_mech.vertical_tracker.dist().await;
        let h = self.tracker_mech.horizontal_tracker.dist().await;
        let (delta_t, delta_v, delta_h) =
            (t - self.state.prev_t, v - self.state.prev_v, h - self.state.prev_h);
        let delta_pose;
        // A really rare case scenario where the robot moves in an exactly straight line
        if delta_t == Angle::from_radians(0.0) {
            delta_pose = Pose::new(delta_h, delta_v, delta_t);
        } else {
            // Here we go... (somewhat complex math)
            // Equation 6 of http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
            delta_pose = Pose::new(
                local_calc(delta_t, delta_h, self.tracker_mech.horizontal_tracker.offset),
                local_calc(delta_t, delta_v, self.tracker_mech.vertical_tracker.offset),
                delta_t,
            );
        }
        let avg_t = t + (delta_t / 2.0);
        let (global_delta_x, global_delta_y) = rotate_vec(delta_pose.x, delta_pose.y, avg_t);
        self.pose.t = t;
        self.pose.x = self.pose.x + global_delta_x;
        self.pose.y = self.pose.y + global_delta_y;
    }

    fn get_coords(&self) -> Pose { self.pose.clone() }
}

async fn get_imu_angle(imu: &Arc<Mutex<InertialSensor>>) -> Angle {
    imu.lock().await.rotation().unwrap_or_else(|e| {
        warn!("IMU Error: {}", e);
        Angle::from_radians(0.0)
    })
}

fn local_calc(delta_t: Angle, delta_dist: Length, offset: Length) -> Length {
    if delta_t.as_radians().abs() < 1e-10 {
        delta_dist // Straight line motion
    } else {
        2.0 * (delta_t / 2.0).sin() * (delta_dist / delta_t.as_radians() + offset)
    }
}

fn rotate_vec(x: Length, y: Length, t: Angle) -> (Length, Length) {
    let new_x = x * t.cos() - y * t.sin();
    let new_y = x * t.sin() + y * t.cos();
    (new_x, new_y)
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     #[test]
//     fn rotate_vec_test_basic() {
//         let (x, y) = (Length::from_inches(1.0), Length::from_inches(3.0));
//         let (new_x, new_y) = rotate_vec(x, y, Angle::from_degrees(90.0));
//         let expected = (Length::from(-3.0), Length::from(1.0));
//         let tolerance = Length::from_inches(1e-10);
//         if
//         assert!((new_x - expected.0) < tolerance);
//         assert!((new_y - expected.1) < tolerance);
//     }

//     #[test]
//     fn rotate_vec_test_adv() {
//         let (x, y) = (4.65, 7.89);
//         let (new_x, new_y) = rotate_vec(x, y, Angle::from_radians(0.34));
//         let expected = (1.7383, 8.9938);
//         let tolerance = 0.2;
//         eprint!("{}, {}", new_x, new_y);
//         assert!((new_x - expected.0).abs() < tolerance);
//         assert!((new_y - expected.1).abs() < tolerance);
//     }
// }
// Uses crate::misc::units::Length for distance math.
