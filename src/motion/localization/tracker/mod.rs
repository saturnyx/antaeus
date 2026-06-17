//! Tracking-wheel localization.
//!
//! Implements a `Localizer` that fuses two tracking wheels (vertical/horizontal)
//! with an IMU heading to integrate robot pose over time. Handles small-angle
//! straight-line motion as well as arc-based deltas during rotation.
//!
//! The `devices` submodule defines the hardware wrapper (`TrackerMech`) for
//! sensors. `Tracker` maintains the current pose and previous sensor readings
//! to compute incremental updates in `tick`.

use devices::TrackerMech;
use vexide::math::Angle;

use super::Localizer;
use crate::{
    motion::localization::tracker::devices::TrackingSensorError,
    utils::{geo::Pose, units::Length},
};

pub mod devices;

const ANGLE_EPS_RAD: f64 = 1e-6;

/// State for the `Tracker` localizer, storing previous sensor readings for
/// delta calculations.
pub struct TrackerState {
    /// Previous Angle
    pub prev_t: Angle,
    /// Previous vertical tracking wheel distance
    pub prev_v: Length,
    /// Previous horizontal tracking wheel distance
    pub prev_h: Length,
}

/// Localization controller that fuses two tracking wheels and an IMU to
/// estimate the robot's position and orientation.
pub struct Tracker {
    /// The tracking mechanism containing the sensors and their configurations.
    pub tracker_mech: TrackerMech,
    /// The current estimated pose of the robot (x, y in inches, t in radians).
    pub pose:         Pose,
    /// The internal state storing previous sensor readings for delta
    /// calculations.
    pub state:        TrackerState,
}

impl Tracker {
    /// Creates a new `Tracker` instance with the specified mechanism and an
    /// initial pose at the origin (0, 0, 0).
    pub fn new(tracker_mech: TrackerMech) -> Self {
        Self {
            tracker_mech,
            pose: Pose::origin(),
            state: TrackerState {
                prev_t: Angle::ZERO,
                prev_v: Length::zero(),
                prev_h: Length::zero(),
            },
        }
    }

    /// Creates a new `Tracker` instance with the specified mechanism and an
    /// initial pose.
    pub fn from_pose(tracker_mech: TrackerMech, pose: Pose) -> Self {
        Self {
            tracker_mech,
            pose,
            state: TrackerState {
                prev_t: Angle::ZERO,
                prev_v: Length::zero(),
                prev_h: Length::zero(),
            },
        }
    }

    /// Resets the tracker's pose to the specified values (or origin if None)
    /// and synchronizes the previous sensor readings to prevent jumps in the
    /// next tick.
    pub async fn reset_origin(
        &mut self,
        t: Option<Angle>,
        x: Option<Length>,
        y: Option<Length>,
    ) -> Result<(), TrackingSensorError> {
        self.pose.t = t.unwrap_or(Angle::ZERO);
        self.pose.x = x.unwrap_or_else(Length::zero);
        self.pose.y = y.unwrap_or_else(Length::zero);

        let imu_t = self.tracker_mech.imu.lock().await.rotation()?;
        let v = self.tracker_mech.vertical_tracker.dist().await?;
        let h = self.tracker_mech.horizontal_tracker.dist().await?;

        self.state.prev_t = imu_t;
        self.state.prev_v = v;
        self.state.prev_h = h;

        Ok(())
    }
}

impl Localizer<TrackingSensorError> for Tracker {
    async fn tick(&mut self) -> Result<(), TrackingSensorError> {
        let t = self.tracker_mech.imu.lock().await.rotation()?;
        let v = self.tracker_mech.vertical_tracker.dist().await?;
        let h = self.tracker_mech.horizontal_tracker.dist().await?;

        let delta_t = t - self.state.prev_t;
        let delta_v = v - self.state.prev_v;
        let delta_h = h - self.state.prev_h;

        let delta_pose = if is_small_angle(delta_t) {
            Pose::new(delta_h, delta_v, delta_t)
        } else {
            Pose::new(
                local_arc_delta(delta_t, delta_h, self.tracker_mech.horizontal_tracker.offset),
                local_arc_delta(delta_t, delta_v, self.tracker_mech.vertical_tracker.offset),
                delta_t,
            )
        };

        let avg_t = self.state.prev_t + (delta_t / 2.0);
        let (global_delta_x, global_delta_y) = rotate_vec(delta_pose.x, delta_pose.y, avg_t);

        self.pose.t = t;
        self.pose.x = self.pose.x + global_delta_x;
        self.pose.y = self.pose.y + global_delta_y;

        self.state.prev_t = t;
        self.state.prev_v = v;
        self.state.prev_h = h;

        Ok(())
    }

    fn get_coords(&self) -> Pose { self.pose }
}

fn is_small_angle(delta_t: Angle) -> bool { delta_t.as_radians().abs() < ANGLE_EPS_RAD }

fn local_arc_delta(delta_t: Angle, delta_dist: Length, offset: Length) -> Length {
    let dt = delta_t.as_radians();
    if dt.abs() < ANGLE_EPS_RAD {
        delta_dist
    } else {
        2.0 * (delta_t / 2.0).sin() * (delta_dist / dt + offset)
    }
}

fn rotate_vec(x: Length, y: Length, t: Angle) -> (Length, Length) {
    let new_x = x * t.cos() - y * t.sin();
    let new_y = x * t.sin() + y * t.cos();
    (new_x, new_y)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rotate_vec_test_basic() {
        let (x, y) = (1.0, 3.0);
        let (new_x, new_y) = rotate_vec(
            Length::from_inches(x),
            Length::from_inches(y),
            Angle::from_degrees(90.0),
        );
        let expected = (Length::from_inches(-3.0), Length::from_inches(1.0));
        let tolerance = Length::from_inches(1e-10);
        assert!((new_x - expected.0).abs() < tolerance);
        assert!((new_y - expected.1).abs() < tolerance);
    }

    #[test]
    fn rotate_vec_test_adv() {
        let (x, y) = (4.65, 7.89);
        let (new_x, new_y) = rotate_vec(
            Length::from_inches(x),
            Length::from_inches(y),
            Angle::from_radians(0.34),
        );
        let expected = (Length::from_inches(1.7383), Length::from_inches(8.9938));
        let tolerance = Length::from_inches(0.02);
        assert!((new_x - expected.0).abs() < tolerance);
        assert!((new_y - expected.1).abs() < tolerance);
    }

    #[test]
    fn local_arc_delta_straight_line_epsilon() {
        let delta_t = Angle::from_radians(ANGLE_EPS_RAD / 10.0);
        let delta_dist = Length::from_inches(5.0);
        let offset = Length::from_inches(2.0);

        let result = local_arc_delta(delta_t, delta_dist, offset);
        let tolerance = Length::from_inches(1e-9);

        assert!((result - delta_dist).abs() < tolerance);
    }
}
