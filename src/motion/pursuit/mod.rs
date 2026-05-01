//! Candidate-Based Pursuit path following algorithm.
//!
//! This module implements a robust variant of the Pure Pursuit algorithm
//! for path following. It handles edge cases that standard pure pursuit
//! struggles with, such as:
//!
//! * Robot starting off the path.
//! * Robot near path endpoints.
//! * Multiple valid lookahead intersections.
//!
//! # Algorithm Overview
//!
//! 1. Draw a circle centered on the robot with radius = lookahead distance.
//! 2. Find all "candidate" points: path waypoints inside the circle,
//!    intersections of the circle with path segments, and the closest
//!    point on the path to the robot.
//! 3. Select the candidate furthest along the path as the target.
//! 4. Drive toward that target using arc movements.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::pursuit::{Pursuit, geo::{Path, Point}};
//!
//! let pursuit = Pursuit { lookahead: 12.0 };
//!
//! let path = Path::from_vec(vec![
//!     Point::new(0.0, 0.0),
//!     Point::new(24.0, 0.0),
//!     Point::new(24.0, 24.0),
//! ]);
//!
//! pursuit.follow(odom, path).await;
//! ```

/// Internal algorithm calculations for path following.
mod algorithm;

/// Geometry primitives for path definition.
///
/// Provides `Point`, `Line`, `Path`, and `Circle` types
/// used by the pursuit algorithm.
pub mod geo;

pub mod control;

const LOOPRATE: Duration = Duration::from_millis(10);

use std::time::Duration;

use control::PursuitControl;
use measurements::Length;
use vexide::time::sleep;

use crate::{
    motion::{
        odom::{devices::Pose, tracker::OdomTracker},
        pursuit::algorithm::abs_arc_point,
    },
    peripherals::drivetrain::Differential,
};
/// Candidate-Based Pursuit path follower.
///
/// Follows a path using the lookahead distance to determine targets.
/// Larger lookahead values result in smoother but less accurate paths.
/// Smaller values track the path more precisely but may cause oscillation.
#[derive(Debug, Clone, Copy)]
pub struct Pursuit {
    /// The lookahead distance in inches.
    ///
    /// This is the radius of the circle used to find target points.
    /// Recommended starting value: 12-18 inches.
    pub lookahead: Length,
}

impl Pursuit {
    /// Creates a new `Pursuit` instance with the specified lookahead distance.
    ///
    /// # Arguments
    ///
    /// * `lookahead` - The lookahead distance in inches, used as the radius for target point calculations.
    pub fn new(lookahead: Length) -> Self { Self { lookahead } }

    /// Follows a path using the Candidate-Based Pursuit algorithm.
    ///
    /// This method continuously calculates target points and commands
    /// arc movements until the robot reaches the end of the path.
    ///
    /// # Arguments
    ///
    /// * `odom` - The odometry movement controller
    /// * `drivetrain` - The differential drivetrain
    /// * `ctrl_algorithm` - The control algorithm
    /// * `path` - The path to follow, defined as a series of waypoints.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let path = Path::from_vec(vec![
    ///     Point::new(0.0, 0.0),
    ///     Point::new(24.0, 12.0),
    ///     Point::new(48.0, 0.0),
    /// ]);
    ///
    /// pursuit.follow(odom, path).await;
    /// ```
    pub async fn follow<C: PursuitControl>(
        &self,
        odom: &OdomTracker,
        drivetrain: &Differential,
        ctrl_algorithm: &C,
        path: geo::Path,
    ) {
        let mut run = true;
        while run {
            let odometry_values = odom.global_pose.lock().await;
            let (x, y, t) = (
                Length::from_inches(odometry_values.x),
                Length::from_inches(odometry_values.y),
                odometry_values.t,
            );
            let cir = geo::Circle {
                x: x.as_inches(),
                y: y.as_inches(),
                r: self.lookahead.as_inches(),
            };
            let target = algorithm::pursuit_target(path.clone(), cir);
            let (tarx, tary) = abs_arc_point(
                Pose::new(Length::as_inches(&x), Length::as_inches(&y), t),
                Length::as_inches(&Length::from_inches(target.x)),
                Length::as_inches(&Length::from_inches(target.y)),
            );
            let ((powl, powr), run_curr) = ctrl_algorithm.control(
                Length::from_inches(tarx),
                Length::from_inches(tary),
                self.lookahead,
            );
            drivetrain.set_left_voltage(powl);
            drivetrain.set_right_voltage(powr);
            run = run_curr;
            sleep(LOOPRATE).await;
        }
    }

    /// Follows a path using the Candidate-Based Pursuit algorithm.
    ///
    /// This method continuously calculates target points and commands
    /// arc movements until the robot reaches the end of the path.
    ///
    /// # Arguments
    ///
    /// * `odom` - The odometry movement controller
    /// * `drivetrain` - The differential drivetrain
    /// * `ctrl_algorithm` - The control algorithm
    /// * `path` - The path to follow, defined as a series of waypoints.
    pub async fn tick<C: PursuitControl>(
        &self,
        odom: &OdomTracker,
        drivetrain: &Differential,
        ctrl_algorithm: &C,
        path: geo::Path,
    ) -> bool {
        let odometry_values = odom.global_pose.lock().await;
        let (x, y, t) = (
            Length::from_inches(odometry_values.x),
            Length::from_inches(odometry_values.y),
            odometry_values.t,
        );
        let cir = geo::Circle {
            x: x.as_inches(),
            y: y.as_inches(),
            r: self.lookahead.as_inches(),
        };
        let target = algorithm::pursuit_target(path.clone(), cir);
        let (tarx, tary) = abs_arc_point(
            Pose::new(Length::as_inches(&x), Length::as_inches(&y), t),
            Length::as_inches(&Length::from_inches(target.x)),
            Length::as_inches(&Length::from_inches(target.y)),
        );
        let ((powl, powr), run_curr) = ctrl_algorithm.control(
            Length::from_inches(tarx),
            Length::from_inches(tary),
            self.lookahead,
        );
        drivetrain.set_left_voltage(powl);
        drivetrain.set_right_voltage(powr);

        run_curr
    }
}
