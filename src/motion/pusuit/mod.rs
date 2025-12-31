//! Candidate-Based Pursuit path following algorithm.
//!
//! This module implements a robust variant of the Pure Pursuit algorithm
//! for path following. It handles edge cases that standard pure pursuit
//! struggles with, such as:
//!
//! - Robot starting off the path.
//! - Robot near path endpoints.
//! - Multiple valid lookahead intersections.
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
//! use antaeus::motion::pusuit::{Pursuit, geo::{Path, Point}};
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

use crate::motion::odom::OdomMovement;

/// Candidate-Based Pursuit path follower.
///
/// Follows a path using the lookahead distance to determine targets.
/// Larger lookahead values result in smoother but less accurate paths.
/// Smaller values track the path more precisely but may cause oscillation.
pub struct Pursuit {
    /// The lookahead distance in inches.
    ///
    /// This is the radius of the circle used to find target points.
    /// Recommended starting value: 12-18 inches.
    pub lookahead: f64,
}

impl Pursuit {
    /// Creates a new `Pursuit` instance with the specified lookahead distance.
    ///
    /// # Arguments
    ///
    /// * `lookahead` - The lookahead distance in inches, used as the radius for target point calculations.
    pub fn new(lookahead: f64) -> Self { Self { lookahead } }

    /// Follows a path using the Candidate-Based Pursuit algorithm.
    ///
    /// This method continuously calculates target points and commands
    /// arc movements until the robot reaches the end of the path.
    ///
    /// # Arguments
    ///
    /// * `odom` - The odometry movement controller (must have arc_pid configured).
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
    pub async fn follow(&self, odom: OdomMovement, path: geo::Path) {
        let mut run = true;
        while run {
            let odometry_values = odom.odometry_values.lock().await;
            let (x, y) = (odometry_values.global_x, odometry_values.global_y);
            let cir = geo::Circle {
                x: x,
                y: y,
                r: self.lookahead,
            };
            let target = algorithm::pursuit_target(path.clone(), cir);
            odom.arc_point(target.x, target.y).await;

            if let Some(arcpid) = odom.arc_pid.clone() {
                let values = arcpid.arcpid_values.lock().await;
                run = values.active;
            }
        }
    }
}
