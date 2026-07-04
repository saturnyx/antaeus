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
//! use antaeus::misc::units::Length;
//! use antaeus::motion::pursuit::{Pursuit, geo::{Path, Point}};
//!
//! let pursuit = Pursuit { lookahead: Length::from_inches(12.0) };
//!
//! let path = Path::from_vec(vec![
//!     Point::new(Length::zero(), Length::zero()),
//!     Point::new(Length::from_inches(24.0), Length::zero()),
//!     Point::new(Length::from_inches(24.0), Length::from_inches(24.0)),
//! ]);
//!
//! pursuit.follow(odom, path).await;
//! ```

mod algorithm;

pub mod control;

const LOOPRATE: Duration = Duration::from_millis(10);

use std::time::Duration;

use control::PursuitControl;
use snafu::Snafu;
use vexide::time::sleep;

use crate::{
    motion::{
        localization::{Localizer, tracker::devices::TrackingSensorError},
        pursuit::algorithm::abs_arc_point,
    },
    peripherals::drivetrain::{Differential, DrivetrainError},
    utils::{
        geo::{self, Pose},
        units::Length,
    },
};

/// A marker trait to isolate tracking sensor error types
pub trait IsLocalizerError: std::error::Error + 'static {}

// Implement it for your default tracking error
impl IsLocalizerError for TrackingSensorError {}

/// An error that occured when the CBP Algorithm was running.
#[derive(Debug, Snafu)]
pub enum PursuitError<E = TrackingSensorError>
where E: IsLocalizerError {
    // Require E to be a valid error type {
    /// An error occurred while accessing a tracking sensor.
    /// This variant is now completely generic.
    #[snafu(transparent)]
    LocalizerError {
        /// The underlying generic error from the tracking hardware.
        source: E,
    },

    /// Failed to borrow the motor group mutably (e.g. already borrowed
    /// elsewhere).
    #[snafu(transparent)]
    DrivetrainError {
        /// Errors that can occur while commanding or reading from the drivetrain.
        source: DrivetrainError,
    },

    /// An unknown error occurred (catch-all for unexpected issues).
    Unknown {
        /// A string describing the unknown error.
        string: String,
    },
}

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
    /// use antaeus::misc::units::Length;
    /// let path = Path::from_vec(vec![
    ///     Point::new(Length::zero(), Length::zero()),
    ///     Point::new(Length::from_inches(24.0), Length::from_inches(12.0)),
    ///     Point::new(Length::from_inches(48.0), Length::zero()),
    /// ]);
    ///
    /// pursuit.follow(odom, path).await;
    /// ```
    // TODO: Instead of getting a drivetrain from a argument, use the drivetrain
    // in odom. Make the argument optional. This removes the need to clone dt.
    // Or make the user feed in the pose instead of odom
    pub async fn follow<C: PursuitControl, E, D: Differential>(
        &self,
        odom: &mut impl Localizer<E>,
        drivetrain: &D,
        ctrl_algorithm: &C,
        path: geo::Path,
    ) -> Result<(), PursuitError<E>>
    // <-- Changed from PursuitError to PursuitError<TE>
    where
        E: IsLocalizerError, // <-- Constrain TE instead of using From<TE>
    {
        let mut run = true;
        while run {
            let odometry_values = odom.get_coords();
            let (x, y, t) = (odometry_values.x, odometry_values.y, odometry_values.t);
            let cir = geo::Circle {
                x: x.as_inches(),
                y: y.as_inches(),
                r: self.lookahead.as_inches(),
            };
            let target = algorithm::pursuit_target(path.clone(), cir);
            let (tarx, tary) = abs_arc_point(
                Pose::new(x, y, t),
                Length::as_inches(Length::from_inches(target.x)),
                Length::as_inches(Length::from_inches(target.y)),
            );
            let ((powl, powr), run_curr) = ctrl_algorithm.control(
                Length::from_inches(tarx),
                Length::from_inches(tary),
                self.lookahead,
            );

            drivetrain.set_left_voltage(powl)?;
            drivetrain.set_right_voltage(powr)?;
            run = run_curr;
            odom.tick().await?;
            sleep(LOOPRATE).await;
        }
        Ok(())
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
    pub async fn tick<C: PursuitControl, E, D: Differential>(
        &self,
        odom: &mut impl Localizer<E>,
        drivetrain: &D,
        ctrl_algorithm: &C,
        path: geo::Path,
    ) -> Result<bool, PursuitError<E>>
    // <-- Changed from PursuitError to PursuitError<TE>
    where
        E: IsLocalizerError, // <-- Constrain TE instead of using From<TE>
    {
        let odometry_values = odom.get_coords();
        let (x, y, t) = (odometry_values.x, odometry_values.y, odometry_values.t);
        let cir = geo::Circle {
            x: x.as_inches(),
            y: y.as_inches(),
            r: self.lookahead.as_inches(),
        };
        let target = algorithm::pursuit_target(path.clone(), cir);
        let (tarx, tary) = abs_arc_point(
            Pose::new(x, y, t),
            Length::as_inches(Length::from_inches(target.x)),
            Length::as_inches(Length::from_inches(target.y)),
        );
        let ((powl, powr), run_curr) = ctrl_algorithm.control(
            Length::from_inches(tarx),
            Length::from_inches(tary),
            self.lookahead,
        );
        drivetrain.set_left_voltage(powl)?;
        drivetrain.set_right_voltage(powr)?;
        odom.tick().await?;

        Ok(run_curr)
    }
}
