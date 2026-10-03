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
//! 2. Find all "candidate" points: path waypoints inside ignore the circle,
//!    intersections of the circle with path segments, and the closest
//!    point on the path to the robot.
//! 3. Select the candidate furthest along the path as the target.
//! 4. Drive toward that target using arc movements.
//!
//! # Example
//!
//! ```
#![doc = include_str!("../../../examples/pursuit.rs")]
//! ```

mod algorithm;

pub mod control;

const LOOPRATE: Duration = Duration::from_millis(10);

use std::time::Duration;

use control::ArcSteer;
use snafu::Snafu;
use vexide::time::sleep;

use crate::{
    motion::{localization::Localizer, pursuit::algorithm::abs_arc_point},
    peripherals::drivetrain::{Differential, DrivetrainError},
    utils::{
        geo::{self, Pose},
        units::Length,
    },
};

/// An error that occured when the CBP Algorithm was running.
#[derive(Snafu)]
pub enum PursuitError<L: Localizer, A: ArcSteer> {
    /// An error occurred while accessing a tracking sensor.
    LocalizerError {
        /// The underlying generic error from the tracking hardware.
        source: L::Error,
    },

    /// Failed to borrow the motor group mutably (e.g. already borrowed
    /// elsewhere).
    #[snafu(transparent)]
    DrivetrainError {
        /// Errors that can occur while commanding or reading from the drivetrain.
        source: DrivetrainError,
    },

    /// Error from ArcSteer Algorithm
    ArcSteerError {
        /// Errors that can occur while commanding or reading from the drivetrain.
        source: A::Error,
    },
}

impl<L: Localizer, A: ArcSteer> std::fmt::Debug for PursuitError<L, A> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::LocalizerError { source } => formatter
                .debug_struct("DriveControlError")
                .field("source", source)
                .finish(),
            Self::DrivetrainError { source } => formatter
                .debug_struct("DriveControlError")
                .field("source", source)
                .finish(),
            Self::ArcSteerError { source } => formatter
                .debug_struct("ArcSteerError")
                .field("source", source)
                .finish(),
        }
    }
}

/// Candidate-Based Pursuit path follower.
///
/// Follows a path using the lookahead distance to determine targets.
/// Larger lookahead values result in smoother but less accurate paths.
/// Smaller values track the path more precisely but may cause oscillation.
#[derive(Debug, Clone, Copy)]
pub struct Pursuit {
    /// The lookahead distance.
    /// This is the radius of the circle used to find target points.
    pub lookahead: Length,
}

impl Pursuit {
    /// Creates a new `Pursuit` instance with the specified lookahead distance.
    ///
    /// # Arguments
    ///
    /// - `lookahead` - The lookahead distance in inches, used as the radius for
    ///   target point calculations.
    pub fn new(lookahead: Length) -> Self { Self { lookahead } }

    /// Follows a path using the Candidate-Based Pursuit algorithm.
    ///
    /// This method continuously calculates target points and commands
    /// arc movements until the robot reaches the end of the path.
    ///
    /// # Arguments
    ///
    /// - `odom` - The odometry movement controller
    /// - `drivetrain` - The differential drivetrain
    /// - `ctrl_algorithm` - The control algorithm
    /// - `path` - The path to follow, defined as a series of waypoints.
    pub async fn follow<A: ArcSteer, L: Localizer, D: Differential>(
        &self,
        odom: &mut L,
        drivetrain: &D,
        arc_steer: &mut A,
        path: geo::Path,
    ) -> Result<(), PursuitError<L, A>> {
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
            let ((powl, powr), _) = arc_steer
                .steer(
                    Length::from_inches(tarx),
                    Length::from_inches(tary),
                    self.lookahead,
                    drivetrain,
                )
                .map_err(|source| PursuitError::ArcSteerError { source })?;

            drivetrain.set_left_voltage(powl)?;
            drivetrain.set_right_voltage(powr)?;
            // Steering is based on the moving lookahead target, but completion
            // must be measured against the path's final waypoint. Otherwise, a
            // controller stops after reaching its first lookahead distance.
            run = should_continue_to_final_waypoint(
                arc_steer,
                Pose::new(x, y, t),
                &path,
                self.lookahead,
                drivetrain,
            )
            .map_err(|source| PursuitError::ArcSteerError { source })?;
            odom.tick()
                .map_err(|source| PursuitError::LocalizerError { source })?;
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
    /// - `odom` - The odometry movement controller
    /// - `drivetrain` - The differential drivetrain
    /// - `steer` - The steering algorithm that controls PID loops and lower
    ///   level hardware
    /// - `path` - The path to follow, defined as a series of waypoints.
    pub fn tick<A: ArcSteer, L: Localizer, D: Differential>(
        &self,
        odom: &mut L,
        drivetrain: &D,
        steer: &mut A,
        path: geo::Path,
    ) -> Result<bool, PursuitError<L, A>> {
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
        let ((powl, powr), _) = steer
            .steer(
                Length::from_inches(tarx),
                Length::from_inches(tary),
                self.lookahead,
                drivetrain,
            )
            .map_err(|source| PursuitError::ArcSteerError { source })?;
        drivetrain.set_left_voltage(powl)?;
        drivetrain.set_right_voltage(powr)?;
        let should_continue = should_continue_to_final_waypoint(
            steer,
            Pose::new(x, y, t),
            &path,
            self.lookahead,
            drivetrain,
        )
        .map_err(|source| PursuitError::ArcSteerError { source })?;
        odom.tick()
            .map_err(|source| PursuitError::LocalizerError { source })?;

        Ok(should_continue)
    }
}

/// Returns whether pursuit should continue based on the distance to the final
/// waypoint, using the control implementation's configured tolerance.
fn should_continue_to_final_waypoint<A: ArcSteer, D: Differential>(
    steer: &mut A,
    pose: Pose,
    path: &geo::Path,
    lookahead: Length,
    drivetrain: &D,
) -> Result<bool, A::Error> {
    let final_waypoint = path
        .waypoints
        .last()
        .copied()
        .unwrap_or_else(geo::Point::origin);
    let (final_x, final_y) = abs_arc_point(pose, final_waypoint.x, final_waypoint.y);
    let (_, should_continue) = steer.steer(
        Length::from_inches(final_x),
        Length::from_inches(final_y),
        lookahead,
        drivetrain,
    )?;
    Ok(should_continue)
}
