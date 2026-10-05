//! Pursuit control algorithms and interfaces.
//!
//! Defines the [`ArcSteer`] trait used by higher-level pursuit logic to
//! convert a lookahead point into drivetrain commands, along with built-in
//! control implementations.
//!
//! These Control Algorithms have their own feedback loops and lower-level
//! hardware control algorithms.

use crate::{prelude::Differential, utils::units::Length};

pub mod basic;

/// ArcSteer Trait
/// The lower-level algorithm used by the Pursuit Algorithm to access and
/// control motor voltages.
pub trait ArcSteer {
    /// ArcSteer Error
    type Error: std::error::Error + Send + Sync + 'static;

    /// A Steering Algorithm that generates wheel velocities depending on a
    /// point relative to the robot.
    fn steer<D: Differential>(
        &mut self,
        x: Length,
        y: Length,
        lookahead: Length,
        drivetrain: &D,
    ) -> Result<((f64, f64), bool), Self::Error>;
}
