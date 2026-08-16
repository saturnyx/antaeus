//! Pursuit control algorithms and interfaces.
//!
//! Defines the `PursuitControl` trait used by higher-level pursuit logic to
//! convert a lookahead point into drivetrain commands, along with built-in
//! control implementations.
//!
//! These Control Algorithms have their own feedback loops and lower-level
//! hardware control algorithms.

use crate::utils::units::Length;

/// Direction policy for the pursuit controller.
#[derive(Debug, Clone, Copy, Default)]
pub enum PursuitDirection {
    /// Always drive forward toward the lookahead point.
    Forward,
    /// Always drive backward toward the lookahead point.
    Backward,
    /// Choose direction automatically based on the target location.
    #[default]
    Auto,
}
pub mod basic;

/// # `PursuitControl` Trait
/// The lower-level algorithm used by the Pursuit Algorithm to access and
/// control motor voltages.
pub trait PursuitControl {
    /// A Control Algorithm that generates wheel velocities depending on a
    /// point relative to the robot.
    fn control(&self, x: Length, y: Length, lookahead: Length) -> ((f64, f64), bool);
}
