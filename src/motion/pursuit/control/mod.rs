use crate::misc::units::Length;

pub enum PursuitDirection {
    Forward,
    Backward,
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
