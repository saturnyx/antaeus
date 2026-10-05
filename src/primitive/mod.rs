//! Primitive Algorithms such as Feedback control

pub mod bang;
pub mod pid;

/// Primitive Feedpack Control Trait
pub trait Feedback {
    /// Implementation Specific Error-type
    type Error: std::error::Error + Send + Sync + 'static;

    /// Tick the feedback loop by one tick
    fn tick(&mut self, reading: f64, dt: f64) -> Result<f64, Self::Error>;

    /// Set the target for the feedback loop
    fn set_target(&mut self, target: f64) -> Result<(), Self::Error>;

    /// Resets the internal state to be brand new
    fn reset(&mut self) -> Result<(), Self::Error>;

    /// Returns whether the Feedback Control is active
    /// - `true`: The controller is  active and the error is greater than the tolerance
    /// - `false`: The controller is inactive and the error is smaller than the tolerance
    fn is_active(&self, reading: f64) -> bool;
}
