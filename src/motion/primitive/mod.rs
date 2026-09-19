//! Primitive Algorithms such as Feedback control

pub mod pid;

/// Primitive Feedpack Control Trait
pub trait Feedback {
    /// Implementation Specific Error-type
    type Error;

    /// Tick the feedback loop by one tick
    fn tick(&mut self, reading: f64, dt: f64) -> Result<f64, Self::Error>;

    /// Set the target for the feedback loop
    fn set_target(&mut self, target: f64) -> Result<(), Self::Error>;
}
