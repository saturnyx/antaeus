//! Ranger Sensor
//!
//! The [`Ranger`] Trait is use to create sensors which output distances, such as
//! distance sensor and ADI Rangefinder. Filters (with sensors) can also
//! implement the [`Ranger`] trait, allowing higher level algorithms to use any
//! Ranger Sensor.

use crate::prelude::Length;

pub mod distance;
pub mod mean;
pub mod rangefinder;

/// The [`Ranger`] trait is use to create sensors which output distances, such as
/// distance sensor and ADI Rangefinder. Filters (with sensors) can also
/// implement the [`Ranger`] trait, allowing higher level algorithms to use any
/// Ranger Sensor.
pub trait Ranger {
    /// Ranger Error type
    type Error: std::error::Error + Send + Sync + 'static;

    /// Get Distance
    ///
    /// This function will retrieve the distance value from the sensor. This
    /// function should only read the current state and should not "tick" or
    /// "update" the sensor.
    fn distance(&self) -> Result<Length, Self::Error>;

    /// Tick the [`Ranger`]
    ///
    /// This function is usually used for filters which have state machines that
    /// need to be ticked.
    fn tick(&mut self) -> Result<(), Self::Error>;
}
