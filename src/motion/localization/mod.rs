//! Localization (odometry) tracking for robot position estimation.
//!
//! # Structure
//!
//! * **Tracker**: The main localization controller that estimates robot pose
//!   using tracking wheels.
//!
//! # Extensions
//!
//! You can write your own localization program by implementing the `Localizer`
//! trait. Below is an example.
//!
//! # Example
//!
//! ```
#![doc = include_str!("../../../examples/integration.rs")]
//! ```

use crate::utils::geo::Pose;
pub mod tracker;

/// Trait for localization controllers that estimate robot pose over time.
pub trait Localizer<E> {
    /// Returns the current estimated pose of the robot.
    fn get_coords(&self) -> Pose;
    /// Updates the internal pose estimate based on sensor readings
    fn tick(&mut self) -> Result<(), E>;
}
