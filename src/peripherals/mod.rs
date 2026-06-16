//! # Peripheral Control
//! Module that assists in controlling of peripherals
//!
//! ## Drivetrain
//! Originally forked from evian's drivetrain module. It now has various
//! upgrades and integrations within Antaeus. Currently only supports
//! differential drivetrains (tank drive).
//!
//! ## Range Sensor
//! A sensor that can be used to measure distances. This class of sensors
//! include the newer Smart v5 Distance Sensor and the older ADI RangeFinder.
//!
//! ## Mapper
//! This module contains the code for mapping the inputs from the controller and
//! other peripherals to the outputs of the motors and ADI devices. This allows
//! for easy control of mechanisms like pneumatic solenoids using controller
//! inputs or other sensors.
//!
//! ## Motorgroup
//! A re-export of Zabackary's `vexide_motorgroup`

pub mod drivetrain;

pub mod range_sensor;

pub mod mapper;

// REMAPPED LIBRARIES ---------------------------------------------------------+

pub use vexide_motorgroup as motorgroup;

// LEGACY ---------------------------------------------------------------------+

#[cfg(feature = "legacy")]
pub mod controller;
