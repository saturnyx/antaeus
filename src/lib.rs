//! # Antaeus
//!
//! Antaeus is a versatile robotics framework built on top of [Vexide](https://vexide.dev).
//! It provides a comprehensive set of tools for VEX V5 robot programming, including:
//!
//! - **Drivetrain Control**: Support for differential drivetrains with tank, arcade, and
//!   reverse control schemes.
//! - **Motion Control**: PID-based movement systems, odometry tracking, and path following
//!   using the Candidate-Based Pursuit algorithm.
//! - **Display Graphics**: An [`embedded-graphics`](https://crates.io/crates/embedded-graphics)
//!   compatible driver for the V5 Brain display, with pre-loaded fonts and logo rendering.
//! - **Operator Control**: Utilities for mapping controller buttons to motors and ADI devices.
//! - **Logging**: A file-based logger for debugging and telemetry.
//!
//! ## Quick Start
//!
//! ```ignore
//! use antaeus::peripherals::drivetrain::Differential;
//! use vexide::prelude::*;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let drivetrain = Differential::new(
//!         [
//!             Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
//!             Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
//!         ],
//!         [
//!             Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
//!             Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
//!         ],
//!     );
//!
//!     let controller = Controller::new(ControllerId::Primary);
//!     loop {
//!         drivetrain.tank(&controller);
//!     }
//! }
//! ```
//!
//! ## Modules
//!
//! - [`peripherals::drivetrain`]: Differential drivetrain control with multiple drive modes.
//! - [`motion`]: Autonomous motion algorithms including PID, localization, and pursuit.
//! - [`display`]: V5 Brain display graphics using `embedded-graphics`.
//! - [`peripherals`]: Controller input mapping to motors, pneumatics, and sensors.
//! - [`fs`]: Filesystem utilities including logging.

#![allow(clippy::too_many_arguments)]
#![warn(missing_docs, rust_2018_idioms, unused, future_incompatible)]
#![deny(clippy::unused_async)]

use std::{cell::RefCell, rc::Rc, sync::Arc};

use vexide::sync::Mutex;

pub mod motion;

pub mod peripherals;

#[cfg(feature = "graphics")]
pub mod graphics;

pub mod utils;

pub mod logger;

/// Makes an object cloneable by wrapping it in `Rc` and `RefCell`
pub fn make_cloneable<T>(v: T) -> Rc<RefCell<T>> { Rc::new(RefCell::new(v)) }

/// Turns a object into a mutex
pub fn to_mutex<T>(v: T) -> Arc<Mutex<T>> { Arc::new(Mutex::new(v)) }
