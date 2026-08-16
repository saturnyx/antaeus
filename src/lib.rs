//! # Antaeus
//!
//! Antaeus is a versatile robotics framework built on top of [Vexide](https://vexide.dev).
//! It provides a comprehensive set of tools for VEX V5 robot programming, including:
//!
//! - **Drivetrain Control**: Support for differential drivetrains with tank, arcade, and
//!   reverse control schemes.
//! - **Motion Control**: PID-based movement systems, localization, and path following
//!   using the Candidate-Based Pursuit algorithm.
//! - **Display Graphics**: An [`embedded-graphics`](https://crates.io/crates/embedded-graphics)
//!   compatible driver for the V5 Brain display, with TUI rendering support
//! - **Operator Control**: Utilities for mapping controller buttons to motors and ADI devices.
//! - **Logging**: A file-based logger for debugging and telemetry.
//!
//!
//! ## Structure
//!
//! - [`peripherals`]: Peripheral Access and Enhancements
//! - [`motion`]: Autonomous motion algorithms including PID, localization, and pursuit.
//! - [`graphics`]: V5 Brain display graphics using `embedded-graphics` and `ratatui`.
//! - [`logger`]: File-based logging.
//! - [`utils`]: Error-handling, units, geometry, etc.
//!
//! ## Example
//! ```
#![doc = include_str!("../examples/basic.rs")]
//! ```

#![allow(clippy::too_many_arguments)]
#![warn(missing_docs, rust_2018_idioms, unused, future_incompatible)]
#![deny(clippy::unused_async)]
#![warn(rustdoc::all)]
#![feature(custom_inner_attributes)]
// #![warn(clippy::pedantic)]
// Will enable this sometime soon

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

/// Turns a object into a mutex (only use for actual threads, not async)
pub fn to_mutex<T>(v: T) -> Arc<Mutex<T>> { Arc::new(Mutex::new(v)) }

/// Prelude for Wildcard Imports
///
/// Use this wisely and only where needed to prevent namespace pollution
pub mod prelude {
    pub use crate::{
        logger::*,
        motion::{feedback_control::*, localization::*, pursuit::*},
        peripherals::{drivetrain::*, mapper::*, motorgroup::*, range_sensor::*},
        utils::{
            error::Report,
            geo::*,
            units::{Length, Speed},
        },
    };
}
