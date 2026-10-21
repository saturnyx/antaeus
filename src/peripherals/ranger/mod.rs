//! Ranger Sensor
//!
//! The [`Ranger`] Trait is use to create sensors which output distances, such as
//! distance sensor and ADI Rangefinder. Filters (with sensors) can also
//! implement the [`Ranger`] trait, allowing higher level algorithms to use any
//! Ranger Sensor.
use std::{cell::RefCell, rc::Rc};

use snafu::Snafu;
use vexide::{adi::range_finder::AdiRangeFinder, smart::distance::DistanceSensor};

use crate::{
    make_cloneable,
    peripherals::ranger::{distance::RangerDistanceError, rangefinder::RangerRangeFinderError},
    prelude::Length,
};

pub mod distance;
pub mod rangefinder;

/// The [`Ranger`] Trait is use to create sensors which output distances, such as
/// distance sensor and ADI Rangefinder. Filters (with sensors) can also
/// implement the [`Ranger`] trait, allowing higher level algorithms to use any
/// Ranger Sensor.
pub trait Ranger {
    /// Ranger Error type
    type Error: std::error::Error + Send + Sync + 'static;

    /// Get Distance
    fn distance(&self) -> Result<Length, Self::Error>;
}

/// A dynamically selected range sensor backed by VEX hardware.
#[derive(Debug, Clone)]
pub enum RangeSensor {
    /// V5 smart distance sensor.
    Distance(Rc<RefCell<DistanceSensor>>),
    /// ADI range finder.
    RangeFinder(Rc<RefCell<AdiRangeFinder>>),
    /// Test-only sensor with manually supplied values.
    #[cfg(any(test, debug_assertions))]
    Mock {
        /// Mock Distance
        distance: Option<Length>,
    },
}

/// Errors returned by [`RangeSensor`].
#[derive(Debug, Snafu)]
pub enum RangeSensorError {
    /// The underlying sensor could not be read.
    #[snafu(transparent)]
    Distance {
        /// Source
        source: RangerDistanceError,
    },
    /// The underlying range finder could not be read.
    #[snafu(transparent)]
    RangeFinder {
        /// Source
        source: RangerRangeFinderError,
    },
    /// The sensor is currently borrowed elsewhere.
    #[snafu(display("sensor is already borrowed"))]
    Borrow,
    /// No object was detected.
    #[snafu(display("sensor did not detect a distance"))]
    NoDistance,
}

impl RangeSensor {
    /// Wraps a V5 smart distance sensor.
    pub fn from_distance(sensor: DistanceSensor) -> Self { Self::Distance(make_cloneable(sensor)) }

    /// Wraps an ADI range finder.
    pub fn from_adi(sensor: AdiRangeFinder) -> Self { Self::RangeFinder(make_cloneable(sensor)) }
}

impl Ranger for RangeSensor {
    type Error = RangeSensorError;

    fn distance(&self) -> Result<Length, Self::Error> {
        match self {
            Self::Distance(sensor) => sensor
                .try_borrow()
                .map_err(|_| RangeSensorError::Borrow)?
                .distance()
                .map_err(|source| RangeSensorError::Distance { source }),
            Self::RangeFinder(sensor) => sensor
                .try_borrow()
                .map_err(|_| RangeSensorError::Borrow)?
                .distance()
                .map_err(|source| RangeSensorError::RangeFinder {
                    source: rangefinder::RangerRangeFinderError::PortError { source },
                })
                .and_then(|distance| match distance {
                    Some(distance) => Ok(Length::from_centimeters(distance as f64)),
                    None => Err(RangeSensorError::NoDistance),
                }),
            #[cfg(any(test, debug_assertions))]
            Self::Mock { distance, .. } => distance.ok_or(RangeSensorError::NoDistance),
        }
    }
}
