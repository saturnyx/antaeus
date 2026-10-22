//! ADI Rangefinder Implementation
//!
//! Implements [`Ranger`] for `vexide::prelude::AdiRangeFinder`

use snafu::Snafu;
use vexide::{prelude::AdiRangeFinder, smart::PortError};

use crate::prelude::{Length, Ranger};

/// Error for RangerRangeFinder
#[derive(Debug, Snafu)]
pub enum RangerRangeFinderError {
    /// Port Error (Sensor was disconnected or is the wrong device)
    #[snafu(transparent)]
    PortError {
        /// Source
        source: PortError,
    },
    /// Occurs when there is no object in range
    ObjectNotFound,
}

impl Ranger for AdiRangeFinder {
    type Error = RangerRangeFinderError;

    fn distance(&self) -> Result<Length, Self::Error> {
        match self.distance()? {
            None => Err(RangerRangeFinderError::ObjectNotFound),
            Some(dist) => Ok(Length::from_centimeters(dist as f64)),
        }
    }

    fn tick(&mut self) -> Result<(), Self::Error> { Ok(()) }
}
