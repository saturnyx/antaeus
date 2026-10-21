//! Distance Sensor Implementation

use snafu::Snafu;
use vexide::{prelude::DistanceSensor, smart::distance::DistanceObjectError};

use crate::prelude::{Length, Ranger};

#[derive(Snafu, Debug)]

/// Error for RangerDistance
pub enum RangerDistanceError {
    /// No object was found
    ObjectNotFound,
    #[snafu(transparent)]
    /// Distance Object error of [`DistanceSensor`]
    DistanceObjectError {
        /// Source
        source: DistanceObjectError,
    },
}

impl Ranger for DistanceSensor {
    type Error = RangerDistanceError;

    fn distance(&self) -> Result<Length, Self::Error> {
        let obj = self.object()?;
        match obj {
            None => Err(RangerDistanceError::ObjectNotFound),
            Some(obj) => Ok(Length::from_millimeters(obj.distance as f64)),
        }
    }
}
