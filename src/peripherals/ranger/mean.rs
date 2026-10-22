//! Mean Filter for Ranger Sensors
//!
//! Works by getting the average(mean) of the past and current readings

use crate::{peripherals::ranger::Ranger, utils::units::Length};

/// Mean Ranger Sensor
///
/// Adds a a mean filter over a sensor that implements [`Ranger`]
/// TODO: Add example
pub struct MeanRanger<const N: usize, S: Ranger> {
    /// Ranger sensor
    ///
    /// The sensor has to implement [`Ranger`] to work.
    pub sensor:   S,
    /// The history is the amount of data the filter will remember. A mean-of-3
    /// filter will have a history of 2. History only includes past recordings,
    /// not the current one. It is recommeded to set this value from around 2-5.
    pub history:  [Option<Length>; N],
    /// The priority of the latest reading ou of 1.0. A priority of 0.0 means
    /// that all of the [`history`] and the current reading have the same
    /// weight. A priority above 0.0 means that each reading's weight
    /// will be reduced according to the position in [`history`]. For example,
    /// a priority of 0.1 means that the first(newest) reading in [`history`]
    /// will have a weight of 0.9 or 90% (1 - 0.1 = 0.9); and the second
    /// reading will have an weight of 0.8. A priority of 1.0 means that
    /// only the current reading will be used (It is not recommended as it only
    /// adds additional redundant computation).
    pub priority: f64,
}

/// Error for MeanRanger
pub enum MeanRangerError<E> {
    /// Sensor Error
    RangerError {
        /// Source
        source: E,
    },
}

impl<E: std::fmt::Debug> std::fmt::Debug for MeanRangerError<E> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::RangerError { source } => formatter
                .debug_struct("MeanRangerError")
                .field("source", source)
                .finish(),
        }
    }
}

impl<E: std::fmt::Display> std::fmt::Display for MeanRangerError<E> {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::RangerError { source } => source.fmt(formatter),
        }
    }
}

impl<E: std::error::Error + 'static> std::error::Error for MeanRangerError<E> {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::RangerError { source } => Some(source),
        }
    }
}

impl<const N: usize, S: Ranger> Ranger for MeanRanger<N, S> {
    type Error = MeanRangerError<S::Error>;

    fn distance(&self) -> Result<Length, Self::Error> {
        let curr = self
            .sensor
            .distance()
            .map_err(|source| MeanRangerError::RangerError { source })?;

        let priority = self.priority.clamp(0.0, 1.0);
        let mut weighted_total = curr.as_inches();
        let mut total_weight = 1.0;

        for (index, reading) in self.history.iter().flatten().enumerate() {
            let weight = (1.0 - priority * (index + 1) as f64).max(0.0);
            weighted_total += reading.as_inches() * weight;
            total_weight += weight;
        }

        Ok(Length::from_inches(weighted_total / total_weight))
    }

    fn tick(&mut self) -> Result<(), Self::Error> {
        self.sensor
            .tick()
            .map_err(|source| MeanRangerError::RangerError { source })?;

        let current = self
            .sensor
            .distance()
            .map_err(|source| MeanRangerError::RangerError { source })?;

        for index in (1..N).rev() {
            self.history[index] = self.history[index - 1];
        }
        if N > 0 {
            self.history[0] = Some(current);
        }

        Ok(())
    }
}
