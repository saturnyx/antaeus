//! A module for error handling utilities.
//!
//! This module provides the `Report` type, which represents a value that may
//! have associated warnings or errors.

/// A type that represents a value that may have associated warnings or errors.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub enum Report<T, E> {
    /// A value with no associated errors.
    Ok(T),
    /// A value with an associated error.
    Warn {
        /// The value that was produced, despite the error.
        value: T,
        /// The error that occurred while producing the value.
        error: E,
    },
}

impl<T, E> Report<T, E> {
    /// Creates a new `Report` with the given value and no errors.
    pub fn new(value: T) -> Self { Self::Ok(value) }

    /// Creates a new `Report` with the given value and error.
    pub fn from_parts(value: T, error: E) -> Self { Self::Warn { value, error } }

    /// Returns `true` if the report has any errors.
    pub fn has_errors(&self) -> bool { matches!(self, Self::Warn { .. }) }

    /// Returns `true` if the report has no errors.
    pub fn is_ok(&self) -> bool { matches!(self, Self::Ok(_)) }

    /// Maps the value of the report using the given function, while preserving
    /// any associated errors.
    pub fn map<U>(self, f: impl FnOnce(T) -> U) -> Report<U, E> {
        match self {
            Report::Ok(v) => Report::Ok(f(v)),
            Report::Warn { value, error } => Report::Warn {
                value: f(value),
                error,
            },
        }
    }

    /// Maps the error of the report using the given function, while preserving
    /// any associated values.
    pub fn into_parts(self) -> (T, Option<E>) {
        match self {
            Report::Ok(v) => (v, None),
            Report::Warn { value, error } => (value, Some(error)),
        }
    }

    /// Returns the value of the report, regardless of any associated errors.
    pub fn value(self) -> T {
        match self {
            Report::Ok(v) => v,
            Report::Warn { value, .. } => value,
        }
    }
}
