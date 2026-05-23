//! Geometry primitives for path definition.
//!
//! Provides `Point`, `Line`, `Path`, and `Circle` types
//! used by the pursuit algorithm.

use std::{
    cmp::Ordering,
    ops::{Add, Div, Mul, Neg, Sub},
    time::Duration,
};

const MM_PER_INCH_FACTOR: f64 = 25.4;

// ==================
// Length
// ==================

/// A length measurement, stored internally as inches but with constructors and
/// accessors for various units. Supports basic arithmetic and comparisons.
#[derive(Copy, Clone, Debug, Default)]
pub struct Length {
    inches: f64,
}

impl Length {
    // --- Constructors ---

    /// A length of zero inches.
    pub fn zero() -> Self { Self { inches: 0.0 } }

    /// Create a length from a value in inches.
    pub fn from_inches(inches: f64) -> Self { Self { inches } }

    /// Create a length from a value in millimeters.
    pub fn from_millimeters(mm: f64) -> Self {
        Self {
            inches: mm / MM_PER_INCH_FACTOR,
        }
    }

    /// Create a length from a value in centimeters.
    pub fn from_centimeters(cm: f64) -> Self { Self::from_millimeters(cm * 10.0) }

    /// Create a length from a value in centimeters.
    pub fn from_centimetres(cm: f64) -> Self { Self::from_centimeters(cm) }

    /// Create a length from a value in meters.
    pub fn from_meters(m: f64) -> Self { Self::from_millimeters(m * 1000.0) }

    /// Create a length from a value in meters.
    pub fn from_metres(m: f64) -> Self { Self::from_meters(m) }

    // --- Accessors ---

    /// Get the length in inches.
    pub fn as_inches(self) -> f64 { self.inches }

    /// Get the length in millimeters.
    pub fn as_millimeters(self) -> f64 { self.inches * MM_PER_INCH_FACTOR }

    /// Get the length in centimeters.
    pub fn as_centimeters(self) -> f64 { self.as_millimeters() / 10.0 }

    /// Get the length in meters.
    pub fn as_meters(self) -> f64 { self.as_millimeters() / 1000.0 }

    /// Get the length in metres.
    pub fn as_metres(self) -> f64 { self.as_meters() }

    // --- Curated helpers ---

    /// Get the absolute value of this length.
    pub fn abs(self) -> Self { Self::from_inches(self.inches.abs()) }

    /// Check if this length is finite (not infinite or NaN).
    pub fn is_finite(self) -> bool { self.inches.is_finite() }

    /// Check if this length is NaN (not a number).
    pub fn is_nan(self) -> bool { self.inches.is_nan() }

    /// Get the minimum of this length and another length.
    pub fn min(self, other: Self) -> Self { if self < other { self } else { other } }

    /// Get the maximum of this length and another length.
    pub fn max(self, other: Self) -> Self { if self > other { self } else { other } }

    /// Compute average speed over a duration in seconds.
    pub fn per_seconds(self, seconds: f64) -> Speed {
        Speed::from_inches_per_second(self.inches / seconds)
    }
}

impl PartialEq for Length {
    fn eq(&self, other: &Self) -> bool { self.inches == other.inches }
}

impl PartialOrd for Length {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.inches.partial_cmp(&other.inches)
    }
}

impl Add for Length {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output { Self::from_inches(self.inches + rhs.inches) }
}

impl Sub for Length {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output { Self::from_inches(self.inches - rhs.inches) }
}

impl Mul<f64> for Length {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output { Self::from_inches(self.inches * rhs) }
}

impl Div<f64> for Length {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output { Self::from_inches(self.inches / rhs) }
}

impl Div<Duration> for Length {
    type Output = Speed;

    fn div(self, rhs: Duration) -> Self::Output {
        let seconds = rhs.as_secs_f64();
        Speed::from_inches_per_second(self.inches / seconds)
    }
}

impl Neg for Length {
    type Output = Self;

    fn neg(self) -> Self::Output { Self::from_inches(-self.inches) }
}

// ==================
// Speed
// ==================

/// A speed measurement, stored internally as inches per second but with
/// constructors and accessors for various units. Supports basic arithmetic and
/// comparisons.
#[derive(Copy, Clone, Debug, Default)]
pub struct Speed {
    inches_per_second: f64,
}

impl Speed {
    // --- Constructors ---

    /// A speed of zero inches per second.
    pub fn zero() -> Self {
        Self {
            inches_per_second: 0.0,
        }
    }

    /// Create a speed from a value in inches per second.
    pub fn from_inches_per_second(in_per_s: f64) -> Self {
        Self {
            inches_per_second: in_per_s,
        }
    }

    /// Create a speed from a value in millimeters per second.
    pub fn from_millimeters_per_second(mm_per_s: f64) -> Self {
        Self {
            inches_per_second: mm_per_s / MM_PER_INCH_FACTOR,
        }
    }

    /// Create a speed from a value in centimeters per second.
    pub fn from_centimeters_per_second(cm_per_s: f64) -> Self {
        Self::from_millimeters_per_second(cm_per_s * 10.0)
    }

    /// Create a speed from a value in centimeters per second.
    pub fn from_centimetres_per_second(cm_per_s: f64) -> Self {
        Self::from_centimeters_per_second(cm_per_s)
    }

    /// Create a speed from a value in meters per second.
    pub fn from_meters_per_second(m_per_s: f64) -> Self {
        Self::from_millimeters_per_second(m_per_s * 1000.0)
    }

    /// Create a speed from a value in meters per second.
    pub fn from_metres_per_second(m_per_s: f64) -> Self { Self::from_meters_per_second(m_per_s) }

    // --- Accessors ---

    /// Get the speed in inches per second.
    pub fn as_inches_per_second(self) -> f64 { self.inches_per_second }

    /// Get the speed in millimeters per second.
    pub fn as_millimeters_per_second(self) -> f64 { self.inches_per_second * MM_PER_INCH_FACTOR }

    /// Get the speed in centimeters per second.
    pub fn as_centimeters_per_second(self) -> f64 { self.as_millimeters_per_second() / 10.0 }

    /// Get the speed in meters per second.
    pub fn as_meters_per_second(self) -> f64 { self.as_millimeters_per_second() / 1000.0 }

    /// Get the speed in metres per second.
    pub fn as_metres_per_second(self) -> f64 { self.as_meters_per_second() }

    // --- Curated helpers ---

    /// Get the absolute value of this speed.
    pub fn abs(self) -> Self { Self::from_inches_per_second(self.inches_per_second.abs()) }

    /// Check if this speed is finite (not infinite or NaN).
    pub fn is_finite(self) -> bool { self.inches_per_second.is_finite() }

    /// Check if this speed is NaN (not a number).
    pub fn is_nan(self) -> bool { self.inches_per_second.is_nan() }

    /// Get the minimum of this speed and another speed.
    pub fn min(self, other: Self) -> Self { if self < other { self } else { other } }

    /// Get the maximum of this speed and another speed.
    pub fn max(self, other: Self) -> Self { if self > other { self } else { other } }

    /// Distance covered in `seconds` at this constant speed.
    pub fn times_seconds(self, seconds: f64) -> Length {
        Length::from_inches(self.inches_per_second * seconds)
    }
}

impl PartialEq for Speed {
    fn eq(&self, other: &Self) -> bool { self.inches_per_second == other.inches_per_second }
}

impl PartialOrd for Speed {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.inches_per_second.partial_cmp(&other.inches_per_second)
    }
}

impl Add for Speed {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::from_inches_per_second(self.inches_per_second + rhs.inches_per_second)
    }
}

impl Sub for Speed {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::from_inches_per_second(self.inches_per_second - rhs.inches_per_second)
    }
}

impl Mul<f64> for Speed {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self::from_inches_per_second(self.inches_per_second * rhs)
    }
}

impl Div<f64> for Speed {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self::from_inches_per_second(self.inches_per_second / rhs)
    }
}

impl Mul<Duration> for Speed {
    type Output = Length;

    fn mul(self, rhs: Duration) -> Self::Output {
        let seconds = rhs.as_secs_f64();
        Length::from_inches(self.inches_per_second * seconds)
    }
}

impl Neg for Speed {
    type Output = Self;

    fn neg(self) -> Self::Output { Self::from_inches_per_second(-self.inches_per_second) }
}

// ==================
// Reverse operators
// ==================

impl Mul<Length> for f64 {
    type Output = Length;

    fn mul(self, rhs: Length) -> Self::Output { rhs * self }
}

impl Mul<Speed> for f64 {
    type Output = Speed;

    fn mul(self, rhs: Speed) -> Self::Output { rhs * self }
}

// ==================
// Test utils
// ==================

const DEFAULT_DELTA: f64 = 1e-5;

/// Check two floating point values are approximately equal
pub fn almost_eq(a: f64, b: f64) -> bool { almost_eq_delta(a, b, DEFAULT_DELTA) }

/// Check two floating point values are approximately equal using some given delta (a fraction of the inputs)
pub fn almost_eq_delta(a: f64, b: f64, d: f64) -> bool { (abs(a - b) / a) < d }

/// Assert two floating point values are approximately equal
pub fn assert_almost_eq(a: f64, b: f64) { assert_almost_eq_delta(a, b, DEFAULT_DELTA); }

/// Assert two floating point values are approximately equal using some given delta (a fraction of the inputs)
pub fn assert_almost_eq_delta(a: f64, b: f64, d: f64) {
    if !almost_eq_delta(a, b, d) {
        panic!("assertion failed: {:?} != {:?} (within {:?})", a, b, d);
    }
}

/// This function doesn't seem to be available no `#![no_std]` so we re-
/// implemented it here.
fn abs(x: f64) -> f64 { if x > 0.0 { x } else { -x } }
