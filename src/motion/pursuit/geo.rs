//! Geometry primitives for path definition and calculations.
//!
//! This module provides simple geometric types used by the pursuit
//! algorithm for path representation and intersection calculations.
//!
//! # Types
//!
//! * `Point`: A 2D point with x and y coordinates.
//! * `Line`: A line segment between two points.
//! * `Path`: A sequence of waypoints forming a path.
//! * `Circle`: A circle defined by center and radius.

use std::vec;

use measurements::{Length, Measurement};

/// A 2D point in the coordinate system.
///
/// Used to represent waypoints, robot positions, and intersection points.
///
/// # Example
///
/// ```ignore
/// let waypoint = Point::new(24.0, 12.0);
/// ```
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// The x-coordinate in inches.
    pub x: f64,
    /// The y-coordinate in inches.
    pub y: f64,
}

/// A sequence of waypoints forming a path.
///
/// The robot will travel through these points in order during
/// path following.
#[derive(Debug, Clone)]
pub struct Path {
    /// The ordered list of waypoints.
    pub waypoints: Vec<Point>,
}

/// A line segment between two points.
///
/// Used internally for path calculations. Not an infinite line—
/// only the segment between `point1` and `point2`.
#[derive(Clone, Copy)]
pub struct Line {
    /// The starting point of the segment.
    pub point1: Point,
    /// The ending point of the segment.
    pub point2: Point,
}

/// A circle defined by center coordinates and radius.
///
/// Used as the lookahead circle in the pursuit algorithm.
#[derive(Clone, Copy)]
pub struct Circle {
    /// X-coordinate of the circle center.
    pub x: f64,
    /// Y-coordinate of the circle center.
    pub y: f64,
    /// Radius of the circle in inches.
    pub r: f64,
}

impl Point {
    /// Create a new point using `x` and `y` coordinates
    pub fn new(x: Length, y: Length) -> Self {
        Point {
            x: x.as_inches(),
            y: y.as_inches(),
        }
    }

    pub fn rnew(x: f64, y: f64) -> Self { Point { x: x, y: y } }

    pub fn origin() -> Self { Point { x: 0.0, y: 0.0 } }
}

impl Path {
    /// Create a path from a vector of points
    pub fn from_vec(waypoints: Vec<Point>) -> Self {
        Self {
            waypoints: waypoints,
        }
    }

    /// Create a path with a start point at origin (0,0)
    pub fn origin() -> Self {
        let vec = vec![Point::new(Length::from_base_units(0.0), Length::from_base_units(0.0))];
        Self { waypoints: vec }
    }

    /// Create a path with a start point
    pub fn from_pt(pt: Point) -> Self {
        let vec = vec![pt];
        Self { waypoints: vec }
    }

    /// Add a point to a path
    pub fn add(&mut self, waypoint: Point) { self.waypoints.push(waypoint); }

    /// Append a vector to the path
    pub fn append_vec(&mut self, mut waypoints: Vec<Point>) {
        self.waypoints.append(&mut waypoints);
    }

    /// Remove a point using its index
    pub fn remove(&mut self, t: usize) { self.waypoints.remove(t); }

    /// Get the path as a vector of lines
    pub fn get_lines(&self) -> Vec<Line> {
        let mut lines: Vec<Line> = Vec::new();
        for i in 0..self.waypoints.len() - 1 {
            lines.push(Line {
                point1: Point::new(
                    Length::from_inches(self.waypoints[i].x),
                    Length::from_inches(self.waypoints[i].y),
                ),
                point2: Point::new(
                    Length::from_inches(self.waypoints[i + 1].x),
                    Length::from_inches(self.waypoints[i + 1].y),
                ),
            });
        }
        lines
    }
}

impl Line {
    /// Create a new line from 2 coordinates
    pub fn new(x1: Length, y1: Length, x2: Length, y2: Length) -> Line {
        Line {
            point1: Point {
                x: (x1.as_inches()),
                y: (y1.as_inches()),
            },
            point2: Point {
                x: (x2.as_inches()),
                y: (y2.as_inches()),
            },
        }
    }

    /// Create a new line from 2 points
    pub fn from_pts(point1: Point, point2: Point) -> Line {
        Line {
            point1: point1,
            point2: point2,
        }
    }
}

impl Circle {
    /// Create a new circle
    pub fn new(x: Length, y: Length, r: Length) -> Circle {
        Circle {
            x: x.as_inches(),
            y: y.as_inches(),
            r: r.as_inches(),
        }
    }

    pub fn rnew(x: f64, y: f64, r: f64) -> Circle { Circle { x: x, y: y, r: r } }
}
