/// **Pursuit Point**
///
/// Contains 3 values:
/// - `x`: The x-coordinate
/// - `y`: The y-coordinate
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point {
    /// The x-coordinate
    pub x: f64,
    /// The y-coordinate
    pub y: f64,
}
/// A Vector of Points
#[derive(Clone)]
pub struct Path {
    /// A vector of Waypoints that make up the path.
    pub waypoints: Vec<Point>,
}
#[derive(Clone, Copy)]
pub struct Line {
    pub point1: Point,
    pub point2: Point,
}

#[derive(Clone, Copy)]
pub struct Circle {
    pub x: f64,
    pub y: f64,
    pub r: f64,
}

impl Point {
    pub fn new(x: f64, y: f64) -> Self { Point { x, y } }
}

impl Path {
    pub fn new(waypoints: Vec<Point>) -> Path {
        Path {
            waypoints: waypoints,
        }
    }

    pub fn add(&mut self, waypoint: Point) { self.waypoints.push(waypoint); }

    pub fn append(&mut self, mut waypoints: Vec<Point>) { self.waypoints.append(&mut waypoints); }

    pub fn remove(&mut self, t: usize) { self.waypoints.remove(t); }

    pub fn get_lines(&self) -> Vec<Line> {
        let mut lines: Vec<Line> = Vec::new();
        for i in 0..self.waypoints.len() - 1 {
            lines.push(Line {
                point1: Point::new(self.waypoints[i].x, self.waypoints[i].y),
                point2: Point::new(self.waypoints[i + 1].x, self.waypoints[i + 1].y),
            });
        }
        lines
    }
}

impl Line {
    pub fn new(x1: f64, y1: f64, x2: f64, y2: f64) -> Line {
        Line {
            point1: Point { x: (x1), y: (y1) },
            point2: Point { x: (x2), y: (y2) },
        }
    }

    pub fn from_pts(point1: Point, point2: Point) -> Line {
        Line {
            point1: point1,
            point2: point2,
        }
    }
}

impl Circle {
    pub fn new(x: f64, y: f64, r: f64) -> Circle { Circle { x: x, y: y, r: r } }
}
