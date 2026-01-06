//! Point-and-shoot navigation using odometry.
//!
//! This module provides the [`PointShoot`] struct which combines odometry
//! tracking with PID control to enable simple point-to-point navigation.
//!
//! The "point-and-shoot" approach rotates to face the target, then drives
//! straight to it. This is simpler than arc-based pursuit but may be slower.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::odom::ptsht::PointShoot;
//! use antaeus::motion::odom::tracker::OdomTracker;
//! use antaeus::motion::pid::pid::PIDMovement;
//!
//! let point_shoot = PointShoot { odom, pid };
//!
//! // Navigate to a point
//! point_shoot.goto_point(24.0, 24.0).await;
//!
//! // Navigate to a pose (position + heading)
//! point_shoot.goto_pose(48.0, 0.0, 90.0).await;
//! ```

use crate::motion::{odom::tracker::OdomTracker, pid::pid::PIDMovement};

/// Default timeout for movement operations in milliseconds.
const TIMEOUT: u64 = 10000;

/// Default delay after movement completion in milliseconds.
const AFTERDELAY: u64 = 10;

/// Point-and-shoot navigation controller.
///
/// Combines odometry tracking with PID control for simple point-to-point
/// navigation. The robot rotates to face the target, then drives straight
/// to it.
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::ptsht::PointShoot;
///
/// let point_shoot = PointShoot { odom, pid };
/// point_shoot.goto_point(24.0, 24.0).await;
/// ```
pub struct PointShoot {
    /// The odometry tracker for position feedback.
    pub odom: OdomTracker,
    /// The PID controller for movement execution.
    pub pid:  PIDMovement,
}

impl PointShoot {
    /// Creates a new PointShoot controller.
    ///
    /// # Arguments
    ///
    /// * `odom` - The odometry tracker for position feedback.
    /// * `pid` - The PID controller for movement execution.
    pub fn new(odom: OdomTracker, pid: PIDMovement) -> Self {
        Self { odom, pid }
    }

    /// Rotates the robot to face a specific point on the field.
    ///
    /// Calculates the angle to the target point and rotates the robot
    /// to face that direction using the IMU for feedback.
    ///
    /// # Arguments
    ///
    /// * `x` - Target X coordinate in inches.
    /// * `y` - Target Y coordinate in inches.
    pub async fn face_point(&self, x: f64, y: f64) {
        let delta_x = x - self.odom.global_pose.lock().await.x;
        let delta_y = y - self.odom.global_pose.lock().await.y;
        let angle = delta_y.atan2(delta_x).to_degrees();

        let imu = &self.odom.trackermech.imu.lock().await;
        self.pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
    }

    /// Moves the robot to a specific point on the field.
    ///
    /// First rotates to face the target, then drives straight to it.
    /// Uses the IMU for rotation feedback and motor encoders for distance.
    ///
    /// # Arguments
    ///
    /// * `x` - Target X coordinate in inches.
    /// * `y` - Target Y coordinate in inches.
    pub async fn goto_point(&self, x: f64, y: f64) {
        let delta_x = x - self.odom.global_pose.lock().await.x;
        let delta_y = y - self.odom.global_pose.lock().await.y;
        let angle = delta_y.atan2(delta_x).to_degrees();
        let hyp = (delta_x.powi(2) + delta_y.powi(2)).sqrt();

        let imu = &self.odom.trackermech.imu.lock().await;
        self.pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
        self.pid.travel(hyp, TIMEOUT, AFTERDELAY).await;
    }

    /// Moves the robot to a specific pose (position and heading).
    ///
    /// Executes three movements:
    /// 1. Rotate to face the target point.
    /// 2. Drive straight to the target point.
    /// 3. Rotate to the specified final heading.
    ///
    /// # Arguments
    ///
    /// * `x` - Target X coordinate in inches.
    /// * `y` - Target Y coordinate in inches.
    /// * `heading` - Final heading in degrees.
    pub async fn goto_pose(&self, x: f64, y: f64, heading: f64) {
        let delta_x = x - self.odom.global_pose.lock().await.x;
        let delta_y = y - self.odom.global_pose.lock().await.y;
        let angle = delta_y.atan2(delta_x).to_degrees();
        let hyp = (delta_x.powi(2) + delta_y.powi(2)).sqrt();
        let imu = &self.odom.trackermech.imu.lock().await;
        self.pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
        self.pid.travel(hyp, TIMEOUT, AFTERDELAY).await;
        self.pid.rotate(heading, TIMEOUT, AFTERDELAY).await;
    }

    /// Moves the robot forward in a straight line.
    ///
    /// Uses the PID controller to drive the specified distance.
    /// Positive values move forward, negative values move backward.
    ///
    /// # Arguments
    ///
    /// * `distance` - Distance to travel in inches.
    pub async fn travel(&self, distance: f64) {
        self.pid.travel(distance, TIMEOUT, AFTERDELAY).await;
    }
}
