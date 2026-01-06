use crate::motion::{odom::tracker::OdomTracker, pid::pid::PIDMovement};

/// Default timeout for movement operations in milliseconds.
const TIMEOUT: u64 = 10000;

/// Default delay after movement completion in milliseconds.
const AFTERDELAY: u64 = 10;

impl PointShoot {
    /// Rotates the robot to face a specific point on the field.
    ///
    /// Calculates the angle to the target point and rotates the robot
    /// to face that direction using the IMU for feedback.
    ///
    /// # Arguments
    ///
    /// * `x` - Target X coordinate in inches.
    /// * `y` - Target Y coordinate in inches.
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no PID controller is configured.
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
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no PID controller is configured.
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
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no PID controller is configured.
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
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no PID controller is configured.
    pub async fn travel(&self, distance: f64) {
        self.pid.travel(distance, TIMEOUT, AFTERDELAY).await;
    }
}

pub struct PointShoot {
    pub odom: OdomTracker,
    pub pid:  PIDMovement,
}
