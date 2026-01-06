//! Odometry tracking controller.
//!
//! This module provides the [`OdomTracker`] struct which manages continuous
//! position tracking using tracking wheels and an inertial sensor.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::odom::tracker::OdomTracker;
//! use antaeus::motion::odom::devices::{TrackerMech, Tracker, TrackingSensor, Pose};
//! use vexide::prelude::*;
//! use std::sync::Arc;
//! use vexide::sync::Mutex;
//!
//! // Create tracking mechanism
//! let mechanism = TrackerMech::new(vertical_tracker, horizontal_tracker, imu);
//!
//! // Create and initialize odometry tracker
//! let odom = OdomTracker::new(mechanism);
//! odom.init();
//!
//! // Read current position
//! let pose = odom.global_pose.lock().await;
//! println!("Position: ({}, {})", pose.x, pose.y);
//! ```

use std::sync::Arc;

use vexide::{math::Angle, sync::Mutex, task::spawn};

use super::{algorithm::*, devices::Pose};
use crate::motion::odom::devices::TrackerMech;

/// Odometry position tracker.
///
/// This struct manages continuous position tracking using tracking wheels
/// and an inertial sensor. It runs a background task that continuously
/// updates the robot's estimated position.
///
/// # Example
///
/// ```ignore
/// use antaeus::motion::odom::tracker::OdomTracker;
/// use antaeus::motion::odom::devices::TrackerMech;
///
/// let odom = OdomTracker::new(mechanism);
/// odom.init();  // Start tracking
///
/// // Access position asynchronously
/// let pose = odom.global_pose.lock().await;
/// ```
pub struct OdomTracker {
    /// The tracking mechanism (wheels and IMU).
    pub trackermech: TrackerMech,
    /// The current global position, updated by the tracking loop.
    pub global_pose: Arc<Mutex<Pose>>,
}

impl OdomTracker {
    /// Initializes the odometry tracking loop.
    ///
    /// Spawns a background task that continuously reads sensor values
    /// and updates the global position. Must be called before reading
    /// position data.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let odom = OdomTracker::new(mechanism);
    /// odom.init();  // Start the tracking loop
    /// ```
    pub fn init(&self) {
        let pose = self.global_pose.clone();
        let trackers = self.trackermech.clone();
        let mainloop = spawn(async move {
            odomloop(&trackers, &pose).await;
        });
        mainloop.detach();
    }

    /// Creates a new OdomTracker starting at the origin (0, 0).
    ///
    /// # Arguments
    ///
    /// * `tm` - The tracking mechanism configuration.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let odom = OdomTracker::new(mechanism);
    /// ```
    pub fn new(tm: TrackerMech) -> Self {
        Self {
            trackermech: tm,
            global_pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, Angle::from_radians(0.0)))),
        }
    }

    /// Creates a new OdomTracker starting at a specific pose.
    ///
    /// # Arguments
    ///
    /// * `tm` - The tracking mechanism configuration.
    /// * `pose` - The initial position and heading.
    ///
    /// # Example
    ///
    /// ```ignore
    /// use vexide::math::Angle;
    /// let start_pose = Pose::new(24.0, 0.0, Angle::from_degrees(90.0));
    /// let odom = OdomTracker::from_pose(mechanism, start_pose);
    /// ```
    pub fn from_pose(tm: TrackerMech, pose: Pose) -> Self {
        Self {
            trackermech: tm,
            global_pose: Arc::new(Mutex::new(pose)),
        }
    }

    /// Resets the position to the origin (0, 0) with heading 0.
    ///
    /// # Example
    ///
    /// ```ignore
    /// odom.reset_origin().await;
    /// ```
    pub async fn reset_origin(&self) {
        let mut gp = self.global_pose.lock().await;
        *gp = Pose::new(0.0, 0.0, Angle::from_radians(0.0));
    }

    /// Resets the position to a specific pose.
    ///
    /// # Arguments
    ///
    /// * `pose` - The new position and heading.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let new_pose = Pose::new(24.0, 24.0, Angle::from_degrees(45.0));
    /// odom.reset_from_pose(new_pose).await;
    /// ```
    pub async fn reset_from_pose(&self, pose: Pose) {
        let mut gp = self.global_pose.lock().await;
        *gp = pose;
    }
}
