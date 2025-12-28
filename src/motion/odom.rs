//! Odometry tracking for robot position estimation.
//!
//! This module implements odometry tracking using perpendicular tracking wheels
//! and an inertial sensor to estimate the robot's global position on the field.
//!
//! # How It Works
//!
//! Odometry uses wheel encoders to measure how far the robot has traveled
//! and an inertial sensor (IMU) to measure rotation. By combining these
//! measurements over time, we can estimate the robot's (x, y) position
//! and heading on the field.
//!
//! # Hardware Requirements
//!
//! - **Vertical tracking wheel**: Measures forward/backward movement.
//! - **Horizontal tracking wheel**: Measures sideways (strafing) movement.
//! - **Inertial sensor (IMU)**: Measures rotation for accurate heading.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::odom::{OdomMovement, Trackers, WheelTracker, TrackingDevice};
//!
//! // Set up tracking hardware
//! let vertical = WheelTracker::new_normal(
//!     TrackingDevice::RotationSensor(rotation_sensor),
//!     2.75,  // wheel diameter in inches
//!     0.0,   // offset from tracking center
//! );
//!
//! // Create odometry instance
//! let odom = OdomMovement { /* ... */ };
//! odom.init();  // Start the tracking loop
//!
//! // Use for navigation
//! odom.goto_point(24.0, 24.0).await;
//! ```

use std::{cell::RefCell, rc::Rc, sync::Arc, time::Duration};

use log::{info, warn};
use vexide::{
    math::Angle,
    prelude::{AdiOpticalEncoder, InertialSensor, RotationSensor},
    sync::Mutex,
    task::spawn,
    time::sleep,
};

use crate::{
    drivetrain::Differential,
    motion::pid::{arcpid::ArcPIDMovement, pid::PIDMovement},
};

/// Loop rate for the odometry tracking task in milliseconds.
const LOOPRATE: u64 = 5;

/// Default timeout for movement operations in milliseconds.
const TIMEOUT: u64 = 10000;

/// Default delay after movement completion in milliseconds.
const AFTERDELAY: u64 = 10;

async fn odom_tracker(values: &Arc<Mutex<OdomValues>>, trackers: &Rc<RefCell<Trackers>>) {
    info!("Odometry Tracking Started");
    let mut prev_dist_v = 0.0;
    let mut prev_dist_h = 0.0;
    let mut prev_heading = 0.0;

    loop {
        // The absolute number of radians turned by the robot
        let abs_rotation;
        let euler_heading;
        {
            let imu = &trackers.borrow().imu;
            abs_rotation = imu
                .rotation()
                .unwrap_or_else(|e| {
                    warn!("Inertial Sensor Error: {}", e);
                    Angle::from_degrees(0.0)
                })
                .as_radians();
            euler_heading = imu.euler().unwrap().b.as_degrees();
        }
        // Getting delta theta (needed later)
        let delta_heading = abs_rotation - prev_heading;

        // Vertical Tracking Wheel Calculations
        let (vertical_rad, delta_v, wheel_dia_v, offset_v);
        {
            let vertical = &trackers.borrow().vertical;
            vertical_rad = vertical.device.position().as_radians().clone();
            let raw_delta_v = (vertical_rad * vertical.wheel_diameter / 2.0) - prev_dist_v;
            delta_v = if vertical.reverse {
                -raw_delta_v
            } else {
                raw_delta_v
            };
            wheel_dia_v = vertical.wheel_diameter.clone();
            offset_v = vertical.offset.clone();
        }

        // Horizontal Tracking Wheel Calculations
        let (horizontal_rad, delta_h, wheel_dia_h, offset_h);
        {
            let horizontal = &trackers.borrow().horizontal;
            horizontal_rad = horizontal.device.position().as_radians().clone();
            let raw_delta_h = (horizontal_rad * horizontal.wheel_diameter / 2.0) - prev_dist_h;
            delta_h = if horizontal.reverse {
                -raw_delta_h
            } else {
                raw_delta_h
            };
            wheel_dia_h = horizontal.wheel_diameter.clone();
            offset_h = horizontal.offset.clone();
        }

        // Getting local change in coords
        let (delta_y, delta_x);
        if delta_heading == 0.0 {
            delta_y = delta_v;
            delta_x = delta_h;
        } else {
            delta_y = 2.0 * (delta_heading / 2.0).sin() * (delta_v / delta_heading + offset_v);
            delta_x = 2.0 * (delta_heading / 2.0).sin() * (delta_h / delta_heading + offset_h);
        }
        let avg_heading = prev_heading + delta_heading / 2.0;
        let (delta_global_x, delta_global_y) = rotate_vector(-avg_heading, delta_x, delta_y);
        {
            let mut s = values.lock().await;
            s.global_heading = euler_heading;
            s.global_x += delta_global_x;
            s.global_y += delta_global_y;
        }
        prev_dist_v = vertical_rad * wheel_dia_v / 2.0;
        prev_dist_h = horizontal_rad * wheel_dia_h / 2.0;

        prev_heading = abs_rotation;
        sleep(Duration::from_millis(LOOPRATE)).await;
    }
}

impl OdomMovement {
    /// Initializes the odometry tracking system.
    ///
    /// This method spawns a background task that continuously updates
    /// the robot's position based on tracking wheel and IMU readings.
    ///
    /// **Must be called before any movement methods.**
    ///
    /// # Example
    ///
    /// ```ignore
    /// let odom = OdomMovement { /* ... */ };
    /// odom.init();  // Start tracking
    /// odom.goto_point(24.0, 0.0).await;
    /// ```
    pub fn init(&self) {
        let thread_clone = self.odometry_values.clone();
        let thread_trackers = self.trackers.clone();
        let mainloop = spawn(async move {
            odom_tracker(&thread_clone, &thread_trackers).await;
        });
        mainloop.detach();
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
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no PID controller is configured.
    pub async fn face_point(&self, x: f64, y: f64) {
        let delta_x = x - self.odometry_values.lock().await.global_x;
        let delta_y = y - self.odometry_values.lock().await.global_y;
        let angle = delta_y.atan2(delta_x).to_degrees();
        let imu = &self.trackers.borrow().imu;
        if let Some(pid) = &self.pid {
            pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot face point without Movement Algorithm (PID needed)")
        }
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
        let delta_x = x - self.odometry_values.lock().await.global_x;
        let delta_y = y - self.odometry_values.lock().await.global_y;
        let angle = delta_y.atan2(delta_x).to_degrees();
        let hyp = (delta_x.powi(2) + delta_y.powi(2)).sqrt();
        let imu = &self.trackers.borrow().imu;
        if let Some(pid) = &self.pid {
            pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
            pid.travel(hyp, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot go to point without Movement Algorithm (PID needed)")
        }
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
        let delta_x = x - self.odometry_values.lock().await.global_x;
        let delta_y = y - self.odometry_values.lock().await.global_y;
        let angle = delta_y.atan2(delta_x).to_degrees();
        let hyp = (delta_x.powi(2) + delta_y.powi(2)).sqrt();
        let imu = &self.trackers.borrow().imu;
        if let Some(pid) = &self.pid {
            pid.rotate_imu(angle, imu, TIMEOUT, AFTERDELAY).await;
            pid.travel(hyp, TIMEOUT, AFTERDELAY).await;
            pid.rotate(heading, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot go to pose without Movement Algorithm (PID needed)")
        }
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
        if let Some(pid) = &self.pid {
            pid.travel(distance, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot travel without Movement Algorithm (PID needed)")
        }
    }

    /// Moves the robot in a curved arc.
    ///
    /// Uses the Arc PID controller to execute a curved movement.
    /// The offset determines the radius of the arc.
    ///
    /// # Arguments
    ///
    /// * `distance` - Arc length to travel in inches.
    /// * `offset` - Curvature offset (larger values = tighter turn).
    ///
    /// # Panics
    ///
    /// Logs a warning and returns early if no Arc PID controller is configured.
    pub async fn arc_travel(&self, distance: f64, offset: f64) {
        if let Some(arc_pid) = &self.arc_pid {
            arc_pid.travel(distance, offset, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot travel without Movement Algorithm (Arc PID needed)")
        }
    }

    /// Moves the robot in an arc to reach specific coordinates.
    ///
    /// Calculates the appropriate arc to reach the target point from
    /// the current position. This is an internal method used by path
    /// following algorithms.
    ///
    /// **Warning**: This method updates absolute position targets and
    /// should not be called directly in most cases.
    ///
    /// # Arguments
    ///
    /// * `x` - Target X coordinate in inches.
    /// * `y` - Target Y coordinate in inches.
    pub async fn arc_point(&self, x: f64, y: f64) {
        // Get current robot position and heading
        let (current_x, current_y, current_heading) = {
            let odom = self.odometry_values.lock().await;
            (odom.global_x, odom.global_y, odom.global_heading)
        };

        // Calculate difference in global coordinates
        let delta_x = x - current_x;
        let delta_y = y - current_y;

        // Convert heading from degrees to radians and rotate to get local coordinates
        let heading_rad = current_heading.to_radians();
        let (local_x, local_y) = rotate_vector(-heading_rad, delta_x, delta_y);

        // Use ArcPID to move to the local coordinates
        if let Some(arc_pid) = &self.arc_pid {
            arc_pid.local_coords(local_x, local_y).await;
        } else {
            warn!("Cannot arc to point without Movement Algorithm (Arc PID needed)")
        }
    }
}

fn rotate_vector(angle: f64, x: f64, y: f64) -> (f64, f64) {
    let cos_theta = angle.cos();
    let sin_theta = angle.sin();
    let global_x = cos_theta * x - sin_theta * y;
    let global_y = sin_theta * x + cos_theta * y;
    (global_x, global_y)
}

/// Global odometry position values.
///
/// This struct holds the robot's estimated position and heading on the field.
/// Values are updated continuously by the odometry tracking task.
///
/// # Coordinate System
///
/// - **X-axis**: Typically left-right on the field.
/// - **Y-axis**: Typically front-back on the field.
/// - **Heading**: Rotation in degrees (0-360).
pub struct OdomValues {
    /// The robot's global X coordinate in inches.
    pub global_x:       f64,
    /// The robot's global Y coordinate in inches.
    pub global_y:       f64,
    /// The robot's global heading in degrees.
    pub global_heading: f64,
}

/// Represents a device that can measure rotational position.
///
/// Tracking wheels can be connected to different sensor types.
/// This enum abstracts over the specific hardware being used.
pub enum TrackingDevice {
    /// An ADI (3-wire) optical shaft encoder.
    AdiOpticalEncoder(AdiOpticalEncoder),
    /// A V5 rotation sensor (high-resolution encoder).
    RotationSensor(RotationSensor),
    /// Uses the drivetrain motors' integrated encoders.
    Differential(Differential),
    /// No tracking device (placeholder for optional tracking).
    None,
}

impl TrackingDevice {
    /// Returns the current rotational position of the tracking device.
    ///
    /// For encoders, this is the total rotation since the last reset.
    /// For differential drivetrains, this is the average motor position.
    ///
    /// # Returns
    ///
    /// The position as an [`Angle`]. Returns zero if the device
    /// encounters an error (a warning is logged).
    pub fn position(&self) -> Angle {
        match self {
            TrackingDevice::AdiOpticalEncoder(encoder) => encoder.position().unwrap_or_else(|e| {
                warn!("ADI Optical Sensor Error: {}", e);
                Angle::from_radians(0.0)
            }),
            TrackingDevice::RotationSensor(encoder) => encoder.position().unwrap_or_else(|e| {
                warn!("Rotation Sensor Error: {}", e);
                Angle::from_radians(0.0)
            }),
            TrackingDevice::Differential(dt) => dt.position(),
            TrackingDevice::None => Angle::from_radians(0.0),
        }
    }
}

/// Configuration for a tracking wheel.
///
/// Tracking wheels are unpowered wheels with encoders used to measure
/// how far the robot has traveled. This struct holds the physical
/// configuration of a tracking wheel.
pub struct WheelTracker {
    /// The sensor device measuring wheel rotation.
    device:         TrackingDevice,
    /// The diameter of the tracking wheel in inches.
    ///
    /// Common sizes are 2.75", 3.25", and 4".
    wheel_diameter: f64,
    /// The perpendicular distance from the tracking center in inches.
    ///
    /// For vertical wheels, this is the horizontal offset.
    /// For horizontal wheels, this is the vertical offset.
    offset:         f64,
    /// Whether to reverse the encoder readings.
    ///
    /// Set to `true` if the wheel reads negative when moving forward.
    reverse:        bool,
}

impl WheelTracker {
    /// Creates a new tracking wheel configuration.
    ///
    /// # Arguments
    ///
    /// * `device` - The sensor measuring wheel rotation.
    /// * `wheel_diameter` - Diameter of the tracking wheel in inches.
    /// * `offset` - Perpendicular distance from the tracking center.
    /// * `reverse` - Whether to reverse encoder readings.
    pub fn new(device: TrackingDevice, wheel_diameter: f64, offset: f64, reverse: bool) -> Self {
        Self {
            device,
            wheel_diameter,
            offset,
            reverse,
        }
    }

    /// Creates a new tracking wheel with normal (non-reversed) readings.
    ///
    /// Equivalent to [`WheelTracker::new`] with `reverse` set to `false`.
    pub fn new_normal(device: TrackingDevice, wheel_diameter: f64, offset: f64) -> Self {
        Self::new(device, wheel_diameter, offset, false)
    }

    /// Creates a new tracking wheel with reversed readings.
    ///
    /// Equivalent to [`WheelTracker::new`] with `reverse` set to `true`.
    /// Use this when the wheel reads negative when moving forward.
    pub fn new_reversed(device: TrackingDevice, wheel_diameter: f64, offset: f64) -> Self {
        Self::new(device, wheel_diameter, offset, true)
    }
}

/// Hardware configuration for odometry tracking.
///
/// Groups together the tracking wheels and inertial sensor
/// needed for position estimation.
pub struct Trackers {
    /// The vertical (forward/backward) tracking wheel.
    vertical:   WheelTracker,
    /// The horizontal (left/right) tracking wheel.
    horizontal: WheelTracker,
    /// The inertial sensor for heading measurement.
    imu:        InertialSensor,
}

/// The main odometry-based movement controller.
///
/// Combines position tracking with movement algorithms to enable
/// precise autonomous navigation. Supports point-to-point movement,
/// arc movements, and path following.
///
/// # Initialization
///
/// You must call [`init()`](OdomMovement::init) before using any movement
/// methods. This starts the background tracking task.
///
/// # Example
///
/// ```ignore
/// let odom = OdomMovement { /* ... */ };
/// odom.init();
///
/// // Navigate to a point
/// odom.goto_point(24.0, 24.0).await;
///
/// // Face a specific heading
/// odom.face_point(48.0, 0.0).await;
/// ```
pub struct OdomMovement {
    /// Thread-safe container for current position values.
    ///
    /// Updated continuously by the background tracking task.
    pub odometry_values: Arc<Mutex<OdomValues>>,
    /// Shared reference to the tracking hardware.
    pub trackers:        Rc<RefCell<Trackers>>,
    /// Optional PID controller for linear movements.
    pub pid:             Option<PIDMovement>,
    /// Optional Arc PID controller for curved movements.
    pub arc_pid:         Option<ArcPIDMovement>,
}
