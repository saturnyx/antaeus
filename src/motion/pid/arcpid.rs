//! Arc PID controller for curved robot movements.
//!
//! This module provides a PID controller that allows the robot to move
//! in arcs rather than stopping to turn. It's useful for smooth movements
//! but less precise than standard PID.
//!
//! # How It Works
//!
//! The Arc PID controller applies different power ratios to the left and
//! right motor groups based on an "offset" value. This causes the robot
//! to curve while moving forward.
//!
//! # When to Use
//!
//! - Path following where smooth curves are preferred over stop-and-turn.
//! - Time-critical autonomous routines where turning in place is too slow.
//! - Approaches to game elements where a curved path is more efficient.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::pid::arcpid::ArcPIDMovement;
//!
//! let arc_pid = ArcPIDMovement { /* ... */ };
//! arc_pid.init();
//!
//! // Travel in a curve: positive offset = curve right
//! arc_pid.travel(24.0, 0.5, 2000, 100).await;
//! ```

use std::{f64::consts::PI, sync::Arc, time::Duration};

use log::info;
use vexide::{smart::motor::BrakeMode, sync::Mutex, task::*, time::*};

use crate::{drivetrain, drivetrain::Differential};

/// Loop rate for the Arc PID control task in milliseconds.
const LOOPRATE: u64 = 5;

async fn arcpid_loop(
    arcpidvalues: &Arc<Mutex<ArcPIDValues>>,
    drivetrain: drivetrain::Differential,
) {
    info!("ArcPID Control Loop Started");
    // Set brake mode and reset positions for left motors
    {
        let mut left_motors = drivetrain.left.borrow_mut();
        let left_slice = left_motors.as_mut();
        for motor in left_slice.iter_mut() {
            let _ = motor.brake(BrakeMode::Brake);
            let _ = motor.reset_position();
        }
    }

    // Set brake mode and reset positions for right motors
    {
        let mut right_motors = drivetrain.right.borrow_mut();
        let right_slice = right_motors.as_mut();
        for motor in right_slice.iter_mut() {
            let _ = motor.brake(BrakeMode::Brake);
            let _ = motor.reset_position();
        }
    }

    let mut perror = 0.0;

    // seconds per loop from configured looprate in ms
    let dt = (LOOPRATE as f64) / 1000.0;

    loop {
        let (target, offset, pwr, kp, kd, tolerance) = {
            let s = arcpidvalues.lock().await;
            (s.target, s.offset, s.maxpwr, s.kp, s.kd, s.tolerance)
        };

        let currs_left = {
            let mut left_motors = drivetrain.left.borrow_mut();
            let left_slice = left_motors.as_mut();
            let sum: f64 = left_slice
                .iter()
                .map(|motor| motor.position().unwrap_or_default().as_radians())
                .sum();
            sum / left_slice.len() as f64
        };

        let currs_right = {
            let mut right_motors = drivetrain.right.borrow_mut();
            let right_slice = right_motors.as_mut();
            let sum: f64 = right_slice
                .iter()
                .map(|motor| motor.position().unwrap_or_default().as_radians())
                .sum();
            sum / right_slice.len() as f64
        };

        let currs = (currs_left + currs_right) / 2.0;
        let error = target - currs;

        let u;
        let u_left;
        let u_right;

        let derror = (error - perror) / dt;

        u = kp * error + kd * derror;

        if offset > 0.0 {
            u_left = abscap(u, pwr.abs());
            u_right = abscap(u * offset.abs(), pwr.abs());
        } else if offset < 0.0 {
            u_left = abscap(u * offset.abs(), pwr.abs());
            u_right = abscap(u, pwr.abs());
        } else {
            u_left = abscap(u, pwr.abs());
            u_right = abscap(u, pwr.abs());
        }
        // Set voltage for left motors
        {
            let mut left_motors = drivetrain.left.borrow_mut();
            let left_slice = left_motors.as_mut();
            for motor in left_slice.iter_mut() {
                let _ = motor.set_voltage(u_left);
            }
        }
        // Set voltage for right motors
        {
            let mut right_motors = drivetrain.right.borrow_mut();
            let right_slice = right_motors.as_mut();
            for motor in right_slice.iter_mut() {
                let _ = motor.set_voltage(u_right);
            }
        }
        let in_band;
        in_band = error.abs() < tolerance;

        if in_band {
            let mut s = arcpidvalues.lock().await;
            s.active = false;

            // Stop left motors
            {
                let mut left_motors = drivetrain.left.borrow_mut();
                let left_slice = left_motors.as_mut();
                for motor in left_slice.iter_mut() {
                    let _ = motor.set_voltage(0.0);
                }
            }

            // Stop right motors
            {
                let mut right_motors = drivetrain.right.borrow_mut();
                let right_slice = right_motors.as_mut();
                for motor in right_slice.iter_mut() {
                    let _ = motor.set_voltage(0.0);
                }
            }
        }
        perror = error;
        sleep(Duration::from_millis(LOOPRATE)).await;
    }
}

impl ArcPIDMovement {
    /// Initializes a ArcPID Loop.
    /// The ArcPID movements will require a ArcPID loop to run as a seperate task or thread.
    /// It is necessary to initialize the ArcPID before running any movements.
    ///
    /// # Examples
    /// ```
    /// async fn auton(arcpid: ArcPIDMovement) {
    ///     arcpid.init(); // Initialize the ArcPID before any movements
    ///     arcpid.set_maximum_power(12.0).await;
    ///     arcpid.travel(100.0, 1000, 10).await;
    /// }
    /// ```
    pub fn init(&self) {
        let mutex_clone = self.arcpid_values.clone();
        let drivetrain = self.drivetrain.clone();
        let mainloop = spawn(async move {
            arcpid_loop(&mutex_clone, drivetrain).await;
        });
        mainloop.detach();
    }

    /// Set the tolerance, Kp, and Kd Values for ArcPD. The values are in radians.
    pub async fn tune(&self, kp: f64, kd: f64, tolerance: f64) {
        let mut arcpid_values = self.arcpid_values.lock().await;
        arcpid_values.kp = kp;
        arcpid_values.kd = kd;
        arcpid_values.tolerance = tolerance;
    }

    /// Sets the maximum power the robot should move at. The maximum value is 12.0
    /// while the minimum value is -12.0 (reverse).
    pub async fn set_maximum_power(&self, maximum_power: f64) {
        let mut arcpid_values = self.arcpid_values.lock().await;
        arcpid_values.maxpwr = maximum_power;
    }

    /// Makes the robot travel in an arc or straight line
    pub async fn travel(&self, distance: f64, offset: f64, timeout: u64, afterdelay: u64) {
        let r = (distance *
            (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
            2.0 *
            PI) /
            self.drivetrain_config.wheel_diameter;
        let mut s = self.arcpid_values.lock().await;
        s.active = true;
        s.target += r;
        s.offset = offset;
        timeout_wait(&self.arcpid_values, timeout).await;
        {
            let mut s = self.arcpid_values.lock().await;
            s.active = false;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }

    pub async fn abs_travel(&self, distance: f64, offset: f64) {
        let r = (distance *
            (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
            2.0 *
            PI) /
            self.drivetrain_config.wheel_diameter;
        let mut s = self.arcpid_values.lock().await;
        s.active = true;
        s.target = r;
        s.offset = offset;
    }

    pub async fn local_coords(&self, x: f64, y: f64) {
        let track_width = self.drivetrain_config.track_width;
        let offset;
        let (radius, angle) = get_arc(x, y);
        if angle > 0.0 {
            offset = ((2.0 * radius) - track_width) / ((2.0 * radius) + track_width);
        } else if angle < 0.0 {
            offset = ((2.0 * radius) + track_width) / ((2.0 * radius) - track_width);
        } else {
            offset = 0.0;
        }
        let distance = radius * angle;
        self.abs_travel(distance, offset).await;
    }
}

/// **The ArcPD Movement Controller**
///
/// Initialize an instance of this to control the robot using ArcPD.
///
/// # Examples
///
/// Creating a ArcPDMovement Instance
/// ```
/// fn new_arcpd() -> ArcPIDMovement {
///     let dt = Differential::new(
///         [
///             Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
///             Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
///         ],
///         [
///             Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
///             Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
///         ],
///     );
///     let config = DrivetrainConfig {
///         wheel_diameter: 3.25,
///         driving_gear:   3.0,
///         driven_gear:    5.0,
///         track_width:    12.0,
///     };
///     let values = ArcPIDValues {
///         kp:           0.5,
///         kd:           0.1,
///         tolerance:    0.02,
///         maxpwr:       12.0,
///         active:       true,
///         target_left:  0.0,
///         target_right: 0.0,
///     };
///     let ArcPD_controller = ArcPIDMovement {
///         drivetrain:        dt,
///         drivetrain_config: config,
///         arcpid_values:     Arc::new(Mutex::new(values)),
///     };
/// }
/// ```
#[derive(Clone)]
pub struct ArcPIDMovement {
    pub drivetrain:        Differential,
    pub drivetrain_config: DrivetrainConfig,
    pub arcpid_values:     Arc<Mutex<ArcPIDValues>>,
}

/// Runtime values for the Arc PID controller.
///
/// These values are updated during movement commands and
/// read by the background control loop.
#[derive(Clone, Copy)]
pub struct ArcPIDValues {
    /// Proportional gain for the PD controller.
    pub kp:        f64,
    /// Derivative gain for the PD controller.
    pub kd:        f64,
    /// Error tolerance in radians.
    ///
    /// Movement completes when error is below this value.
    pub tolerance: f64,
    /// Maximum motor voltage (0-12 volts).
    pub maxpwr:    f64,
    /// Whether a movement is currently active.
    pub active:    bool,
    /// Target motor position in radians.
    pub target:    f64,
    /// Curvature offset for arc movements.
    ///
    /// - Positive values: curve right (left motors faster).
    /// - Negative values: curve left (right motors faster).
    /// - Zero: straight line movement.
    pub offset:    f64,
}

/// Physical configuration of the drivetrain for arc calculations.
///
/// Same as the PID [`DrivetrainConfig`](super::pid::DrivetrainConfig)
/// but includes track width for arc radius calculations.
#[derive(Clone, Copy)]
pub struct DrivetrainConfig {
    /// The wheel diameter in inches.
    ///
    /// Common sizes: 2.75", 3.25", 4".
    wheel_diameter: f64,
    /// The number of teeth on the driving (motor-side) gear.
    driving_gear:   f64,
    /// The number of teeth on the driven (wheel-side) gear.
    driven_gear:    f64,
    /// The distance between left and right wheels in inches.
    ///
    /// Used for calculating arc radii.
    track_width:    f64,
}

async fn timeout_wait(arcpid_values: &Arc<Mutex<ArcPIDValues>>, timeout: u64) {
    let start_time = user_uptime().as_millis();

    loop {
        {
            let s = arcpid_values.lock().await;
            if !s.active {
                break;
            }
        }

        if user_uptime().as_millis() >= start_time + timeout as u128 {
            let mut s = arcpid_values.lock().await;
            s.active = false;
            break;
        }

        sleep(Duration::from_millis(LOOPRATE)).await;
    }
}

fn abscap(val: f64, cap: f64) -> f64 {
    let result: f64;
    if val > cap {
        result = cap;
    } else if val < -cap {
        result = -cap;
    } else {
        result = val;
    }
    result
}

/// Warning! This fn is not as easy as you think...
/// Calculate arc radius and turn angle from (0,0) heading north to (x,y)
/// Returns (radius, turn_angle_radians). Positive angle = right turn, negative = left turn
fn get_arc(x: f64, y: f64) -> (f64, f64) {
    let r = (x * x + y * y) / (2.0 * x);
    let angle = ((y / x).atan()).abs();
    (r.abs(), angle * r.signum())
}
