use std::{f64::consts::PI, marker::PhantomData, sync::Arc, time::Duration};

use log::{info, warn};
use vexide::{
    math::{Angle, EulerAngles},
    smart::{imu::*, motor::BrakeMode},
    sync::Mutex,
    task::*,
    time::*,
};

use crate::{drivetrain, drivetrain::Differential};

const LOOPRATE: u64 = 5; // The ArcPID will run with a loop rate of 5 millisecs

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
    let mut ierror = 0.0;

    // seconds per loop from configured looprate in ms
    let dt = (LOOPRATE as f64) / 1000.0;

    loop {
        let (target, offset, pwr, kp, kd, ki, leeway) = {
            let s = arcpidvalues.lock().await;
            (s.target, s.offset, s.maxpwr, s.kp, s.kd, s.ki, s.leeway)
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

        ierror += error * dt;
        let mut u;
        let mut u_left;
        let mut u_right;
        let ki = ki;
        if ki != 0.0 {
            let i_max = pwr.abs() / ki.abs();
            ierror = ierror.clamp(-i_max, i_max);
        }

        let derror = (error - perror) / dt;

        u = kp * error + ki * ierror + kd * derror;

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
        in_band = error.abs() < leeway;

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

            ierror = 0.0;
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

    /// Set the Leeway, Kp, Ki and Kd Values for ArcPID. The values are in radians.
    pub async fn tune(&self, kp: f64, ki: f64, kd: f64, leeway: f64) {
        let mut arcpid_values = self.arcpid_values.lock().await;
        arcpid_values.kp = kp;
        arcpid_values.ki = ki;
        arcpid_values.kd = kd;
        arcpid_values.leeway = leeway;
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

    pub async fn local_coords(
        &self,
        track_width: f64,
        x: f64,
        y: f64,
        timeout: u64,
        afterdelay: u64,
    ) {
    }
}

/// **The ArcPID Movement Controller**
///
/// Initialize an instance of this to control the robot using ArcPID.
///
/// # Examples
///
/// Creating a ArcPIDMovement Instance
/// ```
/// fn new_arcpid() -> ArcPIDMovement {
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
///         ki:           0.0,
///         leeway:       0.02,
///         maxpwr:       12.0,
///         active:       true,
///         target_left:  0.0,
///         target_right: 0.0,
///     };
///     let ArcPID_controller = ArcPIDMovement {
///         drivetrain:        dt,
///         drivetrain_config: config,
///         arcpid_values:     Arc::new(Mutex::new(values)),
///     };
/// }
/// ```
pub struct ArcPIDMovement {
    drivetrain:        Differential,
    drivetrain_config: DrivetrainConfig,
    arcpid_values:     Arc<Mutex<ArcPIDValues>>,
}

/// A Struct for ArcPID values that will be altered throughout the Autonomous
pub struct ArcPIDValues {
    kp:     f64,
    ki:     f64,
    kd:     f64,
    leeway: f64,
    maxpwr: f64,
    active: bool,
    target: f64,
    offset: f64,
}

/// The Drivtrain's Physical Configuration that will be used for calculations
pub struct DrivetrainConfig {
    /// The wheel diameter in inches.
    /// Most teams use 2.75", 3.25" or sometimes 4".
    /// The older large omni wheels were 4.15".
    wheel_diameter: f64,
    /// The size of the driving gear. This can
    /// be in any units as long as the same units
    /// are used for the driven gear value.
    driving_gear:   f64,
    /// The size of the driven gear. This can
    /// be in any units as long as the same units
    /// are used for the driving gear value.
    driven_gear:    f64,
    // The width from the right wheels to the left.
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

fn get_heading(imu: &InertialSensor) -> f64 {
    let is_calibrating = imu.is_calibrating().unwrap_or_else(|e| {
        warn!("IMU Calibration State Error: {}", e);
        true
    });
    if !is_calibrating {
        let angle = imu
            .euler()
            .unwrap_or_else(|e| {
                warn!("IMU Calibration State Error: {}", e);
                EulerAngles {
                    a:      (Angle::from_degrees(0.0)),
                    b:      (Angle::from_degrees(0.0)),
                    c:      (Angle::from_degrees(0.0)),
                    marker: PhantomData,
                }
            })
            .b
            .as_degrees();
        if angle > 180.0 { angle - 360.0 } else { angle }
    } else {
        0.0
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
