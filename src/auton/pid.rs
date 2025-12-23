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

const LOOPRATE: u64 = 5; // The PID will run with a loop rate of 5 millisecs

async fn pid_loop(pidvalues: &Arc<Mutex<PIDValues>>, drivetrain: drivetrain::Differential) {
    info!("PID Control Loop Started");
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

    let mut perror_left = 0.0;
    let mut perror_right = 0.0;
    let mut ierror_left = 0.0;
    let mut ierror_right = 0.0;

    // seconds per loop from configured looprate in ms
    let dt = (LOOPRATE as f64) / 1000.0;

    loop {
        let (target_left, target_right, pwr, kp, kd, ki, leeway) = {
            let s = pidvalues.lock().await;
            (s.target_left, s.target_right, s.maxpwr, s.kp, s.kd, s.ki, s.leeway)
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
        let error_left = target_left - currs_left;
        let error_right = target_right - currs_right;

        ierror_left += error_left * dt;
        ierror_right += error_right * dt;
        let mut u_left;
        let mut u_right;
        let ki = ki;
        if ki != 0.0 {
            let i_max = pwr.abs() / ki.abs();
            ierror_left = ierror_left.clamp(-i_max, i_max);
            ierror_right = ierror_right.clamp(-i_max, i_max);
        }

        let derror_left = (error_left - perror_left) / dt;
        let derror_right = (error_right - perror_right) / dt;

        u_left = kp * error_left + ki * ierror_left + kd * derror_left;
        u_right = kp * error_right + ki * ierror_right + kd * derror_right;

        u_left = abscap(u_left, pwr.abs());
        u_right = abscap(u_right, pwr.abs());

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
        in_band = error_left.abs() < leeway && error_right.abs() < leeway;

        if in_band {
            let mut s = pidvalues.lock().await;
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

            ierror_left = 0.0;
            ierror_right = 0.0;
        }
        perror_left = error_left;
        perror_right = error_right;
        sleep(Duration::from_millis(LOOPRATE)).await;
    }
}

impl PIDMovement {
    /// Initializes a PID Loop.
    /// The PID movements will require a PID loop to run as a seperate task or thread.
    /// It is necessary to initialize the PID before running any movements.
    ///
    /// # Examples
    /// ```
    /// async fn auton(pid: PIDMovement) {
    ///     pid.init(); // Initialize the PID before any movements
    ///     pid.set_maximum_power(12.0).await;
    ///     pid.travel(100.0, 1000, 10).await;
    /// }
    /// ```
    pub fn init(&self) {
        let mutex_clone = self.pid_values.clone();
        let drivetrain = self.drivetrain.clone();
        let mainloop = spawn(async move {
            pid_loop(&mutex_clone, drivetrain).await;
        });
        mainloop.detach();
    }

    /// Set the Leeway, Kp, Ki and Kd Values for PID. The values are in radians.
    pub async fn tune(&self, kp: f64, ki: f64, kd: f64, leeway: f64) {
        let mut pid_values = self.pid_values.lock().await;
        pid_values.kp = kp;
        pid_values.ki = ki;
        pid_values.kd = kd;
        pid_values.leeway = leeway;
    }

    /// Sets the maximum power the robot should move at. The maximum value is 12.0
    /// while the minimum value is -12.0 (reverse).
    pub async fn set_maximum_power(&self, maximum_power: f64) {
        let mut pid_values = self.pid_values.lock().await;
        pid_values.maxpwr = maximum_power;
    }

    /// Makes the robot travel in a straight line
    pub async fn travel(&self, distance: f64, timeout: u64, afterdelay: u64) {
        let r = (distance *
            (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
            2.0 *
            PI) /
            self.drivetrain_config.wheel_diameter;
        let mut s = self.pid_values.lock().await;
        s.active = true;
        s.target_right += r;
        s.target_left += r;
        timeout_wait(&self.pid_values, timeout).await;
        {
            let mut s = self.pid_values.lock().await;
            s.active = false;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }

    /// Rotates the robot by a certain number for degrees
    pub async fn rotate(&self, degrees: f64, timeout: u64, afterdelay: u64) {
        let target = PI * self.drivetrain_config.track_width / (360.0 / degrees);
        self.rotate_raw(target, timeout, afterdelay).await;
    }

    /// Rotates the robot. The value should be in the number of inches
    /// one side of the robot must rotate.
    pub async fn rotate_raw(&self, distance: f64, timeout: u64, afterdelay: u64) {
        let r = (distance *
            (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
            2.0 *
            PI) /
            self.drivetrain_config.wheel_diameter;
        {
            let mut s = self.pid_values.lock().await;
            s.active = true;
            s.target_left += r;
            s.target_right += -r;
        }
        timeout_wait(&self.pid_values, timeout).await;
        {
            let mut s = self.pid_values.lock().await;
            s.active = false;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }

    /// Rotates the robot using the IMU (Inertial Sensor) for more accurate
    /// and precise turning.
    pub async fn rotate_imu(
        &self,
        degrees: f64,
        imu: &InertialSensor,
        timeout: u64,
        afterdelay: u64,
    ) {
        let start_time = user_uptime().as_millis();
        let mut prev_angle = 0.0;
        let mut delta_angle;
        let mut angle;
        let mut s = self.pid_values.lock().await;
        s.active = true;
        loop {
            angle = degrees - get_heading(&imu);
            delta_angle = angle - prev_angle;
            prev_angle = angle;
            let distance = PI * self.drivetrain_config.track_width / (360.0 / delta_angle);
            let r = (distance *
                (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
                2.0 *
                PI) /
                self.drivetrain_config.wheel_diameter;
            {
                let mut s = self.pid_values.lock().await;
                s.target_left += r;
                s.target_right += -r;
                if !s.active {
                    break;
                }
            }
            if user_uptime().as_millis() >= start_time + timeout as u128 {
                let mut s = self.pid_values.lock().await;
                s.active = false;
                break;
            }
            sleep(Duration::from_millis(LOOPRATE)).await;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }

    /// Swings the robot by moving only one side of the robot forward or backward
    pub async fn swing(&self, degrees: f64, right: bool, timeout: u64, afterdelay: u64) {
        let target = 2.0 * PI * self.drivetrain_config.track_width / (360.0 / degrees);
        self.swing_raw(target.abs(), right, timeout, afterdelay)
            .await;
    }

    /// Swings the robot by moving only one side of the robot forward or backward with values in inches
    /// The value should be in the number of inches the side of the robot must rotate.
    pub async fn swing_raw(&self, distance: f64, right: bool, timeout: u64, afterdelay: u64) {
        let r = (distance *
            (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
            2.0 *
            PI) /
            self.drivetrain_config.wheel_diameter;
        {
            let mut s = self.pid_values.lock().await;
            s.active = true;
            s.target_left += r * if right { 1.0 } else { 0.0 };
            s.target_right += r * if right { 0.0 } else { 1.0 };
        }
        timeout_wait(&self.pid_values, timeout).await;
        {
            let mut s = self.pid_values.lock().await;
            s.active = false;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }

    /// Swings the robot by moving only one side of the robot forward or backward
    /// using the IMU (Inertial Sensor) for more accurate
    /// and precise turning.
    pub async fn swing_imu(
        &self,
        degrees: f64,
        imu: &InertialSensor,
        right: bool,
        timeout: u64,
        afterdelay: u64,
    ) {
        let start_time = user_uptime().as_millis();
        let mut prev_angle = 0.0;
        let mut delta_angle;
        let mut angle;
        let mut s = self.pid_values.lock().await;
        s.active = true;
        loop {
            angle = degrees - get_heading(&imu);
            delta_angle = angle - prev_angle;
            prev_angle = angle;
            let distance = PI * self.drivetrain_config.track_width / (360.0 / delta_angle);
            let r = (distance *
                (self.drivetrain_config.driving_gear / self.drivetrain_config.driven_gear) *
                2.0 *
                PI) /
                self.drivetrain_config.wheel_diameter;
            {
                let mut s = self.pid_values.lock().await;
                s.target_left += r * if right { 1.0 } else { 0.0 };
                s.target_right += r * if right { 0.0 } else { 1.0 };
                if !s.active {
                    break;
                }
            }
            if user_uptime().as_millis() >= start_time + timeout as u128 {
                let mut s = self.pid_values.lock().await;
                s.active = false;
                break;
            }
            sleep(Duration::from_millis(LOOPRATE)).await;
        }
        sleep(Duration::from_millis(afterdelay)).await;
    }
}

/// **The PID Movement Controller**
///
/// Initialize an instance of this to control the robot using PID.
///
/// # Examples
///
/// Creating a PIDMovement Instance
/// ```
/// fn new_pid() -> PIDMovement {
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
///     let values = PIDValues {
///         kp:           0.5,
///         kd:           0.1,
///         ki:           0.0,
///         leeway:       0.02,
///         maxpwr:       12.0,
///         active:       true,
///         target_left:  0.0,
///         target_right: 0.0,
///     };
///     let PID_controller = PIDMovement {
///         drivetrain:        dt,
///         drivetrain_config: config,
///         pid_values:        Arc::new(Mutex::new(values)),
///     };
/// }
/// ```
pub struct PIDMovement {
    pub drivetrain:        Differential,
    pub drivetrain_config: DrivetrainConfig,
    pub pid_values:        Arc<Mutex<PIDValues>>,
}

/// A Struct for PID values that will be altered throughout the Autonomous
pub struct PIDValues {
    pub kp:           f64,
    pub ki:           f64,
    pub kd:           f64,
    pub leeway:       f64,
    pub maxpwr:       f64,
    pub active:       bool,
    pub target_left:  f64,
    pub target_right: f64,
}

/// The Drivtrain's Physical Configuration that will be used for calculations
pub struct DrivetrainConfig {
    /// The wheel diameter in inches.
    /// Most teams use 2.75", 3.25" or sometimes 4".
    /// The older large omni wheels were 4.15".
    pub wheel_diameter: f64,
    /// The size of the driving gear. This can
    /// be in any units as long as the same units
    /// are used for the driven gear value.
    pub driving_gear:   f64,
    /// The size of the driven gear. This can
    /// be in any units as long as the same units
    /// are used for the driving gear value.
    pub driven_gear:    f64,
    // The width from the right wheels to the left.
    pub track_width:    f64,
}

async fn timeout_wait(pid_values: &Arc<Mutex<PIDValues>>, timeout: u64) {
    let start_time = user_uptime().as_millis();

    loop {
        {
            let s = pid_values.lock().await;
            if !s.active {
                break;
            }
        }

        if user_uptime().as_millis() >= start_time + timeout as u128 {
            let mut s = pid_values.lock().await;
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
