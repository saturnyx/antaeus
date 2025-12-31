//! Standalone PID controller for single motor groups.
//!
//! This module provides a PID controller for mechanisms that operate
//! independently from the drivetrain, such as arms, lifts, or flywheels.
//!
//! # Use Cases
//!
//! - **Arms/Lifts**: Position control for mechanisms that move to set heights.
//! - **Flywheels**: Speed control for shooting mechanisms (using velocity as position).
//! - **Intakes**: Position control for deploy/retract sequences.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::motion::pid::singlepid::{SinglePIDMovement, SinglePIDValues};
//!
//! let arm_pid = SinglePIDMovement { /* ... */ };
//! arm_pid.init();
//!
//! // Tune for the mechanism
//! arm_pid.tune(0.8, 0.0, 0.2, 0.01).await;
//!
//! // Move to target position (in radians)
//! arm_pid.set_target(3.14).await;
//! ```

use std::{cell::RefCell, rc::Rc, sync::Arc, time::Duration};

use log::info;
use vexide::{
    smart::motor::{BrakeMode, Motor},
    sync::Mutex,
    task::*,
    time::*,
};

/// Loop rate for the PID control task in milliseconds.
const LOOPRATE: u64 = 5;

async fn single_pid_loop(
    pidvalues: &Arc<Mutex<SinglePIDValues>>,
    motorgroup: Rc<RefCell<dyn AsMut<[Motor]>>>,
) {
    info!("PID Control Loop Started");
    // Set brake mode and reset positions for motors
    {
        let mut motors = motorgroup.borrow_mut();
        let slice = motors.as_mut();
        for motor in slice.iter_mut() {
            let _ = motor.brake(BrakeMode::Brake);
            let _ = motor.reset_position();
        }
    }

    let mut perror = 0.0;
    let mut ierror = 0.0;

    // seconds per loop from configured looprate in ms
    let dt = (LOOPRATE as f64) / 1000.0;

    loop {
        let (target, pwr, kp, kd, ki, tolerance) = {
            let s = pidvalues.lock().await;
            (s.target, s.maxpwr, s.kp, s.kd, s.ki, s.tolerance)
        };

        let currs = {
            let mut motors = motorgroup.borrow_mut();
            let slice = motors.as_mut();
            let sum: f64 = slice
                .iter()
                .map(|motor| motor.position().unwrap_or_default().as_radians())
                .sum();
            sum / slice.len() as f64
        };

        let error = target - currs;

        ierror += error * dt;
        let mut u;
        let ki = ki;
        if ki != 0.0 {
            let i_max = pwr.abs() / ki.abs();
            ierror = ierror.clamp(-i_max, i_max);
        }

        let derror = (error - perror) / dt;

        u = kp * error + ki * ierror + kd * derror;

        u = abscap(u, pwr.abs());

        // Set voltage for motors
        {
            let mut motors = motorgroup.borrow_mut();
            let slice = motors.as_mut();
            for motor in slice.iter_mut() {
                let _ = motor.set_voltage(u);
            }
        }

        let in_band;
        in_band = error.abs() < tolerance;

        if in_band {
            let mut s = pidvalues.lock().await;
            s.active = false;

            // Stop motors
            {
                let mut motors = motorgroup.borrow_mut();
                let slice = motors.as_mut();
                for motor in slice.iter_mut() {
                    let _ = motor.set_voltage(0.0);
                }
            }

            ierror = 0.0;
        }
        perror = error;
        sleep(Duration::from_millis(LOOPRATE)).await;
    }
}

impl SinglePIDMovement {
    /// Initializes a PID Loop.
    /// The PID movements will require a PID loop to run as a seperate task or thread.
    /// It is necessary to initialize the PID before running any movements.

    pub fn init(&self) {
        let mutex_clone = self.pid_values.clone();
        let motorgroup = self.motorgroup.clone();
        let mainloop = spawn(async move {
            single_pid_loop(&mutex_clone, motorgroup).await;
        });
        mainloop.detach();
    }

    /// Set the tolerance, Kp, Ki and Kd Values for PID. The values are in radians.
    pub async fn tune(&self, kp: f64, ki: f64, kd: f64, tolerance: f64) {
        let mut pid_values = self.pid_values.lock().await;
        pid_values.kp = kp;
        pid_values.ki = ki;
        pid_values.kd = kd;
        pid_values.tolerance = tolerance;
    }

    /// Sets the maximum power the robot should move at. The maximum value is 12.0
    /// while the minimum value is -12.0 (reverse).
    pub async fn set_maximum_power(&self, maximum_power: f64) {
        let mut pid_values = self.pid_values.lock().await;
        pid_values.maxpwr = maximum_power;
    }

    /// Sets the target position for the motor group.
    ///
    /// The motors will move to this position and hold. The target
    /// is specified in radians of motor rotation.
    ///
    /// # Arguments
    ///
    /// * `target` - Target position in radians.
    pub async fn set_target(&self, target: f64) {
        {
            let mut s = self.pid_values.lock().await;
            s.active = true;
            s.target = target;
        }
    }
}

/// PID controller for a single motor group.
///
/// Unlike [`PIDMovement`](super::pid::PIDMovement), which controls
/// a differential drivetrain, this controller manages a single group
/// of motors moving together.
///
/// # Example
///
/// ```ignore
/// let arm = SinglePIDMovement {
///     motorgroup: Rc::new(RefCell::new([arm_motor_1, arm_motor_2])),
///     pid_values: Arc::new(Mutex::new(SinglePIDValues { /* ... */ })),
/// };
/// arm.init();
/// ```
pub struct SinglePIDMovement {
    /// The motor group to control.
    pub motorgroup: Rc<RefCell<dyn AsMut<[Motor]>>>,
    /// Thread-safe container for PID runtime values.
    pub pid_values: Arc<Mutex<SinglePIDValues>>,
}

impl SinglePIDMovement {
    pub fn new(motorgroup: Vec<Motor>, pid_values: SinglePIDValues) -> SinglePIDMovement {
        SinglePIDMovement {
            motorgroup: Rc::new(RefCell::new(motorgroup)),
            pid_values: Arc::new(Mutex::new(pid_values)),
        }
    }
}

/// Runtime values for the single motor PID controller.
///
/// These values control the PID behavior and are updated during operation.
pub struct SinglePIDValues {
    /// Proportional gain.
    pub kp:        f64,
    /// Integral gain.
    pub ki:        f64,
    /// Derivative gain.
    pub kd:        f64,
    /// Error tolerance in radians.
    pub tolerance: f64,
    /// Maximum motor voltage (0-12 volts).
    pub maxpwr:    f64,
    /// Whether a movement is currently active.
    pub active:    bool,
    /// Target motor position in radians.
    pub target:    f64,
}

impl SinglePIDValues {
    /// Uses default ArcPID Values
    pub fn default() -> SinglePIDValues {
        SinglePIDValues {
            kp:        0.5,
            ki:        0.0,
            kd:        0.0,
            tolerance: 0.1,
            maxpwr:    12.0,
            active:    true,
            target:    0.0,
        }
    }

    pub fn new(kp: f64, ki: f64, kd: f64, tolerance: f64, maxpwr: f64) -> SinglePIDValues {
        SinglePIDValues {
            kp,
            ki,
            kd,
            tolerance,
            maxpwr,
            active: true,
            target: 0.0,
        }
    }
}

/// Clamps a value to the range [-cap, cap].
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
