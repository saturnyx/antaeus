use std::{cell::RefCell, rc::Rc, sync::Arc, time::Duration};

use log::info;
use vexide::{
    smart::motor::{BrakeMode, Motor},
    sync::Mutex,
    task::*,
    time::*,
};

const LOOPRATE: u64 = 5; // The PID will run with a loop rate of 5 millisecs

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

    /// Sets the target that the motor should move to
    pub async fn set_target(&self, target: f64) {
        {
            let mut s = self.pid_values.lock().await;
            s.active = true;
            s.target = target;
        }
    }
}

/// **The PID Movement Controller**
///
/// Initialize an instance of this to control the robot using PID.
pub struct SinglePIDMovement {
    pub motorgroup: Rc<RefCell<dyn AsMut<[Motor]>>>,
    pub pid_values: Arc<Mutex<SinglePIDValues>>,
}

/// A Struct for PID values that will be altered throughout the Autonomous
pub struct SinglePIDValues {
    pub kp:        f64,
    pub ki:        f64,
    pub kd:        f64,
    pub tolerance: f64,
    pub maxpwr:    f64,
    pub active:    bool,
    pub target:    f64,
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
