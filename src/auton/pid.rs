use core::time::Duration;

use vexide::{prelude::*, sync::Mutex};

use crate::drivetrain;

const LOOPRATE: u64 = 10;

impl PIDControl {
    pub async fn pid_loop(globals: &Mutex<PIDControl>, drivetrain: drivetrain::Differential) {
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
            let (
                target,
                pwr,
                lin_kp,
                lin_kd,
                lin_ki,
                lin_leeway,
                ang_kp,
                ang_kd,
                ang_ki,
                ang_leeway,
                state,
            ) = {
                let s = globals.lock().await;
                (
                    s.target,
                    s.maxpwr,
                    s.linear_pid.kp,
                    s.linear_pid.kd,
                    s.linear_pid.ki,
                    s.linear_pid.leeway,
                    s.angular_pid.kp,
                    s.angular_pid.kd,
                    s.angular_pid.ki,
                    s.angular_pid.leeway,
                    s.state,
                )
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
            let error_left = target - currs_left;
            let error_right = target - currs_right;

            ierror_left += error_left * dt;
            ierror_right += error_right * dt;
            let mut u_left = 0.0;
            let mut u_right = 0.0;
            if state == PIDType::Linear {
                let ki = lin_ki;
                if ki != 0.0 {
                    let i_max = pwr.abs() / ki.abs();
                    ierror_left = ierror_left.clamp(-i_max, i_max);
                    ierror_right = ierror_right.clamp(-i_max, i_max);
                }

                let derror_left = (error_left - perror_left) / dt;
                let derror_right = (error_right - perror_right) / dt;

                u_left = lin_kp * error_left + ki * ierror_left + lin_kd * derror_left;
                u_right = lin_kp * error_right + ki * ierror_right + lin_kd * derror_right;

                u_left = abscap(u_left, pwr.abs());
                u_right = abscap(u_right, pwr.abs());
            } else if state == PIDType::Angular {
                let ki = ang_ki;
                if ki != 0.0 {
                    let i_max = pwr.abs() / ki.abs();
                    ierror_left = ierror_left.clamp(-i_max, i_max);
                    ierror_right = ierror_right.clamp(-i_max, i_max);
                }

                let derror_left = (error_left - perror_left) / dt;
                let derror_right = (error_right - perror_right) / dt;

                u_left = ang_kp * error_left + ki * ierror_left + ang_kd * derror_left;
                u_right = ang_kp * error_right + ki * ierror_right + ang_kd * derror_right;

                u_left = abscap(u_left, pwr.abs());
                u_right = -abscap(u_right, pwr.abs());
            } else if state == PIDType::Swing {
                let ki = ang_ki;
                if ki != 0.0 {
                    let i_max = pwr.abs() / ki.abs();
                    ierror_left = ierror_left.clamp(-i_max, i_max);
                    ierror_right = ierror_right.clamp(-i_max, i_max);
                }

                let derror_left = (error_left - perror_left) / dt;
                let derror_right = (error_right - perror_right) / dt;

                u_left = ang_kp * error_left + ki * ierror_left + ang_kd * derror_left;
                u_right = ang_kp * error_right + ki * ierror_right + ang_kd * derror_right;
                if target >= 0.0 {
                    u_left = abscap(u_left, pwr.abs());
                    u_right = 0.0;
                } else {
                    u_left = 0.0;
                    u_right = abscap(u_right, pwr.abs());
                }
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
            if state == PIDType::Linear {
                in_band = error_left.abs() < lin_leeway && error_right.abs() < lin_leeway;
            } else {
                in_band = error_left.abs() < ang_leeway && error_right.abs() < ang_leeway;
            }
            if in_band {
                let mut s = globals.lock().await;
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

pub struct PIDControl {
    linear_pid:  PID,
    angular_pid: PID,
    state:       PIDType,
    target:      f64,
    maxpwr:      f64,
    active:      bool,
}

pub struct PID {
    kp:     f64,
    ki:     f64,
    kd:     f64,
    leeway: f64,
}

#[derive(Clone, Copy, PartialEq)]
pub enum PIDType {
    Linear,
    Angular,
    Swing,
}
