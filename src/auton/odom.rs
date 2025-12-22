use std::{cell::RefCell, rc::Rc, sync::Arc, time::Duration};

use log::{info, warn};
use vexide::{
    math::Angle,
    prelude::{AdiOpticalEncoder, InertialSensor},
    sync::Mutex,
    task::spawn,
    time::sleep,
};

use crate::auton::{arcpid::ArcPIDMovement, pid::PIDMovement};

const LOOPRATE: u64 = 5;
const TIMEOUT: u64 = 10000;
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
            vertical_rad = vertical
                .device
                .position()
                .unwrap_or_else(|e| {
                    warn!("ADI Encoder Error: {}", e);
                    Angle::from_radians(0.0)
                })
                .as_radians()
                .clone();
            delta_v = (vertical_rad * vertical.wheel_diameter / 2.0) - prev_dist_v;
            wheel_dia_v = vertical.wheel_diameter.clone();
            offset_v = vertical.offset.clone();
        }

        // Horizontal Tracking Wheel Calculations
        let (horizontal_rad, delta_h, wheel_dia_h, offset_h);
        {
            let horizontal = &trackers.borrow().horizontal;
            horizontal_rad = horizontal
                .device
                .position()
                .unwrap_or_else(|e| {
                    warn!("ADI Encoder Error: {}", e);
                    Angle::from_radians(0.0)
                })
                .as_radians()
                .clone();
            delta_h = (horizontal_rad * horizontal.wheel_diameter / 2.0) - prev_dist_h;
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
    pub fn init(&self) {
        let thread_clone = self.odometry_values.clone();
        let thread_trackers = self.trackers.clone();
        let mainloop = spawn(async move {
            odom_tracker(&thread_clone, &thread_trackers).await;
        });
        mainloop.detach();
    }

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

    pub async fn travel(&self, distance: f64) {
        if let Some(pid) = &self.pid {
            pid.travel(distance, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot travel without Movement Algorithm (PID needed)")
        }
    }

    pub async fn arc_travel(&self, distance: f64, offset: f64) {
        if let Some(arc_pid) = &self.arc_pid {
            arc_pid.travel(distance, offset, TIMEOUT, AFTERDELAY).await;
        } else {
            warn!("Cannot travel without Movement Algorithm (Arc PID needed)")
        }
    }

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

/// Struct that will hold all Global Odometry Numbers
pub struct OdomValues {
    pub global_x:       f64,
    pub global_y:       f64,
    pub global_heading: f64,
}

/// Tracking Wheel Data
pub struct WheelTracker {
    device:         AdiOpticalEncoder,
    wheel_diameter: f64,
    offset:         f64,
}

/// Hardware that will be used by Odometry
pub struct Trackers {
    vertical:   WheelTracker,
    horizontal: WheelTracker,
    imu:        InertialSensor,
}

/// The main Odometry Instace
pub struct OdomMovement {
    pub odometry_values: Arc<Mutex<OdomValues>>,
    pub trackers:        Rc<RefCell<Trackers>>,
    pub pid:             Option<PIDMovement>,
    pub arc_pid:         Option<ArcPIDMovement>,
}
