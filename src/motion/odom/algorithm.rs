use std::sync::Arc;

use log::warn;
use vexide::{math::Angle, prelude::InertialSensor, sync::Mutex};

use super::devices::{Pose, TrackerMech};

pub async fn odomloop(tm: &TrackerMech, global_pose: &Arc<Mutex<Pose>>) {
    let (mut prev_t, mut prev_v, mut prev_h) = (Angle::from_radians(0.0), 0.0, 0.0);
    let mut pose = Pose::new(0.0, 0.0, Angle::from_radians(0.0));
    loop {
        let t = get_imu_angle(&tm.imu).await;
        let v = tm.vertical_tracker.dist().await;
        let h = tm.horizontal_tracker.dist().await;
        let (delta_t, delta_v, delta_h) = (t - prev_t, v - prev_v, h - prev_h);
        (prev_t, prev_v, prev_h) = (t, v, h);

        // A really rare case scenario where the robot moves in an exactly straight line
        if delta_t == Angle::from_radians(0.0) {
            (pose.t, pose.x, pose.y) = (get_imu_heading(&tm.imu).await, h, v)
        } else {
            // Here we go... (somewhat complex math)
            // Equation 6 of http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
            pose.t = get_imu_heading(&tm.imu).await;
            pose.x = local_calc(t, delta_t, delta_h, tm.horizontal_tracker.offset);
            pose.y = local_calc(t, delta_t, delta_v, tm.vertical_tracker.offset);
        }

        let avg_t = get_imu_angle(&tm.imu).await + (delta_t / 2.0);
        let (global_delta_x, global_delta_y) = rotate_vec(pose.x, pose.y, avg_t);
        update_pose(
            &global_pose,
            Pose::new(global_delta_x, global_delta_y, get_imu_heading(&tm.imu).await),
        )
        .await;
    }
}

async fn get_imu_angle(imu: &Arc<Mutex<InertialSensor>>) -> Angle {
    imu.lock().await.rotation().unwrap_or_else(|e| {
        warn!("IMU Error: {}", e);
        Angle::from_radians(0.0)
    })
}

async fn get_imu_heading(imu: &Arc<Mutex<InertialSensor>>) -> Angle {
    let angle = imu.lock().await.euler();
    if let Ok(a) = angle {
        a.b
    } else if let Err(e) = angle {
        warn!("IMU Error: {}", e);
        Angle::from_radians(0.0)
    } else {
        warn!("IMU Error");
        Angle::from_radians(0.0)
    }
}

fn local_calc(t: Angle, delta_t: Angle, delta_dist: f64, offset: f64) -> f64 {
    2.0 * (t / 2.0).sin() * (delta_dist / delta_t.as_radians() + offset)
}

fn rotate_vec(x: f64, y: f64, t: Angle) -> (f64, f64) {
    let new_x = x * t.cos() - y * t.sin();
    let new_y = x * t.sin() + y * t.cos();
    (new_x, new_y)
}

async fn update_pose(global_pose: &Arc<Mutex<Pose>>, local_pose: Pose) {
    let mut gp = global_pose.lock().await;
    gp.x += local_pose.x;
    gp.y += local_pose.y;
    gp.t = local_pose.t;
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rotate_vec_test_basic() {
        let (x, y) = (1.0, 3.0);
        let (new_x, new_y) = rotate_vec(x, y, Angle::from_degrees(90.0));
        let expected = (-3.0, 1.0);
        let tolerance = 1e-10;
        assert!((new_x - expected.0).abs() < tolerance);
        assert!((new_y - expected.1).abs() < tolerance);
    }

    #[test]
    fn rotate_vec_test_adv() {
        let (x, y) = (4.65, 7.89);
        let (new_x, new_y) = rotate_vec(x, y, Angle::from_radians(0.34));
        let expected = (1.7383, 8.9938);
        let tolerance = 0.2;
        eprint!("{}, {}", new_x, new_y);
        assert!((new_x - expected.0).abs() < tolerance);
        assert!((new_y - expected.1).abs() < tolerance);
    }
}
