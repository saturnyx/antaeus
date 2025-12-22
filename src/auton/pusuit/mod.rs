pub mod algorithm;
pub mod geo;
use crate::auton::odom::OdomMovement;
pub struct PurePursuit {
    pub lookahead: f64,
}

impl PurePursuit {
    pub async fn follow(odom: OdomMovement, path: geo::Path, lookahead: f64) {
        let mut run = true;
        while run {
            let odometry_values = odom.odometry_values.lock().await;
            let (x, y) = (odometry_values.global_x, odometry_values.global_y);
            let cir = geo::Circle {
                x: x,
                y: y,
                r: lookahead,
            };
            let target = algorithm::pp_target(path.clone(), cir);
            odom.arc_point(target.x, target.y).await;

            if let Some(arcpid) = odom.arc_pid.clone() {
                let values = arcpid.arcpid_values.lock().await;
                run = values.active;
            }
        }
    }
}
