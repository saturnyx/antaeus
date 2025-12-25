/// Math calculations for CBP-Algorithm
mod algorithm;
/// A small geometry library
pub mod geo;
use crate::motion::odom::OdomMovement;

/// # Candidate-Based Pusuit Algorithm
/// A more robust variant of pure pusuit
pub struct Pursuit {
    pub lookahead: f64,
}

impl Pursuit {
    /// The main function of the CBP-Algorithm.
    ///
    /// It uses an instance of `OdomMovement`, and lookahead distance
    /// to follow a path.
    pub async fn follow(&self, odom: OdomMovement, path: geo::Path) {
        let mut run = true;
        while run {
            let odometry_values = odom.odometry_values.lock().await;
            let (x, y) = (odometry_values.global_x, odometry_values.global_y);
            let cir = geo::Circle {
                x: x,
                y: y,
                r: self.lookahead,
            };
            let target = algorithm::pursuit_target(path.clone(), cir);
            odom.arc_point(target.x, target.y).await;

            if let Some(arcpid) = odom.arc_pid.clone() {
                let values = arcpid.arcpid_values.lock().await;
                run = values.active;
            }
        }
    }
}
