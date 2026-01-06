use std::sync::Arc;

use vexide::{math::Angle, sync::Mutex, task::spawn};

use super::{algorithm::*, devices::Pose};
use crate::motion::odom::devices::TrackerMech;
pub struct OdomTracker {
    pub trackermech: TrackerMech,
    pub global_pose: Arc<Mutex<Pose>>,
}

impl OdomTracker {
    pub fn init(&self) {
        let pose = self.global_pose.clone();
        let trackers = self.trackermech.clone();
        let mainloop = spawn(async move {
            odomloop(&trackers, &pose).await;
        });
        mainloop.detach();
    }

    pub fn new(tm: TrackerMech) -> Self {
        Self {
            trackermech: tm,
            global_pose: Arc::new(Mutex::new(Pose::new(0.0, 0.0, Angle::from_radians(0.0)))),
        }
    }

    pub fn from_pose(tm: TrackerMech, pose: Pose) -> Self {
        Self {
            trackermech: tm,
            global_pose: Arc::new(Mutex::new(pose)),
        }
    }

    pub async fn reset_origin(&self) {
        let mut gp = self.global_pose.lock().await;
        *gp = Pose::new(0.0, 0.0, Angle::from_radians(0.0));
    }

    pub async fn reset_from_pose(&self, pose: Pose) {
        let mut gp = self.global_pose.lock().await;
        *gp = pose;
    }
}
