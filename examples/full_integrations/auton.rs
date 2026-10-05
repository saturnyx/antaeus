use antaeus::{
    motion::{pursuit::steer, *},
    utils::{geo, units::Length},
};

use crate::{hardware::Robot, integrations};
pub async fn main_auton(robot: &mut Robot) {
    let mut path = geo::Path::origin();
    path.add(geo::Point::new(Length::from_inches(20.0), Length::from_inches(20.0)));
    path.add(geo::Point::new(Length::from_inches(-20.0), Length::from_inches(20.0)));
    path.add(geo::Point::origin());

    let mut steering = steer::basic::BasicSteer {
        tolerance: Length::from_inches(0.5),
    };

    let mut odomtrack = integrations::DummyOdom;
    let pursuit = pursuit::Pursuit {
        lookahead: Length::from_inches(10.0),
    };
    let _ = pursuit
        .follow(&mut odomtrack, &robot.dt, &mut steering, path.clone())
        .await;
}
