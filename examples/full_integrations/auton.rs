use antaeus::{
    motion::{pursuit::control, *},
    utils::{geo, units::Length},
};

use crate::{hardware::Robot, integrations};
pub async fn main_auton(robot: &mut Robot) {
    let mut path = geo::Path::origin();
    path.add(geo::Point::new(Length::from_inches(20.0), Length::from_inches(20.0)));
    path.add(geo::Point::new(Length::from_inches(-20.0), Length::from_inches(20.0)));
    path.add(geo::Point::origin());

    let basic_ctrl = control::basic::BasicControl {
        track_width: Length::from_inches(13.9),
        tolerance:   Length::from_inches(0.5),
    };

    let mut odomtrack = integrations::DummyOdom;
    let pursuit = pursuit::Pursuit {
        lookahead: Length::from_inches(10.0),
    };
    let _ = pursuit
        .follow(&mut odomtrack, &robot.dt, &basic_ctrl, path.clone())
        .await;
}
