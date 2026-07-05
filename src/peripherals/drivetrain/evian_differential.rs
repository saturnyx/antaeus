//! Compatibility with Evian's Differential Drivetrain

use evian::{
    drivetrain::model::Differential as EvianDifferential,
    math::Angle,
    prelude::{Arcade, Tank},
};
use vexide::prelude::Controller;

use crate::{
    motion::localization::tracker::devices::{Trackable, TrackingSensorError},
    peripherals::drivetrain::{Drivable, DrivetrainError},
};

impl Drivable for EvianDifferential {
    fn tank(&mut self, controller: &Controller) -> Result<(), super::DrivetrainError> {
        let state = controller.state()?;
        match self.drive_tank(state.left_stick.y(), state.right_stick.y()) {
            Ok(()) => Ok(()),
            Err(e) => Err(DrivetrainError::PortError { source: e }),
        }
    }

    fn arcade(
        &mut self,
        controller: &vexide::prelude::Controller,
    ) -> Result<(), super::DrivetrainError> {
        let state = controller.state()?;
        match self.drive_arcade(state.left_stick.y(), state.right_stick.x()) {
            Ok(()) => Ok(()),
            Err(e) => Err(DrivetrainError::PortError { source: e }),
        }
    }

    fn reverse_tank(
        &mut self,
        controller: &vexide::prelude::Controller,
    ) -> Result<(), super::DrivetrainError> {
        let state = controller.state()?;

        match self.drive_tank(-state.right_stick.y(), -state.left_stick.y()) {
            Ok(()) => Ok(()),
            Err(e) => Err(DrivetrainError::PortError { source: e }),
        }
    }

    fn reverse_arcade(
        &mut self,
        controller: &vexide::prelude::Controller,
    ) -> Result<(), super::DrivetrainError> {
        let state = controller.state()?;

        match self.drive_arcade(-state.left_stick.y(), -state.right_stick.x()) {
            Ok(()) => Ok(()),
            Err(e) => Err(DrivetrainError::PortError { source: e }),
        }
    }
}

impl Trackable for EvianDifferential {
    fn reset_track_position(&mut self) -> Result<(), TrackingSensorError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.reset_position()?;
        }

        for motor in right.as_mut() {
            motor.reset_position()?;
        }

        Ok(())
    }

    fn set_track_position(
        &mut self,
        position: evian::math::Angle,
    ) -> Result<(), TrackingSensorError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;

        for motor in left.as_mut() {
            motor.set_position(position)?
        }

        for motor in right.as_mut() {
            motor.set_position(position)?
        }

        Ok(())
    }

    fn track_position(&mut self) -> Result<evian::math::Angle, TrackingSensorError> {
        let mut left = self.left.try_borrow_mut()?;
        let mut right = self.right.try_borrow_mut()?;
        let mut angle: Angle = Angle::from_degrees(0.0);
        let mut denom: f64 = 0.0;
        for motor in left.as_mut() {
            angle += motor.position()?;
            denom += 1.0;
        }
        for motor in right.as_mut() {
            angle += motor.position()?;
            denom += 1.0;
        }

        Ok(angle / denom)
    }
}
