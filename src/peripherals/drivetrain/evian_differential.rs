//! Compatibility with Evian's Differential Drivetrain

// TODO: Add trackable trait
use evian::{
    drivetrain::model::Differential,
    prelude::{Arcade, Tank},
};
use vexide::prelude::Controller;

use crate::peripherals::drivetrain::{Drivable, DrivetrainError};

impl Drivable for Differential {
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
