use vexide::smart::{PortError, motor::Motor};

use super::*;

pub trait MotorMapper {
    fn from_digital_input(
        &mut self,
        input: DigitalInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError>;

    fn from_analog_input(
        &mut self,
        input: AnalogInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError>;

    fn from_dual_input(
        &mut self,
        first_input: DigitalInput,
        second_input: DigitalInput,
        first_active_power: f64,
        second_active_power: f64,
        passive_power: f64,
    ) -> Result<(), PortError>;
}

impl MotorMapper for Motor {
    fn from_digital_input(
        &mut self,
        input: DigitalInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError> {
        if input.as_bool()? {
            self.set_voltage(high_power)?;
        } else {
            self.set_voltage(low_power)?;
        }
        Ok(())
    }

    fn from_analog_input(
        &mut self,
        input: AnalogInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError> {
        let delta = high_power - low_power;
        self.set_voltage(input.clamped()? * delta + low_power)?;
        Ok(())
    }

    fn from_dual_input(
        &mut self,
        first_input: DigitalInput,
        second_input: DigitalInput,
        first_active_power: f64,
        second_active_power: f64,
        passive_power: f64,
    ) -> Result<(), PortError> {
        if first_input.as_bool()? {
            self.set_voltage(first_active_power)?;
        } else if second_input.as_bool()? {
            self.set_voltage(second_active_power)?;
        } else {
            self.set_voltage(passive_power)?;
        }
        Ok(())
    }
}
