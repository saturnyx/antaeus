use vexide::prelude::AdiDigitalOut;

use super::*;
pub trait AdiOutMapper {
    fn from_digital_input(&mut self, input: DigitalInput, inverse: bool) -> Result<(), PortError>;
    fn toggle_from_digital_input(
        &mut self,
        input: DigitalInput,
        inverse: bool,
    ) -> Result<(), PortError>;
    fn from_analog_input(
        &mut self,
        input: AnalogInput,
        threshold: f64,
        inverse: bool,
    ) -> Result<(), PortError>;
    fn toggle_from_analog_input(
        &mut self,
        input: AnalogInput,
        threshold: f64,
        inverse: bool,
    ) -> Result<(), PortError>;
}

impl AdiOutMapper for AdiDigitalOut {
    fn from_digital_input(&mut self, input: DigitalInput, inverse: bool) -> Result<(), PortError> {
        let input = input.as_bool()? ^ inverse;
        if input {
            self.set_high()?;
        } else {
            self.set_low()?;
        }
        Ok(())
    }

    fn toggle_from_digital_input(
        &mut self,
        input: DigitalInput,
        inverse: bool,
    ) -> Result<(), PortError> {
        let input = input.as_bool_now()? ^ inverse;
        if input {
            self.toggle()?;
        }
        Ok(())
    }

    fn from_analog_input(
        &mut self,
        input: AnalogInput,
        threshold: f64,
        inverse: bool,
    ) -> Result<(), PortError> {
        let input = (input.clamped()? > threshold) ^ inverse;
        if input {
            self.set_high()?;
        } else {
            self.set_low()?;
        }
        Ok(())
    }

    fn toggle_from_analog_input(
        &mut self,
        input: AnalogInput,
        threshold: f64,
        inverse: bool,
    ) -> Result<(), PortError> {
        let input = (input.clamped()? > threshold) ^ inverse;
        if input {
            self.toggle()?;
        }
        Ok(())
    }
}
