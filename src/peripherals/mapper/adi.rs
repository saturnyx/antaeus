//! Mappers for ADI devices (e.g. pneumatic solenoids)
//!
//! This module provides traits and implementations for mapping digital and
//! analog inputs to ADI devices, allowing for easy control of mechanisms like
//! pneumatic solenoids using controller inputs or other sensors.

use vexide::prelude::AdiDigitalOut;

use super::*;
/// A trait for mapping digital and analog inputs to ADI devices (e.g. pneumatic
/// solenoids), allowing for easy control of mechanisms using controller inputs
/// or other sensors.
pub trait AdiOutMapper {
    /// Maps a digital input to a device (e.g. pneumatic solenoid)
    ///
    /// # Arguments
    /// - `input`: The digital input from another peripheral (e.g. controller
    ///    button)
    /// - `inverse`: If true, the output will be inverted (e.g. high becomes
    ///    low)
    fn from_digital_input(&mut self, input: DigitalInput, inverse: bool) -> Result<(), PortError>;
    /// Maps a digital input to a device (e.g. pneumatic solenoid)
    /// (Toggles the device instead of direct mapping)
    ///
    /// # Arguments
    /// - `input`: The digital input from another peripheral (e.g. controller
    ///    button)
    /// - `inverse`: If true, the output will be inverted (e.g. high becomes
    ///    low)
    fn toggle_from_digital_input(
        &mut self,
        input: DigitalInput,
        inverse: bool,
    ) -> Result<(), PortError>;

    /// Maps a analog input to a ADI device (e.g. pneumatic solenoid)
    ///
    /// # Arguments
    /// - `input`: The analog input from another peripheral (e.g. controller
    ///    joystick)
    /// - `threshold`: The threshold the input has to reach in order for the
    ///    device to be activated
    /// - `inverse`: If true, the output will be inverted (e.g. high becomes
    ///    low)
    fn from_analog_input(
        &mut self,
        input: AnalogInput,
        threshold: f64,
        inverse: bool,
    ) -> Result<(), PortError>;

    /// Maps a analog input to a ADI device (e.g. pneumatic solenoid)
    /// (Toggles the device instead of direct mapping)
    ///
    /// # Arguments
    /// - `input`: The analog input from another peripheral (e.g. controller
    ///    joystick)
    /// - `threshold`: The threshold the input has to reach in order for the
    ///    device to be activated
    /// - `inverse`: If true, the output will be inverted (e.g. high becomes
    ///    low)
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
