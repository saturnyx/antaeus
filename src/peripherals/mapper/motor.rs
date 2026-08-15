//! Mappers for smart motors
//!
//! This module provides traits and implementations for mapping digital and
//! analog inputs to motors, allowing for easy control of mechanisms like
//! pneumatic solenoids using controller inputs or other sensors.

use vexide::smart::{PortError, motor::Motor};

use super::*;

/// A trait for mapping digital and analog inputs to motors, allowing for easy
/// control of mechanisms using controller inputs or other sensors.
pub trait MotorMapper {
    /// Maps a digital input to a motor's voltage
    ///
    /// # Arguments
    /// - `input`: The digital input from another peripheral (e.g. controller
    ///   button)
    /// - `high_power`: The voltage the motor will be set to when the input is
    ///   active
    fn digital_input(
        &mut self,
        input: DigitalInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError>;

    /// Maps a analog input to a motor's voltage
    ///
    /// # Arguments
    /// - `input`: The analog input from another peripheral (e.g. controller
    ///   joystick)
    /// - `high_power`: The voltage the motor will be set to when the input is
    ///   at its maximum value (e.g. joystick fully pushed forward)
    /// - `low_power`: The voltage the motor will be set to when the input is at
    ///   its minimum value (e.g. joystick fully pulled back)
    ///
    /// (The voltage will be linearly interpolated between low_power and
    /// high_power based on the input value)
    fn analog_input(
        &mut self,
        input: AnalogInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError>;

    /// Maps two digital inputs to a motor's voltage
    ///
    /// # Arguments
    /// - `first_input`: The first digital input from another peripheral (e.g.
    ///   controller button)
    /// - `second_input`: The second digital input from another peripheral (e.g.
    ///   controller button)
    /// - `first_active_power`: The voltage the motor will be set to when the
    ///   first input is active and the second input is not active
    /// - `second_active_power`: The voltage the motor will be set to when the
    ///   second input is active and the first input is not active
    /// - `passive_power`: The voltage the motor will be set to when neither
    ///   input is active
    ///
    /// (If both inputs are active, the motor will be set to first_active_power)
    fn dual_input(
        &mut self,
        first_input: DigitalInput,
        second_input: DigitalInput,
        first_active_power: f64,
        second_active_power: f64,
        passive_power: f64,
    ) -> Result<(), PortError>;
}

impl MotorMapper for Motor {
    fn digital_input(
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

    fn analog_input(
        &mut self,
        input: AnalogInput,
        high_power: f64,
        low_power: f64,
    ) -> Result<(), PortError> {
        let delta = high_power - low_power;
        self.set_voltage(input.clamped()? * delta + low_power)?;
        Ok(())
    }

    fn dual_input(
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
