//! A module for controlling the robot with feedback control.
//!
//! This module provides traits and implementations for controlling the robot's
//! drivetrain using feedback control, such as PID controllers.
use std::time::Duration;

use vexide::{math::Angle, prelude::InertialSensor};

use crate::utils::units::Length;

pub mod drive;
pub mod group;
pub mod primitive;

/// A trait for controlling the robot's drivetrain with feedback control.
pub trait DriveControl {
    /// Error type for DriveControl
    type Error;

    /// Commands the robot to travel a certain distance in a straight line
    fn travel(
        &mut self,
        target: Length,
        timeout: Duration,
    ) -> impl Future<Output = Result<AutoTickOutcome, Self::Error>> + '_;
    /// Commands the robot to rotate a certain angle on the same spot
    fn rotate(
        &mut self,
        angle: Angle,
        timeout: Duration,
    ) -> impl Future<Output = Result<AutoTickOutcome, Self::Error>> + '_;
    /// Commands the robot to pivot on one of its sides
    fn pivot(
        &mut self,
        angle: Angle,
        timeout: Duration,
    ) -> impl Future<Output = Result<AutoTickOutcome, Self::Error>> + '_;

    /// Commands the robot to travel a certain distance in a straight line using
    /// an IMU for heading correction
    fn imu_rotate<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &'a InertialSensor,
        angle_tolerance: Angle,
    ) -> impl Future<Output = Result<AutoTickOutcome, Self::Error>> + 'a;

    /// Commands the robot to rotate a certain angle on the same spot using an
    /// IMU for heading correction
    fn imu_pivot<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &'a InertialSensor,
        angle_tolerance: Angle,
    ) -> impl Future<Output = Result<AutoTickOutcome, Self::Error>> + 'a;
}

/// Outcome of Autotick Functions
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoTickOutcome {
    /// The controller reached both targets within tolerance before `timeout`.
    Completed,
    /// The controller did not settle before `timeout` elapsed.
    TimedOut,
}
