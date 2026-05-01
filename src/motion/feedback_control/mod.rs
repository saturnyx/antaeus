use std::time::Duration;

use vexide::{math::Angle, prelude::InertialSensor};

use crate::misc::units::Length;
pub mod pid;

/// PID control algorithms.
///
/// Contains multiple legacy PID implementations:
/// - [`linear_pid`](legacy_pid::linear_pid): Standard PID for drivetrain control.
/// - [`arcpid`](legacy_pid::arcpid): PID that allows arc movements.
/// - [`singlepid`](legacy_pid::singlepid): PID for single motor groups.
pub mod legacy_pid;

pub trait DriveControl {
    fn travel(&mut self, target: Length, timeout: Duration) -> impl Future<Output = ()> + '_;
    fn rotate(&mut self, angle: Angle, timeout: Duration) -> impl Future<Output = ()> + '_;
    fn pivot(&mut self, angle: Angle, timeout: Duration) -> impl Future<Output = ()> + '_;

    fn imu_rotate<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &'a InertialSensor,
        angle_tolerance: Angle,
    ) -> impl Future<Output = ()> + 'a;

    fn imu_pivot<'a>(
        &'a mut self,
        angle: Angle,
        timeout: Duration,
        imu: &'a InertialSensor,
        angle_tolerance: Angle,
    ) -> impl Future<Output = ()> + 'a;
}
