use std::time::Duration;

use measurements::Length;
use vexide::{math::Angle, prelude::InertialSensor};
pub mod pid;

/// PID control algorithms.
///
/// Contains multiple PID implementations:
/// - [`pid`](pid::pid): Standard PID for drivetrain control.
/// - [`arcpid`](pid::arcpid): PID that allows arc movements.
/// - [`singlepid`](pid::singlepid): PID for single motor groups.
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
