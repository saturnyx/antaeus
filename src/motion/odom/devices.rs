use std::sync::Arc;

use log::warn;
use vexide::{
    adi::encoder::AdiOpticalEncoder,
    math::Angle,
    smart::{PortError, imu::InertialSensor, rotation::RotationSensor},
    sync::Mutex,
};

use crate::peripherals::drivetrain::Differential;

#[derive(Clone)]
pub enum TrackingSensor {
    AdiOpticalEncoder(Arc<Mutex<AdiOpticalEncoder>>),
    RotationSensor(Arc<Mutex<RotationSensor>>),
    Differential(Differential),
    None,
}

impl TrackingSensor {
    pub fn new_adi_optical_encoder(encoder: AdiOpticalEncoder) -> Self {
        Self::AdiOpticalEncoder(Arc::new(Mutex::new(encoder)))
    }

    pub fn new_rotation_sensor(sensor: RotationSensor) -> Self {
        Self::RotationSensor(Arc::new(Mutex::new(sensor)))
    }

    pub fn new_differential(diff: Differential) -> Self { Self::Differential(diff) }

    pub fn new_none() -> Self { Self::None }

    pub async fn position(&self) -> Angle {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => {
                encoder.lock().await.position().unwrap_or_else(|e| {
                    warn!("ADI Optical Encoder Position Error: {}", e);
                    Angle::from_radians(0.0)
                })
            }
            TrackingSensor::RotationSensor(encoder) => {
                encoder.lock().await.position().unwrap_or_else(|e| {
                    warn!("Rotation Sensor Position Error: {}", e);
                    Angle::from_radians(0.0)
                })
            }
            TrackingSensor::Differential(dt) => dt.position(),
            TrackingSensor::None => Angle::from_radians(0.0),
        }
    }

    pub async fn reset_position(&self) -> Result<(), PortError> {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => encoder.lock().await.reset_position(),
            TrackingSensor::RotationSensor(encoder) => encoder.lock().await.reset_position(),
            TrackingSensor::Differential(dt) => dt.reset_position(),
            TrackingSensor::None => Ok(()),
        }
    }

    pub async fn set_position(&self, position: Angle) -> Result<(), PortError> {
        match self {
            TrackingSensor::AdiOpticalEncoder(encoder) => {
                encoder.lock().await.set_position(position)
            }
            TrackingSensor::RotationSensor(encoder) => encoder.lock().await.set_position(position),
            TrackingSensor::Differential(dt) => dt.set_position(position),
            TrackingSensor::None => Ok(()),
        }
    }
}

#[derive(Clone)]
pub struct Tracker {
    pub sensor:         TrackingSensor,
    pub wheel_diameter: f64,
    pub driven_gear:    f64,
    pub driving_gear:   f64,
    pub offset:         f64,
}

impl Tracker {
    pub fn new(
        sensor: TrackingSensor,
        wheel_diameter: f64,
        driven_gear: f64,
        driving_gear: f64,
        offset: f64,
    ) -> Self {
        Self {
            sensor,
            wheel_diameter,
            driven_gear,
            driving_gear,
            offset,
        }
    }

    pub async fn dist(&self) -> f64 {
        let angle = self.sensor.position().await;
        let gear_ratio = self.driving_gear as f64 / self.driven_gear as f64;
        let distance = angle.as_radians() * gear_ratio * (self.wheel_diameter / 2.0);
        distance + self.offset
    }
}

#[derive(Clone)]
pub struct TrackerMech {
    pub vertical_tracker:   Tracker,
    pub horizontal_tracker: Tracker,
    pub imu:                Arc<Mutex<InertialSensor>>,
}

impl TrackerMech {
    pub fn new(
        vertical_tracker: Tracker,
        horizontal_tracker: Tracker,
        imu: Arc<Mutex<InertialSensor>>,
    ) -> Self {
        Self {
            vertical_tracker,
            horizontal_tracker,
            imu,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub x: f64,
    pub y: f64,
    pub t: Angle,
}

impl Pose {
    pub fn new(x: f64, y: f64, t: Angle) -> Self { Self { x, y, t } }

    pub fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            t: Angle::from_radians(0.0),
        }
    }
}
