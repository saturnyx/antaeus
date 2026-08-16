use antaeus::{
    peripherals::range_sensor::{KalmanRangeSensor, RangeSensor},
    prelude::{Length, Speed},
};
use vexide::{prelude::*, smart::distance::DistanceSensor};

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let sensor = DistanceSensor::new(peripherals.port_1);
    let range = RangeSensor::from_distance(sensor);
    let mut filter = KalmanRangeSensor::new(
        range,
        0.02,
        0.1,
        Length::from_metres(0.5),
        Speed::from_metres_per_second(0.0),
    );

    filter.predict();
    filter.update().await;
    let filtered = filter.measurement();
    println!("{:?}", filtered)
}
