use antaeus::{motion::localization::Localizer, utils::geo::Pose};
use snafu::Snafu;

pub struct DummyOdom;

#[derive(Debug, Snafu)]
pub struct SomeError;

impl Localizer for DummyOdom {
    type Error = SomeError;

    fn get_coords(&self) -> Pose { Pose::origin() }

    fn tick(&mut self) -> Result<(), SomeError> { Ok(()) }
}
