use antaeus::{
    motion::{localization::Localizer, pursuit::IsLocalizerError},
    utils::geo::Pose,
};
use snafu::Snafu;

pub struct DummyOdom;

#[derive(Debug, Snafu)]
pub struct SomeError;

impl IsLocalizerError for SomeError {}

impl Localizer<SomeError> for DummyOdom {
    fn get_coords(&self) -> Pose { Pose::origin() }

    async fn tick(&mut self) -> Result<(), SomeError> { Ok(()) }
}
