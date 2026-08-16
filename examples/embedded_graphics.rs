use antaeus::graphics::embedded_graphics::DisplayDriver;
use vexide::prelude::*;

#[vexide::main]
#[allow(unused_variables)]
async fn main(peripherals: Peripherals) {
    let display = DisplayDriver::new(peripherals.display); // Embedded Graphics Display Backend
}
