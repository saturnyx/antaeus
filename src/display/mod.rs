pub mod graphics;
use embedded_graphics::{Drawable, image::Image, pixelcolor::Rgb888, prelude::Point};
use tinybmp::Bmp;
use vexide::display::Display;

use crate::display::graphics::DisplayDriver;

pub fn print_logo(display: &mut DisplayDriver) {
    let data = include_bytes!("../../assets/logo-large.bmp");
    let bmp = Bmp::<Rgb888>::from_slice(data);
    if let Ok(img) = bmp {
        let logo = Image::new(
            &img,
            Point::new(
                Display::HORIZONTAL_RESOLUTION as i32 / 2 - 60,
                Display::VERTICAL_RESOLUTION as i32 / 2 - 60,
            ),
        );
        let _ = logo.draw(display);
    } else if let Err(e) = bmp {
        panic!("ejcuej {:?}", e);
    } else {
        panic!("help!!!")
    }
}
