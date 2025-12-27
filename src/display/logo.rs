//! Display the Antaeus Logo on the V5 Brain

use embedded_graphics::{Drawable, image::Image, pixelcolor::Rgb888, prelude::Point};
use log::warn;
use tinybmp::{Bmp, ParseError};
use vexide::display::Display;

use crate::display::DisplayDriver;

pub fn print_logo(display: &mut DisplayDriver) {
    let bmp = get_logo();
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
        warn!("Error Parsing Bitmap: {:?}", e);
    } else {
        warn!("Error Parsing Bitmap")
    }
}

fn get_logo() -> Result<Bmp<'static, Rgb888>, ParseError> {
    let data = include_bytes!("../../assets/img/logo/bmp/logo-xtra-large.bmp");
    Bmp::<Rgb888>::from_slice(data)
}

pub fn print_badge(display: &mut DisplayDriver) {
    let bmp = get_badge();
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
        panic!("Error Parsing Bitmap {:?}", e);
    } else {
        panic!("Unknown Error")
    }
}

fn get_badge() -> Result<Bmp<'static, Rgb888>, ParseError> {
    let data = include_bytes!("../../assets/img/badge/bmp/badge-xtra-large.bmp");
    Bmp::<Rgb888>::from_slice(data)
}

#[cfg(test)]
mod tests {
    use embedded_graphics::{Drawable, image::Image, pixelcolor::Rgb888, prelude::*};
    use embedded_graphics_simulator::{OutputSettingsBuilder, SimulatorDisplay, Window};
    use vexide::prelude::Display;

    use crate::display::logo::get_logo;

    #[test]
    #[ignore = "manual verification needed (popup display)"]
    fn logo_test() -> Result<(), core::convert::Infallible> {
        // Create a simulator display
        let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(
            Display::HORIZONTAL_RESOLUTION as u32,
            Display::VERTICAL_RESOLUTION as u32,
        ));

        let bmp = get_logo();

        if let Ok(img) = bmp {
            let logo = Image::new(
                &img,
                Point::new(
                    Display::HORIZONTAL_RESOLUTION as i32 / 2 - 100,
                    Display::VERTICAL_RESOLUTION as i32 / 2 - 100,
                ),
            );
            logo.draw(&mut display)?;
        } else {
            panic!("Failed to load logo BMP");
        }

        // Show the result
        let output_settings = OutputSettingsBuilder::new().build();
        Window::new("Logo Test", &output_settings).show_static(&display);

        Ok(())
    }

    #[test]
    #[ignore = "manual verification needed (popup display)"]
    fn badge_test() -> Result<(), core::convert::Infallible> {
        // Create a simulator display
        let mut display: SimulatorDisplay<Rgb888> = SimulatorDisplay::new(Size::new(
            Display::HORIZONTAL_RESOLUTION as u32,
            Display::VERTICAL_RESOLUTION as u32,
        ));

        let bmp = crate::display::logo::get_badge();

        if let Ok(img) = bmp {
            let badge = Image::new(
                &img,
                Point::new(
                    Display::HORIZONTAL_RESOLUTION as i32 / 2 - 100,
                    Display::VERTICAL_RESOLUTION as i32 / 2 - 100,
                ),
            );
            badge.draw(&mut display)?;
        } else {
            panic!("Failed to load badge BMP");
        }

        // Show the result
        let output_settings = OutputSettingsBuilder::new().build();
        Window::new("badge Test", &output_settings).show_static(&display);

        Ok(())
    }
}
