//! Antaeus logo and badge display utilities.
//!
//! This module provides functions to display the Antaeus logo and badge
//! on the V5 Brain display. These are useful for branding screens, splash
//! screens, or team identification.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::display::{DisplayDriver, logo};
//!
//! let mut display = DisplayDriver::new(peripherals.display);
//! logo::print_logo(&mut display);
//! ```

use embedded_graphics::{Drawable, image::Image, pixelcolor::Rgb888, prelude::Point};
use log::warn;
use tinybmp::{Bmp, ParseError};
use vexide::display::Display;

use crate::display::DisplayDriver;

/// Displays the Antaeus logo centered on the screen.
///
/// The logo is loaded from an embedded BMP image and drawn at the center
/// of the display. If the image fails to parse, a warning is logged and
/// nothing is drawn.
///
/// # Arguments
///
/// * `display` - A mutable reference to the [`DisplayDriver`].
///
/// # Example
///
/// ```ignore
/// use antaeus::display::{DisplayDriver, logo};
///
/// let mut display = DisplayDriver::new(peripherals.display);
/// logo::print_logo(&mut display);
/// ```
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

/// Retrieves the embedded Antaeus logo as a BMP image.
///
/// Returns a parsed BMP image that can be drawn to a display target.
fn get_logo() -> Result<Bmp<'static, Rgb888>, ParseError> {
    let data = include_bytes!("../../assets/img/logo/bmp/logo-xtra-large.bmp");
    Bmp::<Rgb888>::from_slice(data)
}

/// Displays the Antaeus badge centered on the screen.
///
/// The badge is a smaller branding element than the full logo.
/// Unlike [`print_logo`], this function will panic if the image
/// fails to parse, as this indicates a build-time error.
///
/// # Arguments
///
/// * `display` - A mutable reference to the [`DisplayDriver`].
///
/// # Panics
///
/// Panics if the embedded badge BMP cannot be parsed.
///
/// # Example
///
/// ```ignore
/// use antaeus::display::{DisplayDriver, logo};
///
/// let mut display = DisplayDriver::new(peripherals.display);
/// logo::print_badge(&mut display);
/// ```
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

/// Retrieves the embedded Antaeus badge as a BMP image.
///
/// Returns a parsed BMP image that can be drawn to a display target.
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
