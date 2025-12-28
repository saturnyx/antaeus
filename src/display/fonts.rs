//! Pre-loaded fonts for use with embedded-graphics on the V5 Brain display.
//!
//! This module provides a collection of TrueType fonts that are embedded in the
//! binary and can be used for text rendering without requiring external files.
//!
//! # Available Fonts
//!
//! - **Collegiate**: Two variants (solid and outline) used by Team 8059 Blank.
//! - **Jacquard12**: A decorative pixel-style font.
//! - **Silkscreen**: A clean pixel font with regular and bold variants.
//! - **Micro5**: A compact 5-pixel font for small displays.
//!
//! # Example
//!
//! ```ignore
//! use antaeus::display::fonts::AntaeusFonts;
//! use embedded_graphics::{pixelcolor::Rgb565, prelude::*, text::Text};
//!
//! let style = AntaeusFonts::silkscreen(24, Rgb565::WHITE);
//! let text = Text::new("Hello, VEX!", Point::new(10, 50), style);
//! text.draw(&mut display)?;
//! ```

use embedded_graphics::pixelcolor::Rgb565;
use embedded_ttf::*;
use rusttype::Font;

/// A collection of pre-loaded fonts for embedded-graphics text rendering.
///
/// All fonts are embedded directly in the binary, so no external font files
/// are required. Each method returns a [`FontTextStyle`] configured with the
/// specified size and color.
pub struct AntaeusFonts;

impl AntaeusFonts {
    /// Returns the Collegiate Outline FLF font style.
    ///
    /// This is a solid collegiate-style font. Used by VEX Team 8059 Blank.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn blank_solid(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/CollegiateOutlineFLF.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Returns the Collegiate Black FLF font style.
    ///
    /// This is an outline/hollow version of the collegiate font.
    /// Used by VEX Team 8059 Blank.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn blank_outline(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/CollegiateBlackFLF.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Returns the Jacquard12 Regular font style.
    ///
    /// A decorative pixel-style font suitable for headers and titles.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn jacquard12(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Jacquard12-Regular.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Returns the Silkscreen Regular font style.
    ///
    /// A clean, readable pixel font ideal for general text display.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn silkscreen(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Silkscreen-Regular.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Returns the Silkscreen Bold font style.
    ///
    /// A bold variant of the Silkscreen pixel font for emphasis.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn silkscreen_bold(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Silkscreen-Bold.ttf")).unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Returns the Micro5 Regular font style.
    ///
    /// A very compact 5-pixel font for displaying a lot of information
    /// in small spaces. Good for debugging overlays and data displays.
    ///
    /// # Arguments
    ///
    /// * `size` - Font size in pixels.
    /// * `color` - Text color in RGB565 format.
    pub fn micro_5(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Micro5-Regular.ttf")).unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }
}

#[cfg(test)]
mod tests {
    use embedded_graphics::{Drawable, pixelcolor::Rgb565, prelude::*, text::Text};
    use embedded_graphics_simulator::{OutputSettingsBuilder, SimulatorDisplay, Window};
    use vexide::display::Display;

    use super::AntaeusFonts;

    #[test]
    #[ignore = "manual verification needed (pop up display)"]
    fn font_test() {
        let mut display: SimulatorDisplay<Rgb565> = SimulatorDisplay::new(Size::new(
            Display::HORIZONTAL_RESOLUTION as u32,
            Display::VERTICAL_RESOLUTION as u32,
        ));

        let style = AntaeusFonts::micro_5(50, Rgb565::WHITE);
        let txt = "QWERTYUIOPAS\nDFGHJKLZXCVB\nNM1234567890";
        let _ = Text::new(txt, Point::new(15, 30), style).draw(&mut display);
        let output_settings = OutputSettingsBuilder::new().build();
        Window::new("Fonts", &output_settings).show_static(&display);
    }
}
