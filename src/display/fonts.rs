use embedded_graphics::pixelcolor::Rgb565;
use embedded_ttf::*;
use rusttype::Font;

/// A List of Fonts that can be used with Embedded Graphics
pub struct AntaeusFonts;

impl AntaeusFonts {
    /// CollegiateOutlineFLF
    ///
    /// This is the font used by 8059 Blank.
    pub fn blank_solid(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/CollegiateOutlineFLF.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// CollegiateBlackFLF
    ///
    /// This is the font used by 8059 Blank.
    pub fn blank_outline(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/CollegiateBlackFLF.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Jacquard12-Regular
    pub fn jacquard12(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Jacquard12-Regular.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Silkscreen-Regular
    pub fn silkscreen(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Silkscreen-Regular.ttf"))
                .unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Silkscreen-Bold
    pub fn silkscreen_bold(size: u32, color: Rgb565) -> FontTextStyle<Rgb565> {
        FontTextStyleBuilder::new(
            Font::try_from_bytes(include_bytes!("../../assets/fonts/Silkscreen-Bold.ttf")).unwrap(),
        )
        .font_size(size)
        .text_color(color)
        .build()
    }

    /// Micro5
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
