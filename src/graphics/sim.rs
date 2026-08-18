//! Simulator Driver that mirrors the vexide-embedded-graphics api
use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*};
use embedded_graphics_simulator::{OutputSettings, SimulatorDisplay, SimulatorEvent, Window};
use vexide::display::Display;

/// Simulator Display Driver
pub struct DisplayDriver {
    sim:    SimulatorDisplay<Rgb888>,
    window: Window,
}

impl DisplayDriver {
    /// Create a new Simulator Display
    /// (The display argument is actually used)
    #[must_use]
    pub fn new(_display: Display) -> Self {
        let sim = SimulatorDisplay::new(Size::new(
            Display::HORIZONTAL_RESOLUTION as u32,
            Display::VERTICAL_RESOLUTION as u32,
        ));
        let mut window = Window::new(
            "V5 Sim",
            &OutputSettings {
                scale: 2,
                ..Default::default()
            },
        );
        window.set_max_fps(60);
        window.update(&sim);
        Self { sim, window }
    }

    /// Pumps SDL2 events and exits cleanly if the window was closed.
    fn pump(&mut self) {
        self.window.update(&self.sim);
        if self.window.events().any(|e| e == SimulatorEvent::Quit) {
            std::process::exit(0);
        }
    }
}

impl OriginDimensions for DisplayDriver {
    fn size(&self) -> Size { self.sim.size() }
}

impl DrawTarget for DisplayDriver {
    type Color = Rgb888;
    type Error = core::convert::Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where I: IntoIterator<Item = Pixel<Self::Color>> {
        self.sim.draw_iter(pixels)?;
        self.pump();
        Ok(())
    }

    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics_core::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        self.sim.fill_contiguous(area, colors)?;
        self.pump();
        Ok(())
    }

    fn fill_solid(
        &mut self,
        area: &embedded_graphics_core::primitives::Rectangle,
        color: Self::Color,
    ) -> Result<(), Self::Error> {
        self.sim.fill_solid(area, color)?;
        self.pump();
        Ok(())
    }
}
