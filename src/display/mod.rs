//! [`embedded-graphics`] Driver for the VEX V5
//!
//! [`embedded-graphics`]: https://crates.io/crates/embedded-graphics
//!
//! This module provides a `DrawTarget` implementation for the VEX V5 brain display,
//! allowing you to draw to the display using the `embedded-graphics` ecosystem.
//!
//! # Usage
//!
//! To begin, turn your `display` peripheral into a `DisplayDriver`:
//!
//! ```ignore
//! use vexide::prelude::*;
//! use antaeus::display::DisplayDriver;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let mut display = DisplayDriver::new(peripherals.display);
//! }
//! ```
//!
//! `DisplayDriver` is a `DrawTarget` that the `embedded-graphics` crate is
//! able to draw to.
//!
//! ```ignore
//! #![no_std]
//! #![no_main]
//!
//! use embedded_graphics::{
//!     mono_font::{MonoTextStyle, ascii::FONT_6X10},
//!     pixelcolor::Rgb888,
//!     prelude::*,
//!     text::Text,
//! };
//! use vexide::prelude::*;
//! use antaeus::display::DisplayDriver;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let mut display = DisplayDriver::new(peripherals.display);
//!     let style = MonoTextStyle::new(&FONT_6X10, Rgb888::GREEN);
//!
//!     Text::new("Hello,\nRust!", Point::new(2, 28), style)
//!         .draw(&mut display)
//!         .unwrap();
//! }
//! ```
//!
//! Check out the [`embedded-graphics` docs] for more examples.
//!
//! [`embedded-graphics` docs]: https://docs.rs/embedded-graphics/latest/embedded_graphics/examples/index.html

/// Pre-loaded TrueType fonts for text rendering.
///
/// See the `fonts` module documentation for available fonts and usage examples.
pub mod fonts;

/// Antaeus logo and badge display utilities.
///
/// Functions for displaying the Antaeus branding on the V5 Brain screen.
pub mod logo;

use core::convert::Infallible;

use embedded_graphics_core::{pixelcolor::Rgb888, prelude::*};
use vex_sdk::{vexDisplayCopyRect, vexDisplayForegroundColor, vexDisplayRectFill};
use vexide::display::{Display, RenderMode, TouchEvent};

/// An embedded-graphics draw target for the V5 Brain display
/// Currently, this does not support touch detection like the regular [`Display`] API.
pub struct DisplayDriver {
    display: Display,
    buffer:  [u32; Display::HORIZONTAL_RESOLUTION as usize * Display::VERTICAL_RESOLUTION as usize],
}

impl DisplayDriver {
    /// Create a new [`DisplayDriver`] from a [`Display`].
    ///
    /// The display peripheral must be moved into this struct,
    /// as it is used to render the display and having multiple
    /// mutable references to it is unsafe.
    #[must_use]
    pub fn new(display: Display) -> Self {
        Self {
            display,
            #[allow(clippy::large_stack_arrays)] // we got plenty
            buffer: [0; Display::HORIZONTAL_RESOLUTION as usize
                * Display::VERTICAL_RESOLUTION as usize],
        }
    }

    /// Returns the current touch status of the display.
    #[must_use]
    pub fn touch_status(&self) -> TouchEvent { self.display.touch_status() }

    /// Sets the rendering mode of the display
    pub fn set_render_mode(&mut self, mode: RenderMode) { self.display.set_render_mode(mode); }

    /// Returns the current rendering mode of the display
    #[must_use]
    pub fn render_mode(&self) -> RenderMode { self.display.render_mode() }

    /// Renders the display if the rendering mode is set to [`RenderMode::DoubleBuffered`].
    pub fn render(&mut self) { self.display.render(); }
}

impl OriginDimensions for DisplayDriver {
    fn size(&self) -> Size {
        Size {
            width:  Display::HORIZONTAL_RESOLUTION as _,
            height: Display::VERTICAL_RESOLUTION as _,
        }
    }
}

impl DrawTarget for DisplayDriver {
    type Color = Rgb888;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where I: IntoIterator<Item = Pixel<Self::Color>> {
        pixels.into_iter().for_each(|Pixel(pos, color)| {
            if pos.x >= 0 &&
                pos.x < Display::HORIZONTAL_RESOLUTION as i32 &&
                pos.y >= 0 &&
                pos.y < Display::VERTICAL_RESOLUTION as i32
            {
                unsafe {
                    vex_sdk::vexDisplayForegroundColor(color.into_storage());
                    vex_sdk::vexDisplayPixelSet(
                        pos.x as u32,
                        Display::HEADER_HEIGHT as u32 + pos.y as u32,
                    );
                }
            }
        });

        Ok(())
    }

    // Note: clear is not implemented because vexDisplayErase does not allow
    // changing the background color.

    fn fill_contiguous<I>(
        &mut self,
        area: &embedded_graphics_core::primitives::Rectangle,
        colors: I,
    ) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        if let Some(bottom_right) = area.bottom_right() {
            // Copy the colors into the buffer
            colors.into_iter().enumerate().for_each(|(i, color)| {
                self.buffer[i] = color.into_storage();
            });
            // Copy the buffer to the display
            unsafe {
                vexDisplayCopyRect(
                    area.top_left.x,
                    Display::HEADER_HEIGHT as i32 + area.top_left.y,
                    bottom_right.x,
                    Display::HEADER_HEIGHT as i32 + bottom_right.y,
                    self.buffer.as_mut_ptr(),
                    area.size.width as i32,
                );
            }
        }

        Ok(())
    }

    fn fill_solid(
        &mut self,
        area: &embedded_graphics_core::primitives::Rectangle,
        color: Self::Color,
    ) -> Result<(), Self::Error> {
        if let Some(bottom_right) = area.bottom_right() {
            unsafe {
                vexDisplayForegroundColor(color.into_storage());
                vexDisplayRectFill(
                    area.top_left.x,
                    Display::HEADER_HEIGHT as i32 + area.top_left.y,
                    bottom_right.x,
                    Display::HEADER_HEIGHT as i32 + bottom_right.y,
                );
            }
        }

        Ok(())
    }
}
