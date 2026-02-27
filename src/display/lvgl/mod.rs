//! LVGL (Light and Versatile Graphics Library) driver for the VEX V5 Brain display.
//!
//! This module provides LVGL integration for the V5 Brain, wrapping the hardware
//! display and touchscreen into LVGL-compatible display and input drivers.
//!
//! # Prerequisites
//!
//! Enable the `lvgl-support` feature in your `Cargo.toml`:
//!
//! ```toml
//! [dependencies]
//! antaeus = { version = "0.3", features = ["lvgl-support"] }
//! ```
//!
//! # Usage
//!
//! ```ignore
//! use vexide::prelude::*;
//! use antaeus::display::lvgl::LvglDriver;
//!
//! #[vexide::main]
//! async fn main(peripherals: Peripherals) {
//!     let mut driver = LvglDriver::new(peripherals.display)
//!         .expect("Failed to initialize LVGL");
//!
//!     // Get the active screen and add widgets
//!     let screen = driver.display().get_scr_act().unwrap();
//!     // ... add LVGL widgets to `screen` ...
//!
//!     loop {
//!         // Tick LVGL forward and process events
//!         driver.tick(core::time::Duration::from_millis(5));
//!         driver.task_handler();
//!         vexide::time::sleep(core::time::Duration::from_millis(5)).await;
//!     }
//! }
//! ```

use core::time::Duration;

use lvgl::{
    self,
    display::{Display as LvDisplay, DisplayRefresh, DrawBuffer},
    input_device::{
        BufferStatus,
        InputDriver,
        pointer::{Pointer, PointerInputData},
    },
};
use vex_sdk::vexDisplayCopyRect;
use vexide::display::{Display, TouchEvent, TouchState};

/// The horizontal resolution of the V5 Brain display.
const HOR_RES: u32 = Display::HORIZONTAL_RESOLUTION as u32;

/// The vertical resolution of the V5 Brain display.
const VER_RES: u32 = Display::VERTICAL_RESOLUTION as u32;

/// Size of the LVGL draw buffer in pixels.
/// Using 1/10th of the full framebuffer is the LVGL-recommended default.
const BUFFER_SIZE: usize = (HOR_RES as usize * VER_RES as usize) / 10;

/// Errors that can occur during LVGL driver initialization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LvglError {
    /// LVGL display registration failed.
    DisplayRegistration,
    /// LVGL input device registration failed.
    InputRegistration,
}

impl core::fmt::Display for LvglError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            LvglError::DisplayRegistration => write!(f, "Failed to register LVGL display"),
            LvglError::InputRegistration => write!(f, "Failed to register LVGL input device"),
        }
    }
}

/// An LVGL driver for the VEX V5 Brain display and touchscreen.
///
/// This struct owns the V5 `Display` peripheral and manages the LVGL display
/// and pointer input device lifecycle. It bridges LVGL's rendering pipeline
/// to the V5 hardware via `vexDisplayCopyRect` for display flushing and
/// `Display::touch_status()` for touch input.
pub struct LvglDriver {
    /// The underlying vexide Display peripheral, used for touch status.
    display:    Display,
    /// The registered LVGL display handle.
    lv_display: LvDisplay,
    /// The registered LVGL pointer input device.
    _pointer:   Pointer,
}

impl LvglDriver {
    /// Creates a new [`LvglDriver`] from a VEX V5 [`Display`] peripheral.
    ///
    /// This initializes LVGL, registers the display flush driver, and sets up
    /// the touchscreen as an LVGL pointer input device.
    ///
    /// # Errors
    ///
    /// Returns [`LvglError::DisplayRegistration`] if the LVGL display cannot be registered,
    /// or [`LvglError::InputRegistration`] if the input device fails to register.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let driver = LvglDriver::new(peripherals.display)
    ///     .expect("LVGL init failed");
    /// ```
    pub fn new(display: Display) -> Result<Self, LvglError> {
        // Initialize LVGL (safe to call multiple times).
        lvgl::init();

        // Create the draw buffer (1/10th of the screen).
        let draw_buffer = DrawBuffer::<BUFFER_SIZE>::default();

        // Register the LVGL display with a flush callback that copies
        // the rendered pixel data to the V5 display hardware.
        let lv_display = LvDisplay::register(
            draw_buffer,
            HOR_RES,
            VER_RES,
            |refresh: &DisplayRefresh<BUFFER_SIZE>| {
                // Extract the area that needs updating.
                let area = &refresh.area;
                let x1 = area.x1 as i32;
                let y1 = area.y1 as i32;
                let x2 = area.x2 as i32;
                let y2 = area.y2 as i32;

                // Convert LVGL colors to raw u32 pixel data for vexDisplayCopyRect.
                // The stride is the width of the update area.
                let width = (x2 - x1 + 1) as i32;

                // Build a buffer of raw u32 color values.
                // SAFETY: We pass the correct stride and coordinates to
                // vexDisplayCopyRect, which copies pixels to the display
                // framebuffer.
                let colors = &refresh.colors;
                let color_count = (width as usize) * ((y2 - y1 + 1) as usize);
                let mut pixel_buf: alloc::vec::Vec<u32> =
                    alloc::vec::Vec::with_capacity(color_count);

                for color in colors.iter().take(color_count) {
                    // Convert LVGL Color to raw 0x00RRGGBB u32.
                    let r = color.r() as u32;
                    let g = color.g() as u32;
                    let b = color.b() as u32;
                    pixel_buf.push((r << 16) | (g << 8) | b);
                }

                unsafe {
                    vexDisplayCopyRect(x1, y1, x2, y2, pixel_buf.as_mut_ptr(), width);
                }
            },
        )
        .map_err(|_| LvglError::DisplayRegistration)?;

        // Register pointer (touch) input device.
        // We use a static to communicate touch state to LVGL's callback,
        // since Pointer::register requires an Fn() closure.
        let pointer = Pointer::register(
            || -> BufferStatus {
                // Read the current touch state from the V5 hardware.
                // SAFETY: We read touch data through the VEX SDK.
                let touch = unsafe {
                    let mut status = vex_sdk::V5_TouchStatus::default();
                    vex_sdk::vexTouchDataGet(&raw mut status);
                    status
                };

                let point = lvgl::Point::new(touch.lastXpos as i32, touch.lastYpos as i32);

                let input_data = PointerInputData::Touch(point);

                match TouchState::from(touch.lastEvent) {
                    TouchState::Pressed | TouchState::Held => input_data.pressed().once(),
                    TouchState::Released => input_data.released().once(),
                }
            },
            &lv_display,
        )
        .map_err(|_| LvglError::InputRegistration)?;

        Ok(Self {
            display,
            lv_display,
            _pointer: pointer,
        })
    }

    /// Returns a reference to the LVGL [`Display`](LvDisplay), which can be
    /// used to get the active screen and manage LVGL objects.
    #[must_use]
    pub fn display(&self) -> &LvDisplay { &self.lv_display }

    /// Returns the current touch status from the underlying V5 display.
    #[must_use]
    pub fn touch_status(&self) -> TouchEvent { self.display.touch_status() }

    /// Advances the LVGL tick counter.
    ///
    /// This must be called periodically (e.g., every 5ms) to keep LVGL's
    /// internal timing accurate for animations, input debouncing, etc.
    pub fn tick(duration: Duration) {
        unsafe {
            lvgl_sys::lv_tick_inc(duration.as_millis() as u32);
        }
    }

    /// Runs the LVGL task handler.
    ///
    /// This processes pending display refreshes, input events, animations,
    /// and other LVGL tasks. Must be called periodically in your main loop.
    pub fn task_handler() {
        unsafe {
            lvgl_sys::lv_timer_handler();
        }
    }
}
