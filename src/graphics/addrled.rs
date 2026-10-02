//! ADI Addressable LEDs (Simulator)
//!
//! This module will automatically map to Vexide's WS2812B Driver if compiled for VexOS.
//!
//! # Hardware Overview
//!
//! ADI ports are capable of controlling a WS2812B LED strip with up to 64 diodes per set of 8 ADI
//! ports. This limitation is due to the 2A current limit on ADI ports — plugging multiple strips
//! into the same set of ADI ports may cause your lights to flicker due to this limit being reached.
//! If you require more than 64 continuously running diodes, then you can run each strip through its
//! own [ADI Expander](crate::smart::expander::AdiExpander).
//!
//! The V5's ADI ports can present some technical challenges when interfacing with LEDs. Some
//! commercially available strips will not work with the V5 out of the box, but mileage may vary.
//! This is mainly caused by two "quirks" of the V5's ADI ports:
//!
//! - ADI ports operate at 3.3V digital logic, but most WS2812B strips expect 5V logic.
//! - The Brain's ADI ports include built-in short protection via a 1kΩ resistor that may impact
//!   signal timing on some strips, slowing down the edges of digital logic pulses sent to strip. In
//!   rare cases, this can cause issues with some strips.
//!
//! Using something like a [74HCT125 buffer] inline with the output to convert the 3.3-5V logic
//! addresses both these problems.
//!
//! # `smart-leds-trait` Integration
//!
//! vexide implements the [`SmartLedsWrite`] trait from the [`smart-leds-rs`](https://github.com/smart-leds-rs)
//! ecosystem on [`AdiAddrLed`]. This is useful if you need more advanced features for controlling
//! the strip, such as gradients or gamma correction.
//!
//! [WS2812B]: https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf
//! [74HCT125 buffer]: https://www.diodes.com/assets/Datasheets/74HCT125.pdf
//! [`smart-leds-trait`]: https://docs.rs/smart-leds-trait/0.3.0/smart_leds_trait/index.html
//! [`SmartLedsWrite`]: https://docs.rs/smart-leds-trait/0.3.0/smart_leds_trait/trait.SmartLedsWrite.html
use std::{
    collections::BTreeMap,
    fmt::Write as _,
    iter,
    path::{Path, PathBuf},
    sync::{Mutex, MutexGuard, Once},
    thread,
    time::Duration,
};

use sdl2::{
    event::Event,
    keyboard::Keycode,
    pixels::{Color as SdlColor, PixelFormatEnum},
    rect::{Point, Rect},
    render::{BlendMode, Canvas},
    surface::Surface,
    video::Window,
};
use smart_leds_trait::SmartLedsWrite;
use vexide::{
    adi::{AdiDevice, AdiDeviceType, AdiPort},
    color::Color,
    smart::PortError,
};

/// Converts a vexide [`Color`] to the `0x00RRGGBB` value drawn by the simulator.
///
/// If `Color::into_raw` is not public in your vexide version, swap in `u32::from(color)` or
/// build the value from `color.r()`, `color.g()` and `color.b()`.
fn raw(color: Color) -> u32 { color.into_raw() & 0x00FF_FFFF }

/// Exit the whole process when the window is closed (like closing any simulator).
/// If `false`, the window just goes away and the program keeps running.
const EXIT_ON_CLOSE: bool = true;

/// WS2812B Addressable LED Strip (simulated).
#[derive(Debug, Eq, PartialEq)]
pub struct AdiAddrLed<const N: usize> {
    port: AdiPort,
}

impl<const N: usize> AdiAddrLed<N> {
    /// The max number of LED diodes on one strip that a single ADI port can control.
    pub const MAX_LENGTH: usize = MAX_PX;

    /// Initializes an LED strip with a given length on an ADI port.
    #[must_use]
    pub fn new(port: AdiPort) -> Self {
        const {
            assert!(N <= MAX_PX, "AdiAddrLed strip size exceeded MAX_LENGTH (64)");
        }

        let this = Self { port };
        register(this.key(), N);
        this
    }

    fn key(&self) -> StripKey {
        StripKey {
            expander: self.port.expander_number(),
            port:     self.port.number(),
        }
    }

    /// Set the entire LED strip to one color.
    ///
    /// # Errors
    ///
    /// These errors are only returned if the device is plugged into an
    /// [`AdiExpander`](crate::smart::expander::AdiExpander).
    ///
    /// - A [`PortError::Disconnected`] error is returned if no expander was connected to the port.
    /// - A [`PortError::IncorrectDevice`] error is returned if a device other than an expander was
    ///   connected to the port.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use antaeus::graphics::led::sim::AdiAddrLed;
    /// use vexide::{color::Color, prelude::*};
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // Create a new LED strip with 8 addressable pixels.
    ///     let mut leds = AdiAddrLed::<8>::new(peripherals.adi_a);
    ///
    ///     // Set all pixels to white.
    ///     _ = leds.set_all(Color::WHITE);
    /// }
    /// ```
    pub fn set_all(&mut self, color: impl Into<Color>) -> Result<(), PortError> {
        push_pixels(self.key(), N, 0, std::iter::repeat_n(raw(color.into()), N));
        Ok(())
    }

    /// Sets the color of an individual diode on the strip.
    ///
    /// # Panics
    ///
    /// Panics if the index is out of range for this strip (`index < N`).
    ///
    /// # Errors
    ///
    /// These errors are only returned if the device is plugged into an
    /// [`AdiExpander`](crate::smart::expander::AdiExpander).
    ///
    /// - A [`PortError::Disconnected`] error is returned if no expander was connected to the port.
    /// - A [`PortError::IncorrectDevice`] error is returned if a device other than an expander was
    ///   connected to the port.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use antaeus::graphics::led::sim::AdiAddrLed;
    /// use vexide::{color::Color, prelude::*};
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // Create a new LED strip with 8 addressable pixels.
    ///     let mut leds = AdiAddrLed::<8>::new(peripherals.adi_a);
    ///
    ///     // Set the first pixel in the strip to white.
    ///     _ = leds.set_pixel(0, Color::WHITE);
    /// }
    /// ```
    pub fn set_pixel(&mut self, index: usize, color: impl Into<Color>) -> Result<(), PortError> {
        assert!(index < N, "pixel index was out of range for LED strip size");

        push_pixels(self.key(), N, index, iter::once(raw(color.into())));
        Ok(())
    }

    /// Attempt to write an iterator of colors to the LED strip. Returns how many colors were
    /// actually written.
    ///
    /// # Errors
    ///
    /// These errors are only returned if the device is plugged into an
    /// [`AdiExpander`](crate::smart::expander::AdiExpander).
    ///
    /// - A [`PortError::Disconnected`] error is returned if no expander was connected to the port.
    /// - A [`PortError::IncorrectDevice`] error is returned if a device other than an expander was
    ///   connected to the port.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use antaeus::graphics::led::sim::AdiAddrLed;
    /// use vexide::{color::Color, prelude::*};
    ///
    /// #[vexide::main]
    /// async fn main(peripherals: Peripherals) {
    ///     // Create a new LED strip with 8 addressable pixels.
    ///     let mut leds = AdiAddrLed::<8>::new(peripherals.adi_a);
    ///
    ///     // List of colors that each LED pixel will be set to.
    ///     let colors = [
    ///         Color::RED,
    ///         Color::YELLOW,
    ///         Color::GREEN,
    ///         Color::BLUE,
    ///         Color::PURPLE,
    ///         Color::RED,
    ///         Color::YELLOW,
    ///         Color::GREEN,
    ///     ];
    ///
    ///     // Set the first pixel in the strip to white.
    ///     _ = leds.set_buffer(&colors);
    /// }
    /// ```
    pub fn set_buffer(&mut self, buf: &[Color]) -> Result<usize, PortError> {
        push_pixels(self.key(), N, 0, buf.iter().take(N).map(|&c| raw(c)));
        Ok(buf.len().min(N))
    }
}

impl<const N: usize> AdiDevice<1> for AdiAddrLed<N> {
    fn port_numbers(&self) -> [u8; 1] { [self.port.number()] }

    fn expander_port_number(&self) -> Option<u8> { self.port.expander_number() }

    fn device_type(&self) -> AdiDeviceType { AdiDeviceType::DigitalOut }
}

impl<const N: usize> SmartLedsWrite for AdiAddrLed<N> {
    type Color = Color;
    type Error = PortError;

    fn write<T, I>(&mut self, iterator: T) -> Result<(), Self::Error>
    where
        T: IntoIterator<Item = I>,
        I: Into<Self::Color>, {
        let pixels = iterator
            .into_iter()
            .map(|c| raw(c.into()))
            .chain(iter::repeat(0))
            .take(N);
        push_pixels(self.key(), N, 0, pixels);
        Ok(())
    }
}

const MAX_PX: usize = 64;
/// Horizontal distance between diode centres, in window pixels.
const PITCH: i32 = 16;
/// Side length of the (square, 5050-style) diode package.
const DIE: i32 = 10;
const MARGIN: i32 = 16;
const ROW_H: i32 = 36;
const BANNER_H: i32 = 4;
/// Bloom: concentric additive discs around each lit diode.
const GLOW_RINGS: i32 = 4;
const GLOW_STEP: i32 = 3;
const GLOW_ALPHA: u8 = 16;
const WINDOW_W: u32 = (MAX_PX as i32 * PITCH + 2 * MARGIN) as u32;
const FRAME_TIME: Duration = Duration::from_millis(16);

/// Identifies one physical strip: which ADI port, on which expander (if any).
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
struct StripKey {
    expander: Option<u8>,
    port:     u8,
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct Strip {
    len: usize,
    /// `0x00RRGGBB`
    px:  [u32; MAX_PX],
}

impl Strip {
    fn new(len: usize) -> Self {
        Self {
            len: len.min(MAX_PX),
            px:  [0; MAX_PX],
        }
    }
}

struct State {
    strips: BTreeMap<StripKey, Strip>,
}

static STATE: Mutex<State> = Mutex::new(State {
    strips: BTreeMap::new(),
});
static START: Once = Once::new();

fn state() -> MutexGuard<'static, State> { STATE.lock().unwrap_or_else(|e| e.into_inner()) }

fn start() {
    START.call_once(|| {
        let spawned = thread::Builder::new().name("addrled-sim".into()).spawn(|| {
            if let Err(e) = render_loop() {
                eprintln!("[addrled-sim] SDL2 backend unavailable: {e}");
            }
        });
        if let Err(e) = spawned {
            eprintln!("[addrled-sim] could not spawn render thread: {e}");
        }
    });
}

fn register(key: StripKey, len: usize) {
    start();
    state()
        .strips
        .entry(key)
        .and_modify(|s| s.len = len.min(MAX_PX))
        .or_insert_with(|| Strip::new(len));
}

/// Equivalent of the hardware write: put `pixels` into the strip starting at `offset`, leaving all
/// other pixels untouched. Anything past the end of the strip is dropped.
fn push_pixels(key: StripKey, len: usize, offset: usize, pixels: impl Iterator<Item = u32>) {
    start();
    let mut st = state();
    let strip = st.strips.entry(key).or_insert_with(|| Strip::new(len));
    for (i, px) in pixels.enumerate() {
        let idx = offset.saturating_add(i);
        if idx >= strip.len {
            break;
        }
        strip.px[idx] = px & 0x00FF_FFFF;
    }
}

fn render_loop() -> Result<(), String> {
    let sdl = sdl2::init()?;
    let video = sdl.video()?;
    let window = video
        .window("Antaeus AdiAddrLed simulator", WINDOW_W, window_h(1))
        .position_centered()
        .build()
        .map_err(|e| e.to_string())?;
    let mut canvas = window
        .into_canvas()
        .software()
        .build()
        .map_err(|e| e.to_string())?;
    let mut pump = sdl.event_pump()?;

    let mut dump_path: Option<PathBuf> = std::env::var_os("ADDRLED_SIM_DUMP").map(PathBuf::from);
    let mut last_dumped: Option<Vec<(StripKey, Strip)>> = None;
    let mut layout: Vec<(StripKey, usize)> = Vec::new();

    loop {
        for event in pump.poll_iter() {
            match event {
                Event::Quit { .. } |
                Event::KeyDown {
                    keycode: Some(Keycode::Escape),
                    ..
                } => {
                    if EXIT_ON_CLOSE {
                        std::process::exit(0);
                    }
                    return Ok(());
                }
                _ => {}
            }
        }

        let snapshot: Vec<(StripKey, Strip)> =
            state().strips.iter().map(|(k, s)| (*k, *s)).collect();

        sync_window(&mut canvas, &snapshot, &mut layout)?;
        draw(&mut canvas, &snapshot)?;

        if let Some(path) = &dump_path &&
            last_dumped.as_ref() != Some(&snapshot)
        {
            if let Err(e) = dump_frame(&canvas, path) {
                eprintln!("[addrled-sim] frame dump failed, disabling: {e}");
                dump_path = None;
            }
            last_dumped = Some(snapshot);
        }

        canvas.present();
        thread::sleep(FRAME_TIME);
    }
}

fn window_h(rows: usize) -> u32 { (BANNER_H + 2 * MARGIN + rows.max(1) as i32 * ROW_H) as u32 }

/// Resize the window and refresh the title whenever the set of strips changes.
fn sync_window(
    canvas: &mut Canvas<Window>,
    strips: &[(StripKey, Strip)],
    layout: &mut Vec<(StripKey, usize)>,
) -> Result<(), String> {
    let current: Vec<(StripKey, usize)> = strips.iter().map(|(k, s)| (*k, s.len)).collect();
    if *layout == current {
        return Ok(());
    }
    let win = canvas.window_mut();
    win.set_size(WINDOW_W, window_h(strips.len()))
        .map_err(|e| e.to_string())?;
    win.set_title(&title_for(strips))
        .map_err(|e| e.to_string())?;
    *layout = current;
    Ok(())
}

fn label(key: StripKey) -> String {
    let port = match key.port {
        n @ 1..=8 => ((b'A' + n - 1) as char).to_string(),
        n => n.to_string(),
    };
    match key.expander {
        Some(e) => format!("expander {e} ADI {port}"),
        None => format!("ADI {port}"),
    }
}

fn title_for(strips: &[(StripKey, Strip)]) -> String {
    let mut title = String::from("Antaeus AdiAddrLed simulator");
    for (i, (key, strip)) in strips.iter().enumerate() {
        let sep = if i == 0 { " - " } else { ", " };
        let _ = write!(title, "{sep}row {}: {} ({} px)", i + 1, label(*key), strip.len);
    }
    title
}

fn fill_circle(canvas: &mut Canvas<Window>, cx: i32, cy: i32, r: i32) -> Result<(), String> {
    for dy in -r..=r {
        let dx = ((r * r - dy * dy) as f32).sqrt() as i32;
        canvas.draw_line(Point::new(cx - dx, cy + dy), Point::new(cx + dx, cy + dy))?;
    }
    Ok(())
}

fn rgb(px: u32) -> (u8, u8, u8) { ((px >> 16) as u8, (px >> 8) as u8, px as u8) }

fn die_rect(cx: i32, cy: i32) -> Rect {
    Rect::new(cx - DIE / 2, cy - DIE / 2, DIE as u32, DIE as u32)
}

fn draw(canvas: &mut Canvas<Window>, strips: &[(StripKey, Strip)]) -> Result<(), String> {
    canvas.set_blend_mode(BlendMode::None);
    canvas.set_draw_color(SdlColor::RGB(0x0d, 0x0d, 0x12));
    canvas.clear();

    let row_cy = |row: usize| BANNER_H + MARGIN + row as i32 * ROW_H + ROW_H / 2;
    let col_cx = |i: usize| MARGIN + i as i32 * PITCH + PITCH / 2;

    for (row, (_, strip)) in strips.iter().enumerate() {
        let cy = row_cy(row);
        canvas.set_draw_color(SdlColor::RGB(0x1a, 0x1a, 0x21));
        canvas.fill_rect(Rect::new(
            MARGIN - 4,
            cy - DIE / 2 - 5,
            (strip.len as i32 * PITCH + 8) as u32,
            (DIE + 10) as u32,
        ))?;
        canvas.set_draw_color(SdlColor::RGB(0xb0, 0xb0, 0xb8));
        canvas.fill_rect(Rect::new(MARGIN - 9, cy - DIE / 2, 3, DIE as u32))?;
        canvas.set_draw_color(SdlColor::RGB(0x2c, 0x2c, 0x34));
        for i in 0..strip.len {
            canvas.fill_rect(die_rect(col_cx(i), cy))?;
        }
    }

    canvas.set_blend_mode(BlendMode::Add);
    for (row, (_, strip)) in strips.iter().enumerate() {
        let cy = row_cy(row);
        for i in 0..strip.len {
            let (r, g, b) = rgb(strip.px[i]);
            if (r | g | b) == 0 {
                continue;
            }
            canvas.set_draw_color(SdlColor::RGBA(r, g, b, GLOW_ALPHA));
            for ring in (1..=GLOW_RINGS).rev() {
                fill_circle(canvas, col_cx(i), cy, DIE / 2 + ring * GLOW_STEP)?;
            }
        }
    }

    canvas.set_blend_mode(BlendMode::None);
    for (row, (_, strip)) in strips.iter().enumerate() {
        let cy = row_cy(row);
        for i in 0..strip.len {
            let (r, g, b) = rgb(strip.px[i]);
            if (r | g | b) != 0 {
                canvas.set_draw_color(SdlColor::RGB(r, g, b));
                canvas.fill_rect(die_rect(col_cx(i), cy))?;
            }
        }
    }

    Ok(())
}

fn dump_frame(canvas: &Canvas<Window>, path: &Path) -> Result<(), String> {
    let (w, h) = canvas.output_size()?;
    let mut data = canvas.read_pixels(None::<Rect>, PixelFormatEnum::RGB24)?;
    let surface = Surface::from_data(&mut data, w, h, w * 3, PixelFormatEnum::RGB24)?;
    surface.save_bmp(path)
}
