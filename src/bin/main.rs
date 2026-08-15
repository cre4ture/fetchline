#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embedded_graphics::{
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config as I2cConfig, I2c},
    main,
    time::{Duration, Instant, Rate},
};
use fetchline::board::{OLED_HEIGHT, OLED_I2C_ADDRESS, OLED_WIDTH};
use log::info;
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[main]
fn main() -> ! {
    // generator version: 1.3.0
    // generator parameters: --chip esp32c3 -o log -o esp-backtrace -o ci

    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("failed to configure I2C0")
        .with_sda(peripherals.GPIO5)
        .with_scl(peripherals.GPIO6);

    let interface = I2CDisplayInterface::new_custom_address(i2c, OLED_I2C_ADDRESS);
    let mut display = Ssd1306::new(interface, DisplaySize72x40, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();

    display.init().expect("failed to initialize OLED");

    let text_style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let centered = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();

    Rectangle::new(Point::zero(), Size::new(OLED_WIDTH, OLED_HEIGHT))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(&mut display)
        .expect("failed to draw border");
    Text::with_text_style(
        "FETCHLINE",
        Point::new((OLED_WIDTH / 2) as i32, 4),
        text_style,
        centered,
    )
    .draw(&mut display)
    .expect("failed to draw title");
    Text::with_text_style(
        "ESP32-C3",
        Point::new((OLED_WIDTH / 2) as i32, 16),
        text_style,
        centered,
    )
    .draw(&mut display)
    .expect("failed to draw board name");
    Text::with_text_style(
        "READY",
        Point::new((OLED_WIDTH / 2) as i32, 28),
        text_style,
        centered,
    )
    .draw(&mut display)
    .expect("failed to draw status");
    display.flush().expect("failed to update OLED");

    info!("OLED initialized: {OLED_WIDTH}x{OLED_HEIGHT} at 0x{OLED_I2C_ADDRESS:02x}");

    loop {
        info!("fetchline is running");
        let delay_start = Instant::now();
        while delay_start.elapsed() < Duration::from_secs(5) {}
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.1.0/examples
}
