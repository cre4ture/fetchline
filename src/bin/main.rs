#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_net::{Runner, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Timer};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use embedded_io_async::{Read, Write};
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
};
use esp_radio::wifi::{
    Config as WifiConfig, ControllerConfig, Interface, WifiController, sta::StationConfig,
};
use fetchline::board::{
    BRIDGE_TCP_PORT, OLED_CONTROLLER, OLED_HEIGHT, OLED_I2C_ADDRESS, OLED_WIDTH, SERVO_UART_BAUD,
    SERVO_UART_RX_GPIO, SERVO_UART_TX_GPIO,
};
use log::{info, warn};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

extern crate alloc;

use alloc::format;
use core::fmt;

// These values are compiled into the firmware. build.rs tracks both variables,
// so changing credentials always triggers a rebuild.
const WIFI_SSID: &str = match option_env!("WIFI_SSID") {
    Some(value) => value,
    None => "",
};
const WIFI_PASSWORD: &str = match option_env!("WIFI_PASSWORD") {
    Some(value) => value,
    None => "",
};

const TCP_BUFFER_SIZE: usize = 4096;
const COPY_BUFFER_SIZE: usize = 512;

// This creates the application descriptor required by the ESP-IDF bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($type:ty, $value:expr) => {{
        static CELL: static_cell::StaticCell<$type> = static_cell::StaticCell::new();
        CELL.uninit().write($value)
    }};
}

#[derive(Clone, Copy)]
enum CopyDirection {
    NetworkToUart,
    UartToNetwork,
}

#[derive(Clone, Copy)]
enum BridgeExit {
    InputClosed(CopyDirection),
    ReadError(CopyDirection),
    WriteError(CopyDirection),
}

impl fmt::Display for CopyDirection {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::NetworkToUart => formatter.write_str("network to UART"),
            Self::UartToNetwork => formatter.write_str("UART to network"),
        }
    }
}

impl fmt::Display for BridgeExit {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InputClosed(direction) => write!(formatter, "{direction} input closed"),
            Self::ReadError(direction) => write!(formatter, "{direction} read error"),
            Self::WriteError(direction) => write!(formatter, "{direction} write error"),
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the network stack and TCP buffers intentionally live for the lifetime of main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let i2c_config = I2cConfig::default().with_frequency(Rate::from_khz(400));
    let i2c = I2c::new(peripherals.I2C0, i2c_config)
        .expect("failed to configure I2C0")
        .with_sda(peripherals.GPIO5)
        .with_scl(peripherals.GPIO6);

    // SSD1315 implements the SSD1306-compatible commands used by this driver.
    let interface = I2CDisplayInterface::new_custom_address(i2c, OLED_I2C_ADDRESS);
    let mut display = Ssd1306::new(interface, DisplaySize72x40, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().expect("failed to initialize OLED");
    show_status(&mut display, ["FETCHLINE", "WIFI UART", "STARTING"]);
    display.flush().expect("failed to update OLED");

    info!(
        "{OLED_CONTROLLER} OLED initialized: {OLED_WIDTH}x{OLED_HEIGHT} at 0x{OLED_I2C_ADDRESS:02x}"
    );

    let uart_config = UartConfig::default().with_baudrate(SERVO_UART_BAUD);
    let uart = Uart::new(peripherals.UART1, uart_config)
        .expect("failed to configure UART1")
        .with_rx(peripherals.GPIO20)
        .with_tx(peripherals.GPIO21)
        .into_async();
    let (mut uart_rx, mut uart_tx) = uart.split();
    info!(
        "servo UART ready: GPIO{SERVO_UART_RX_GPIO} RX, GPIO{SERVO_UART_TX_GPIO} TX, \
         {SERVO_UART_BAUD} baud, 8N1"
    );

    let timer_group = TimerGroup::new(peripherals.TIMG0);
    let software_interrupt = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timer_group.timer0, software_interrupt.software_interrupt0);

    if WIFI_SSID.is_empty() {
        warn!("WIFI_SSID is empty; rebuild with WIFI_SSID and WIFI_PASSWORD set");
        show_status(&mut display, ["WIFI CONFIG", "SET BUILD", "VARIABLES"]);
        display.flush().expect("failed to update OLED");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }

    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(WIFI_SSID)
            .with_password(WIFI_PASSWORD.into()),
    );
    let (controller, interfaces) = esp_radio::wifi::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .expect("failed to initialize Wi-Fi");

    let network_config = embassy_net::Config::dhcpv4(Default::default());
    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;
    let (stack, runner) = embassy_net::new(
        interfaces.station,
        network_config,
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(wifi_connection(controller).expect("failed to allocate Wi-Fi connection task"));
    spawner.spawn(network_task(runner).expect("failed to allocate network task"));

    let mut tcp_rx_buffer = [0_u8; TCP_BUFFER_SIZE];
    let mut tcp_tx_buffer = [0_u8; TCP_BUFFER_SIZE];
    let mut network_to_uart_buffer = [0_u8; COPY_BUFFER_SIZE];
    let mut uart_to_network_buffer = [0_u8; COPY_BUFFER_SIZE];

    loop {
        stack.wait_config_up().await;
        if let Some(config) = stack.config_v4() {
            info!(
                "Wi-Fi ready: IP {}, raw TCP port {BRIDGE_TCP_PORT}",
                config.address
            );
            show_ip_address(&mut display, config.address.address());
            display.flush().expect("failed to update OLED");
        }

        let mut socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);
        socket.set_nagle_enabled(false);
        socket.set_keep_alive(Some(Duration::from_secs(10)));
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("waiting for one TCP client on port {BRIDGE_TCP_PORT}");
        if let Err(error) = socket.accept(BRIDGE_TCP_PORT).await {
            warn!("TCP accept failed: {error:?}");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        info!("TCP client connected: {:?}", socket.remote_endpoint());

        let result = {
            let (mut tcp_rx, mut tcp_tx) = socket.split();
            match select(
                forward(
                    &mut tcp_rx,
                    &mut uart_tx,
                    &mut network_to_uart_buffer,
                    CopyDirection::NetworkToUart,
                ),
                forward(
                    &mut uart_rx,
                    &mut tcp_tx,
                    &mut uart_to_network_buffer,
                    CopyDirection::UartToNetwork,
                ),
            )
            .await
            {
                Either::First(result) | Either::Second(result) => result,
            }
        };

        warn!("TCP bridge stopped: {result}");
        socket.abort();
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the async UART/TCP driver futures retain peripheral state across await points"
)]
async fn forward<R, W>(
    reader: &mut R,
    writer: &mut W,
    buffer: &mut [u8],
    direction: CopyDirection,
) -> BridgeExit
where
    R: Read,
    R::Error: fmt::Debug,
    W: Write,
    W::Error: fmt::Debug,
{
    let mut consecutive_read_errors = 0_u32;
    let mut forwarding_started = false;

    loop {
        let count = match reader.read(buffer).await {
            Ok(0) => return BridgeExit::InputClosed(direction),
            Ok(count) => {
                consecutive_read_errors = 0;
                count
            }
            Err(error) if matches!(direction, CopyDirection::UartToNetwork) => {
                consecutive_read_errors = consecutive_read_errors.saturating_add(1);
                if consecutive_read_errors <= 3 || consecutive_read_errors.is_power_of_two() {
                    warn!(
                        "servo UART RX error: {error:?} (consecutive: \
                         {consecutive_read_errors}); keeping TCP client connected"
                    );
                }
                Timer::after(Duration::from_millis(10)).await;
                continue;
            }
            Err(error) => {
                warn!("{direction} read error: {error:?}");
                return BridgeExit::ReadError(direction);
            }
        };

        if let Err(error) = writer.write_all(&buffer[..count]).await {
            warn!("{direction} write error: {error:?}");
            return BridgeExit::WriteError(direction);
        }

        if !forwarding_started {
            info!("{direction} forwarding started with {count} bytes");
            forwarding_started = true;
        }
    }
}

fn show_ip_address<D>(display: &mut D, address: core::net::Ipv4Addr)
where
    D: DrawTarget<Color = BinaryColor>,
{
    let [first, second, third, fourth] = address.octets();
    let first_line = format!("{first}.{second}.");
    let second_line = format!("{third}.{fourth}");

    show_status(
        display,
        ["IP ADDRESS", first_line.as_str(), second_line.as_str()],
    );
}

fn show_status<D>(display: &mut D, lines: [&str; 3])
where
    D: DrawTarget<Color = BinaryColor>,
{
    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    let centered = TextStyleBuilder::new()
        .alignment(Alignment::Center)
        .baseline(Baseline::Top)
        .build();

    let _ = display.clear(BinaryColor::Off);
    let _ = Rectangle::new(Point::zero(), Size::new(OLED_WIDTH, OLED_HEIGHT))
        .into_styled(PrimitiveStyle::with_stroke(BinaryColor::On, 1))
        .draw(display);

    for (line, y) in lines.into_iter().zip([4, 16, 28]) {
        let _ = Text::with_text_style(
            line,
            Point::new((OLED_WIDTH / 2) as i32, y),
            style,
            centered,
        )
        .draw(display);
    }
}

#[embassy_executor::task]
#[allow(
    clippy::large_stack_frames,
    reason = "the esp-radio connection future owns the controller state for this static task"
)]
async fn wifi_connection(mut controller: WifiController<'static>) {
    loop {
        info!("connecting to Wi-Fi SSID {WIFI_SSID:?}");
        match controller.connect_async().await {
            Ok(info) => {
                info!("Wi-Fi associated: {info:?}");
                let disconnected = controller.wait_for_disconnect_async().await.ok();
                warn!("Wi-Fi disconnected: {disconnected:?}");
            }
            Err(error) => warn!("Wi-Fi connection failed: {error:?}"),
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

#[embassy_executor::task]
async fn network_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await;
}
