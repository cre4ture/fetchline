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
use embassy_time::{Duration, Timer, with_timeout};
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
    Async,
    clock::CpuClock,
    i2c::master::{Config as I2cConfig, I2c},
    interrupt::software::SoftwareInterruptControl,
    rng::Rng,
    time::Rate,
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart, UartRx, UartTx},
};
use esp_radio::wifi::{
    Config as WifiConfig, ControllerConfig, Interface, WifiController, sta::StationConfig,
};
use fetchline::board::{
    OLED_CONTROLLER, OLED_HEIGHT, OLED_I2C_ADDRESS, OLED_WIDTH, SERVO_UART_BAUD,
    SERVO_UART_RX_GPIO, SERVO_UART_TX_GPIO,
};
use fetchline_protocol::{
    CONTROLLER_TCP_PORT, Command, DecodeError, ErrorCode, FRAME_LEN, Frame, Response,
};
use log::{info, warn};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

extern crate alloc;

use alloc::format;
use core::fmt;

const TCP_BUFFER_SIZE: usize = 4096;
const COPY_BUFFER_SIZE: usize = 512;
const STS_RESPONSE_TIMEOUT: Duration = Duration::from_millis(50);
const STS_HEADER: [u8; 2] = [0xff, 0xff];
const STS_BROADCAST_ID: u8 = 0xfe;
const STS_INSTRUCTION_READ: u8 = 0x02;
const STS_INSTRUCTION_WRITE: u8 = 0x03;
const STS_MODE: u8 = 33;
const STS_TORQUE_ENABLE: u8 = 40;
const STS_ACCELERATION: u8 = 41;
const STS_TORQUE_LIMIT: u8 = 48;
const STS_PRESENT_POSITION: u8 = 56;
const WIFI_CONFIG_FLASH_OFFSET: usize = 0x003f_0000;
const WIFI_CONFIG_MAGIC: [u8; 4] = *b"FLWC";
const WIFI_CONFIG_VERSION: u8 = 1;
const WIFI_SSID_MAX_LEN: usize = 32;
const WIFI_PASSWORD_MAX_LEN: usize = 63;
// The ROM reader requires a four-byte aligned length.
const WIFI_CONFIG_READ_SIZE: usize = 108;

struct WifiCredentials {
    ssid: [u8; WIFI_SSID_MAX_LEN],
    ssid_len: usize,
    password: [u8; WIFI_PASSWORD_MAX_LEN],
    password_len: usize,
}

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

#[derive(Clone, Copy, Debug)]
enum ControllerSessionExit {
    PeerClosed,
    InvalidFrame(DecodeError),
    DebugTunnelRequested,
}

#[derive(Clone, Copy, Debug)]
enum ServoFailure {
    InvalidServoId,
    InvalidArgument,
    Timeout,
    InvalidReply,
    ReportedError(u8),
    Transport,
}

impl ServoFailure {
    const fn response(self) -> Response {
        match self {
            Self::InvalidServoId => Response::Error {
                code: ErrorCode::InvalidServoId,
                detail: 0,
            },
            Self::InvalidArgument => Response::Error {
                code: ErrorCode::InvalidArgument,
                detail: 0,
            },
            Self::Timeout => Response::Error {
                code: ErrorCode::ServoTimeout,
                detail: 0,
            },
            Self::InvalidReply => Response::Error {
                code: ErrorCode::InvalidServoReply,
                detail: 0,
            },
            Self::ReportedError(status) => Response::Error {
                code: ErrorCode::ServoReportedError,
                detail: status as u16,
            },
            Self::Transport => Response::Error {
                code: ErrorCode::ServoTransport,
                detail: 0,
            },
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
    show_status(&mut display, ["FETCHLINE", "WIFI STS", "STARTING"]);
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

    let wifi_credentials = match load_wifi_credentials() {
        Some(credentials) => credentials,
        None => {
            warn!("Wi-Fi credentials are not provisioned; use host provisioning over USB");
            show_status(&mut display, ["WIFI CONFIG", "USB PROVISION", "REQUIRED"]);
            display.flush().expect("failed to update OLED");
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };
    let wifi_ssid = core::str::from_utf8(&wifi_credentials.ssid[..wifi_credentials.ssid_len])
        .expect("provisioned Wi-Fi SSID must be UTF-8");
    let wifi_password =
        core::str::from_utf8(&wifi_credentials.password[..wifi_credentials.password_len])
            .expect("provisioned Wi-Fi password must be UTF-8");

    let station_config = WifiConfig::Station(
        StationConfig::default()
            .with_ssid(wifi_ssid)
            .with_password(wifi_password.into()),
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
                "Wi-Fi ready: IP {}, controller TCP port {CONTROLLER_TCP_PORT}",
                config.address
            );
            show_ip_address(&mut display, config.address.address());
            display.flush().expect("failed to update OLED");
        }

        let mut socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);
        socket.set_nagle_enabled(false);
        socket.set_keep_alive(Some(Duration::from_secs(10)));
        socket.set_timeout(Some(Duration::from_secs(30)));

        info!("waiting for one controller TCP client on port {CONTROLLER_TCP_PORT}");
        if let Err(error) = socket.accept(CONTROLLER_TCP_PORT).await {
            warn!("TCP accept failed: {error:?}");
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }

        info!(
            "controller TCP client connected: {:?}",
            socket.remote_endpoint()
        );

        match controller_session(&mut socket, &mut uart_rx, &mut uart_tx).await {
            ControllerSessionExit::PeerClosed => {
                info!("controller TCP client disconnected");
            }
            ControllerSessionExit::InvalidFrame(error) => {
                warn!("controller TCP session ended after invalid frame: {error:?}");
            }
            ControllerSessionExit::DebugTunnelRequested => {
                warn!("debug raw UART tunnel enabled for this TCP session");
                let result = raw_tunnel(
                    &mut socket,
                    &mut uart_rx,
                    &mut uart_tx,
                    &mut network_to_uart_buffer,
                    &mut uart_to_network_buffer,
                )
                .await;
                warn!("debug raw UART tunnel stopped: {result}");
            }
        }
        socket.abort();
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the async controller session retains the TCP socket and UART controller futures"
)]
async fn controller_session(
    socket: &mut TcpSocket<'_>,
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
) -> ControllerSessionExit {
    let mut bytes = [0_u8; FRAME_LEN];
    loop {
        if socket.read_exact(&mut bytes).await.is_err() {
            return ControllerSessionExit::PeerClosed;
        }
        let frame = match Frame::decode(bytes) {
            Ok(frame) => frame,
            Err(error) => return ControllerSessionExit::InvalidFrame(error),
        };
        let sequence = frame.sequence();
        let command = match frame.as_command() {
            Ok(command) => command,
            Err(DecodeError::UnknownMessage) => {
                if !send_controller_response(
                    socket,
                    sequence,
                    Response::Error {
                        code: ErrorCode::UnsupportedCommand,
                        detail: 0,
                    },
                )
                .await
                {
                    return ControllerSessionExit::PeerClosed;
                }
                continue;
            }
            Err(error) => {
                if !send_controller_response(
                    socket,
                    sequence,
                    Response::Error {
                        code: ErrorCode::InvalidRequest,
                        detail: 0,
                    },
                )
                .await
                {
                    return ControllerSessionExit::PeerClosed;
                }
                warn!("rejected malformed controller command: {error:?}");
                continue;
            }
        };

        if matches!(command, Command::OpenRawTunnel) {
            if send_controller_response(socket, sequence, Response::RawTunnelReady).await {
                return ControllerSessionExit::DebugTunnelRequested;
            }
            return ControllerSessionExit::PeerClosed;
        }

        let response = execute_command(command, uart_rx, uart_tx).await;
        if !send_controller_response(socket, sequence, response).await {
            return ControllerSessionExit::PeerClosed;
        }
    }
}

async fn send_controller_response(
    socket: &mut TcpSocket<'_>,
    sequence: u32,
    response: Response,
) -> bool {
    let bytes = Frame::response(sequence, response).encode();
    socket.write_all(&bytes).await.is_ok()
}

#[allow(
    clippy::large_stack_frames,
    reason = "each controller command retains local STS UART transaction futures"
)]
async fn execute_command(
    command: Command,
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
) -> Response {
    let result = match command {
        Command::Ping => Ok(Response::Ack),
        Command::StartMotor {
            id,
            counter_clockwise,
            speed,
            acceleration,
        } => start_motor(uart_rx, uart_tx, id, counter_clockwise, speed, acceleration)
            .await
            .map(|()| Response::Ack),
        Command::StopMotor { id } => stop_motor(uart_rx, uart_tx, id)
            .await
            .map(|()| Response::Ack),
        Command::SetPosition {
            id,
            position,
            acceleration,
            torque_limit,
        } => move_position(uart_rx, uart_tx, id, position, acceleration, torque_limit)
            .await
            .map(|()| Response::Ack),
        Command::ReadPosition { id } => read_position(uart_rx, uart_tx, id)
            .await
            .map(|position| Response::Position { id, position }),
        Command::OpenRawTunnel => unreachable!("debug tunnel is handled before serial execution"),
    };
    match result {
        Ok(response) => response,
        Err(error) => {
            warn!("local STS command failed: {error:?}; draining UART before continuing");
            resynchronize_uart(uart_rx).await;
            error.response()
        }
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS write sequence retains UART transaction futures"
)]
async fn start_motor(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
    counter_clockwise: bool,
    speed: u16,
    acceleration: u8,
) -> Result<(), ServoFailure> {
    validate_servo_id(id)?;
    if speed > 4095 {
        return Err(ServoFailure::InvalidArgument);
    }
    write_register(uart_rx, uart_tx, id, STS_MODE, &[1]).await?;
    write_register(uart_rx, uart_tx, id, STS_TORQUE_ENABLE, &[1]).await?;
    let signed_speed = if counter_clockwise {
        speed | 0x8000
    } else {
        speed
    };
    let mut command = [0_u8; 7];
    command[0] = acceleration;
    command[5..7].copy_from_slice(&signed_speed.to_le_bytes());
    write_register(uart_rx, uart_tx, id, STS_ACCELERATION, &command).await
}

async fn stop_motor(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
) -> Result<(), ServoFailure> {
    validate_servo_id(id)?;
    write_register(uart_rx, uart_tx, id, STS_ACCELERATION, &[0; 7]).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS write sequence retains UART transaction futures"
)]
async fn move_position(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
    position: u16,
    acceleration: u8,
    torque_limit: u16,
) -> Result<(), ServoFailure> {
    validate_servo_id(id)?;
    if position > 4095 || torque_limit > 1000 {
        return Err(ServoFailure::InvalidArgument);
    }
    write_register(uart_rx, uart_tx, id, STS_TORQUE_ENABLE, &[1]).await?;
    write_register(
        uart_rx,
        uart_tx,
        id,
        STS_TORQUE_LIMIT,
        &torque_limit.to_le_bytes(),
    )
    .await?;
    let mut command = [0_u8; 7];
    command[0] = acceleration;
    command[1..3].copy_from_slice(&position.to_le_bytes());
    write_register(uart_rx, uart_tx, id, STS_ACCELERATION, &command).await
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS read retains the UART response parser future"
)]
async fn read_position(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
) -> Result<i16, ServoFailure> {
    validate_servo_id(id)?;
    let status = send_sts_packet(
        uart_rx,
        uart_tx,
        id,
        STS_INSTRUCTION_READ,
        &[STS_PRESENT_POSITION, 2],
    )
    .await?;
    if status.payload_len != 2 {
        return Err(ServoFailure::InvalidReply);
    }
    let value = u16::from_le_bytes([status.payload[0], status.payload[1]]);
    Ok(decode_signed_15(value))
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS write retains the UART transaction future"
)]
async fn write_register(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
    address: u8,
    data: &[u8],
) -> Result<(), ServoFailure> {
    let mut parameters = [0_u8; 8];
    if data.len() > parameters.len() - 1 {
        return Err(ServoFailure::InvalidArgument);
    }
    parameters[0] = address;
    parameters[1..=data.len()].copy_from_slice(data);
    let status = send_sts_packet(
        uart_rx,
        uart_tx,
        id,
        STS_INSTRUCTION_WRITE,
        &parameters[..=data.len()],
    )
    .await?;
    if status.payload_len == 0 {
        Ok(())
    } else {
        Err(ServoFailure::InvalidReply)
    }
}

struct StsStatus {
    payload: [u8; 64],
    payload_len: usize,
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS transaction retains UART I/O and timeout futures"
)]
async fn send_sts_packet(
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    id: u8,
    instruction: u8,
    parameters: &[u8],
) -> Result<StsStatus, ServoFailure> {
    let length = parameters
        .len()
        .checked_add(2)
        .ok_or(ServoFailure::InvalidArgument)?;
    let length = u8::try_from(length).map_err(|_| ServoFailure::InvalidArgument)?;
    let mut packet = [0_u8; 16];
    let packet_len = parameters.len() + 6;
    if packet_len > packet.len() {
        return Err(ServoFailure::InvalidArgument);
    }
    packet[..2].copy_from_slice(&STS_HEADER);
    packet[2] = id;
    packet[3] = length;
    packet[4] = instruction;
    packet[5..5 + parameters.len()].copy_from_slice(parameters);
    packet[packet_len - 1] = checksum(&packet[2..packet_len - 1]);
    uart_tx
        .write_all(&packet[..packet_len])
        .await
        .map_err(|_| ServoFailure::Transport)?;

    match with_timeout(STS_RESPONSE_TIMEOUT, read_sts_status(uart_rx, id)).await {
        Ok(result) => result,
        Err(_) => Err(ServoFailure::Timeout),
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the local STS status parser retains UART read futures"
)]
async fn read_sts_status(
    uart_rx: &mut UartRx<'_, Async>,
    expected_id: u8,
) -> Result<StsStatus, ServoFailure> {
    let mut previous = 0_u8;
    let mut found_header = false;
    for _ in 0..128 {
        let byte = read_uart_byte(uart_rx).await?;
        if previous == 0xff && byte == 0xff {
            found_header = true;
            break;
        }
        previous = byte;
    }
    if !found_header {
        return Err(ServoFailure::InvalidReply);
    }
    let id = read_uart_byte(uart_rx).await?;
    let length = read_uart_byte(uart_rx).await? as usize;
    if id != expected_id || !(2..=66).contains(&length) {
        return Err(ServoFailure::InvalidReply);
    }
    let error = read_uart_byte(uart_rx).await?;
    let payload_len = length - 2;
    let mut payload = [0_u8; 64];
    for byte in &mut payload[..payload_len] {
        *byte = read_uart_byte(uart_rx).await?;
    }
    let received_checksum = read_uart_byte(uart_rx).await?;
    let mut checksum_bytes = [0_u8; 66];
    checksum_bytes[0] = error;
    checksum_bytes[1..1 + payload_len].copy_from_slice(&payload[..payload_len]);
    if received_checksum != status_checksum(id, length as u8, &checksum_bytes[..1 + payload_len]) {
        return Err(ServoFailure::InvalidReply);
    }
    if error != 0 {
        return Err(ServoFailure::ReportedError(error));
    }
    Ok(StsStatus {
        payload,
        payload_len,
    })
}

async fn read_uart_byte(uart_rx: &mut UartRx<'_, Async>) -> Result<u8, ServoFailure> {
    let mut byte = [0_u8; 1];
    uart_rx
        .read_async(&mut byte)
        .await
        .map_err(|_| ServoFailure::Transport)?;
    Ok(byte[0])
}

async fn resynchronize_uart(uart_rx: &mut UartRx<'_, Async>) {
    // `read_async` is cancellation-safe in esp-hal.  Waiting for a short quiet
    // interval and discarding any trailing bytes prevents an old STS response
    // from being associated with the next local command.
    let mut discarded = [0_u8; 64];
    for _ in 0..4 {
        if with_timeout(Duration::from_millis(2), uart_rx.read_async(&mut discarded))
            .await
            .is_err()
        {
            break;
        }
    }
}

const fn validate_servo_id(id: u8) -> Result<(), ServoFailure> {
    if id == 0 || id == STS_BROADCAST_ID || id == u8::MAX {
        Err(ServoFailure::InvalidServoId)
    } else {
        Ok(())
    }
}

const fn checksum(bytes: &[u8]) -> u8 {
    let mut sum = 0_u8;
    let mut index = 0;
    while index < bytes.len() {
        sum = sum.wrapping_add(bytes[index]);
        index += 1;
    }
    !sum
}

const fn status_checksum(id: u8, length: u8, bytes: &[u8]) -> u8 {
    let mut sum = id.wrapping_add(length);
    let mut index = 0;
    while index < bytes.len() {
        sum = sum.wrapping_add(bytes[index]);
        index += 1;
    }
    !sum
}

const fn decode_signed_15(value: u16) -> i16 {
    if value & 0x8000 != 0 {
        -((value & 0x7fff) as i16)
    } else {
        value as i16
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "the debug tunnel retains both TCP-to-UART forwarding futures"
)]
async fn raw_tunnel(
    socket: &mut TcpSocket<'_>,
    uart_rx: &mut UartRx<'_, Async>,
    uart_tx: &mut UartTx<'_, Async>,
    network_to_uart_buffer: &mut [u8],
    uart_to_network_buffer: &mut [u8],
) -> BridgeExit {
    let (mut tcp_rx, mut tcp_tx) = socket.split();
    match select(
        forward(
            &mut tcp_rx,
            uart_tx,
            network_to_uart_buffer,
            CopyDirection::NetworkToUart,
        ),
        forward(
            uart_rx,
            &mut tcp_tx,
            uart_to_network_buffer,
            CopyDirection::UartToNetwork,
        ),
    )
    .await
    {
        Either::First(result) | Either::Second(result) => result,
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
        info!("connecting to provisioned Wi-Fi network");
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

#[allow(
    clippy::large_stack_frames,
    reason = "the provisioned credential record is read once during startup"
)]
fn load_wifi_credentials() -> Option<WifiCredentials> {
    // The 4 MB flash has a dedicated 64 KB configuration area at its end. The
    // application image lives at the low end of flash, so normal `espflash
    // flash` updates leave this area untouched.
    // The application partition is the only flash range mapped into DROM at
    // startup. This record sits after that partition, so read it through the
    // ESP ROM instead of dereferencing a mapped address.
    let mut bytes = [0_u8; WIFI_CONFIG_READ_SIZE];
    let result = unsafe {
        esp_rom_sys::rom::spiflash::esp_rom_spiflash_read(
            WIFI_CONFIG_FLASH_OFFSET as u32,
            bytes.as_mut_ptr().cast(),
            WIFI_CONFIG_READ_SIZE as u32,
        )
    };
    if result != esp_rom_sys::rom::spiflash::ESP_ROM_SPIFLASH_RESULT_OK {
        return None;
    }
    if bytes[..4] != WIFI_CONFIG_MAGIC || bytes[4] != WIFI_CONFIG_VERSION {
        return None;
    }
    let ssid_len = bytes[5] as usize;
    let password_len = bytes[6] as usize;
    if ssid_len == 0 || ssid_len > WIFI_SSID_MAX_LEN || password_len > WIFI_PASSWORD_MAX_LEN {
        return None;
    }
    let payload_len = ssid_len + password_len;
    let expected_checksum = u32::from_le_bytes(bytes[8..12].try_into().ok()?);
    if wifi_config_checksum(&bytes[..8], &bytes[12..12 + payload_len]) != expected_checksum {
        return None;
    }
    let mut credentials = WifiCredentials {
        ssid: [0; WIFI_SSID_MAX_LEN],
        ssid_len,
        password: [0; WIFI_PASSWORD_MAX_LEN],
        password_len,
    };
    credentials.ssid[..ssid_len].copy_from_slice(&bytes[12..12 + ssid_len]);
    credentials.password[..password_len].copy_from_slice(&bytes[12 + ssid_len..12 + payload_len]);
    core::str::from_utf8(&credentials.ssid[..ssid_len]).ok()?;
    core::str::from_utf8(&credentials.password[..password_len]).ok()?;
    Some(credentials)
}

fn wifi_config_checksum(header: &[u8], payload: &[u8]) -> u32 {
    header
        .iter()
        .chain(payload)
        .fold(0x811c_9dc5_u32, |hash, byte| {
            (hash ^ u32::from(*byte)).wrapping_mul(0x0100_0193)
        })
}

#[embassy_executor::task]
async fn network_task(mut runner: Runner<'static, Interface<'static>>) {
    runner.run().await;
}
