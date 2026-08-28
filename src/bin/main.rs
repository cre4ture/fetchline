#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use embassy_executor::Spawner;
#[path = "../websocket.rs"]
mod websocket;

use embassy_futures::select::{Either, Either3, select, select3};
use embassy_net::{Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_sync::{
    blocking_mutex::{Mutex as BlockingMutex, raw::CriticalSectionRawMutex},
    mutex::Mutex,
    signal::Signal,
};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_graphics::{
    draw_target::DrawTarget,
    mono_font::{MonoTextStyle, ascii::FONT_6X10},
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{PrimitiveStyle, Rectangle},
    text::{Alignment, Baseline, Text, TextStyleBuilder},
};
use embedded_io_async::Write;
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
use fetchline_protocol::{CONTROLLER_TCP_PORT, RAW_TUNNEL_TCP_PORT};
use log::{info, warn};
use serde::{Deserialize, Serialize, ser::SerializeStruct};
use ssd1306::{I2CDisplayInterface, Ssd1306, prelude::*};

extern crate alloc;

use alloc::{boxed::Box, format};
use core::cell::Cell;

const TCP_BUFFER_SIZE: usize = 4096;
const COPY_BUFFER_SIZE: usize = 512;
const JSON_RPC_BUFFER_SIZE: usize = 1024;
const MAX_POSITION_BATCH: usize = 6;
const STS_RESPONSE_TIMEOUT: Duration = Duration::from_millis(50);
const STS_HEADER: [u8; 2] = [0xff, 0xff];
const STS_BROADCAST_ID: u8 = 0xfe;
// STS reserves 254 for broadcast; valid unicast servo IDs end at 253.
const MAX_SERVO_ID: u8 = STS_BROADCAST_ID - 1;
const MAX_SCAN_RESULTS: usize = MAX_SERVO_ID as usize;
const STS_INSTRUCTION_PING: u8 = 0x01;
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

#[derive(Clone, Copy, Debug)]
enum ServoFailure {
    InvalidServoId,
    InvalidArgument,
    Timeout,
    InvalidReply,
    ReportedError,
    Transport,
}

impl ServoFailure {
    const fn rpc_failure(self) -> RpcFailure {
        match self {
            Self::InvalidServoId | Self::InvalidArgument => RpcFailure::INVALID_PARAMS,
            Self::Timeout => RpcFailure::SERVO_TIMEOUT,
            Self::InvalidReply => RpcFailure::SERVO_REPLY,
            Self::ReportedError => RpcFailure::SERVO_REPORTED_ERROR,
            Self::Transport => RpcFailure::SERVO_TRANSPORT,
        }
    }
}

type SharedUartRx = Mutex<CriticalSectionRawMutex, UartRx<'static, Async>>;
type SharedUartTx = Mutex<CriticalSectionRawMutex, UartTx<'static, Async>>;

#[derive(Clone, Copy)]
enum RawTunnelCommand {
    Enable,
    Disable,
}

struct RawTunnelControl {
    enabled: BlockingMutex<CriticalSectionRawMutex, Cell<bool>>,
    command: Signal<CriticalSectionRawMutex, RawTunnelCommand>,
}

impl RawTunnelControl {
    const fn new() -> Self {
        Self {
            enabled: BlockingMutex::new(Cell::new(false)),
            command: Signal::new(),
        }
    }

    fn is_enabled(&self) -> bool {
        self.enabled.lock(|enabled| enabled.get())
    }

    fn enable(&self) {
        let changed = self.enabled.lock(|enabled| {
            if enabled.get() {
                false
            } else {
                enabled.set(true);
                true
            }
        });
        if changed {
            self.command.signal(RawTunnelCommand::Enable);
        }
    }

    fn disable(&self) {
        let changed = self.enabled.lock(|enabled| {
            if enabled.get() {
                enabled.set(false);
                true
            } else {
                false
            }
        });
        if changed {
            self.command.signal(RawTunnelCommand::Disable);
        }
    }
}

static RAW_TUNNEL: RawTunnelControl = RawTunnelControl::new();

#[derive(Deserialize)]
struct JsonRpcRequest<'a> {
    jsonrpc: &'a str,
    method: &'a str,
    #[serde(default)]
    params: JsonRpcParams,
    #[serde(default)]
    id: Option<u32>,
}

#[derive(Default, Deserialize)]
struct JsonRpcParams {
    #[serde(default)]
    id: Option<u8>,
    #[serde(default)]
    speed: Option<u16>,
    #[serde(default)]
    acceleration: Option<u8>,
    #[serde(default)]
    direction: Option<MotorDirection>,
    #[serde(default)]
    position: Option<u16>,
    #[serde(rename = "torqueLimit", default)]
    torque_limit: Option<u16>,
    #[serde(default)]
    ids: Option<serde_json_core::heapless::Vec<u8, MAX_POSITION_BATCH>>,
    #[serde(rename = "startId", default)]
    start_id: Option<u8>,
    #[serde(rename = "endId", default)]
    end_id: Option<u8>,
}

#[derive(Clone, Copy, Deserialize)]
#[serde(rename_all = "lowercase")]
enum MotorDirection {
    Clockwise,
    Counterclockwise,
}

enum ControllerCommand {
    StartMotor {
        id: u8,
        counter_clockwise: bool,
        speed: u16,
        acceleration: u8,
    },
    StopMotor {
        id: u8,
    },
    SetPosition {
        id: u8,
        position: u16,
        acceleration: u8,
        torque_limit: u16,
    },
    GetPosition {
        id: u8,
    },
    GetPositions {
        ids: serde_json_core::heapless::Vec<u8, MAX_POSITION_BATCH>,
    },
    ScanServos {
        start_id: u8,
        end_id: u8,
    },
}

struct PositionResult {
    id: u8,
    position: i16,
}

impl Serialize for PositionResult {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        let mut state = serializer.serialize_struct("PositionResult", 2)?;
        state.serialize_field("id", &self.id)?;
        state.serialize_field("position", &self.position)?;
        state.end()
    }
}

enum ControllerResult {
    Ready,
    Accepted,
    Position(PositionResult),
    Positions(serde_json_core::heapless::Vec<PositionResult, MAX_POSITION_BATCH>),
    Servos(Box<serde_json_core::heapless::Vec<u8, MAX_SCAN_RESULTS>>),
    RawTunnel { active: bool },
}

impl Serialize for ControllerResult {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        match self {
            Self::Ready => {
                let mut state = serializer.serialize_struct("Ready", 1)?;
                state.serialize_field("ready", &true)?;
                state.end()
            }
            Self::Accepted => {
                let mut state = serializer.serialize_struct("Accepted", 1)?;
                state.serialize_field("accepted", &true)?;
                state.end()
            }
            Self::Position(position) => position.serialize(serializer),
            Self::Positions(positions) => {
                let mut state = serializer.serialize_struct("Positions", 1)?;
                state.serialize_field("positions", positions)?;
                state.end()
            }
            Self::Servos(ids) => {
                let mut state = serializer.serialize_struct("Servos", 1)?;
                state.serialize_field("ids", &**ids)?;
                state.end()
            }
            Self::RawTunnel { active } if *active => {
                let mut state = serializer.serialize_struct("RawTunnel", 2)?;
                state.serialize_field("port", &RAW_TUNNEL_TCP_PORT)?;
                state.serialize_field("active", active)?;
                state.end()
            }
            Self::RawTunnel { active } => {
                let mut state = serializer.serialize_struct("RawTunnel", 1)?;
                state.serialize_field("active", active)?;
                state.end()
            }
        }
    }
}

#[derive(Clone, Copy)]
struct RpcFailure {
    code: i32,
    message: &'static str,
}

impl RpcFailure {
    const PARSE_ERROR: Self = Self {
        code: -32700,
        message: "Parse error",
    };
    const INVALID_REQUEST: Self = Self {
        code: -32600,
        message: "Invalid Request",
    };
    const METHOD_NOT_FOUND: Self = Self {
        code: -32601,
        message: "Method not found",
    };
    const INVALID_PARAMS: Self = Self {
        code: -32602,
        message: "Invalid params",
    };
    const RAW_TUNNEL_ACTIVE: Self = Self {
        code: -32010,
        message: "Raw tunnel is active",
    };
    const SERVO_TIMEOUT: Self = Self {
        code: -32001,
        message: "STS servo timeout",
    };
    const SERVO_REPLY: Self = Self {
        code: -32002,
        message: "Invalid STS servo reply",
    };
    const SERVO_REPORTED_ERROR: Self = Self {
        code: -32003,
        message: "Servo reported STS error",
    };
    const SERVO_TRANSPORT: Self = Self {
        code: -32004,
        message: "STS UART transport failure",
    };
}

#[derive(Serialize)]
struct JsonRpcSuccess<'a> {
    jsonrpc: &'static str,
    result: &'a ControllerResult,
    id: u32,
}

#[derive(Serialize)]
struct JsonRpcError {
    jsonrpc: &'static str,
    error: JsonRpcErrorBody,
    id: Option<u32>,
}

#[derive(Serialize)]
struct JsonRpcErrorBody {
    code: i32,
    message: &'static str,
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
    let (uart_rx, uart_tx) = uart.split();
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
        mk_static!(StackResources<4>, StackResources::<4>::new()),
        seed,
    );

    spawner.spawn(wifi_connection(controller).expect("failed to allocate Wi-Fi connection task"));
    spawner.spawn(network_task(runner).expect("failed to allocate network task"));

    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        info!(
            "Wi-Fi ready: IP {}, JSON-RPC WebSocket port {CONTROLLER_TCP_PORT}",
            config.address
        );
        show_ip_address(&mut display, config.address.address());
        display.flush().expect("failed to update OLED");
    }

    let uart_rx = mk_static!(SharedUartRx, Mutex::new(uart_rx));
    let uart_tx = mk_static!(SharedUartTx, Mutex::new(uart_tx));
    spawner.spawn(
        services::controller_service(stack, uart_rx, uart_tx)
            .expect("failed to allocate JSON-RPC controller task"),
    );
    spawner.spawn(
        services::raw_tunnel_service(stack, uart_rx, uart_tx)
            .expect("failed to allocate raw tunnel task"),
    );

    loop {
        Timer::after(Duration::from_secs(60)).await;
    }
}

#[allow(
    clippy::large_stack_frames,
    reason = "controller and raw TCP task frames contain fixed, statically allocated protocol buffers"
)]
mod services {
    use super::*;

    #[embassy_executor::task]
    #[allow(
        clippy::large_stack_frames,
        reason = "the JSON-RPC service keeps one WebSocket and its TCP buffers for its task lifetime"
    )]
    pub(super) async fn controller_service(
        stack: Stack<'static>,
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) -> ! {
        let mut tcp_rx_buffer = [0_u8; TCP_BUFFER_SIZE];
        let mut tcp_tx_buffer = [0_u8; TCP_BUFFER_SIZE];
        loop {
            stack.wait_config_up().await;
            let mut socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);
            socket.set_nagle_enabled(false);
            socket.set_keep_alive(Some(Duration::from_secs(10)));
            socket.set_timeout(Some(Duration::from_secs(30)));

            info!("waiting for JSON-RPC WebSocket client on port {CONTROLLER_TCP_PORT}");
            if let Err(error) = socket.accept(CONTROLLER_TCP_PORT).await {
                warn!("controller TCP accept failed: {error:?}");
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
            info!(
                "controller TCP client connected: {:?}",
                socket.remote_endpoint()
            );
            match websocket::upgrade(&mut socket).await {
                Ok(()) => controller_session(&mut socket, uart_rx, uart_tx).await,
                Err(error) => warn!("rejected controller WebSocket handshake: {error:?}"),
            }
            socket.abort();
        }
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "each JSON-RPC message owns bounded request and response buffers"
    )]
    async fn controller_session(
        socket: &mut TcpSocket<'_>,
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) {
        let mut request_buffer = [0_u8; JSON_RPC_BUFFER_SIZE];
        let mut response_buffer = [0_u8; JSON_RPC_BUFFER_SIZE];
        loop {
            let request = match websocket::read_text(socket, &mut request_buffer).await {
                Ok(request) => request,
                Err(websocket::Error::Closed) => {
                    info!("controller WebSocket client disconnected");
                    return;
                }
                Err(error) => {
                    warn!("controller WebSocket session ended: {error:?}");
                    return;
                }
            };
            let Some(response_len) =
                handle_json_rpc(request, &mut response_buffer, uart_rx, uart_tx).await
            else {
                continue;
            };
            if let Err(error) =
                websocket::write_text(socket, &response_buffer[..response_len]).await
            {
                warn!("could not send JSON-RPC response: {error:?}");
                return;
            }
        }
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "a controller command keeps MCU-local STS transaction futures alive"
    )]
    async fn handle_json_rpc(
        text: &str,
        response: &mut [u8],
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) -> Option<usize> {
        let request = match serde_json_core::from_slice::<JsonRpcRequest<'_>>(text.as_bytes()) {
            Ok((request, _)) => request,
            Err(_) => return encode_error(response, None, RpcFailure::PARSE_ERROR),
        };
        if request.jsonrpc != "2.0" {
            return encode_error(response, request.id, RpcFailure::INVALID_REQUEST);
        }

        let result = match request.method {
            "system.ping" => Ok(ControllerResult::Ready),
            "debug.enableRawTunnel" => {
                RAW_TUNNEL.enable();
                Ok(ControllerResult::RawTunnel { active: true })
            }
            "debug.disableRawTunnel" => {
                RAW_TUNNEL.disable();
                Ok(ControllerResult::RawTunnel { active: false })
            }
            method => {
                if RAW_TUNNEL.is_enabled() {
                    Err(RpcFailure::RAW_TUNNEL_ACTIVE)
                } else {
                    match command_from_request(method, request.params) {
                        Ok(command) => execute_command(command, uart_rx, uart_tx)
                            .await
                            .map_err(ServoFailure::rpc_failure),
                        Err(error) => Err(error),
                    }
                }
            }
        };

        let id = request.id?;
        match result {
            Ok(result) => encode_success(response, id, &result),
            Err(error) => encode_error(response, Some(id), error),
        }
    }

    fn command_from_request(
        method: &str,
        params: JsonRpcParams,
    ) -> Result<ControllerCommand, RpcFailure> {
        match method {
            "motor.start" => Ok(ControllerCommand::StartMotor {
                id: params.id.ok_or(RpcFailure::INVALID_PARAMS)?,
                counter_clockwise: matches!(
                    params.direction.ok_or(RpcFailure::INVALID_PARAMS)?,
                    MotorDirection::Counterclockwise
                ),
                speed: params.speed.ok_or(RpcFailure::INVALID_PARAMS)?,
                acceleration: params.acceleration.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            "motor.stop" => Ok(ControllerCommand::StopMotor {
                id: params.id.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            "servo.setPosition" => Ok(ControllerCommand::SetPosition {
                id: params.id.ok_or(RpcFailure::INVALID_PARAMS)?,
                position: params.position.ok_or(RpcFailure::INVALID_PARAMS)?,
                acceleration: params.acceleration.ok_or(RpcFailure::INVALID_PARAMS)?,
                torque_limit: params.torque_limit.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            "servo.getPosition" => Ok(ControllerCommand::GetPosition {
                id: params.id.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            "servo.getPositions" => Ok(ControllerCommand::GetPositions {
                ids: params.ids.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            "servo.scan" => Ok(ControllerCommand::ScanServos {
                start_id: params.start_id.ok_or(RpcFailure::INVALID_PARAMS)?,
                end_id: params.end_id.ok_or(RpcFailure::INVALID_PARAMS)?,
            }),
            _ => Err(RpcFailure::METHOD_NOT_FOUND),
        }
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "a controller command owns bounded RPC results and local STS transaction futures"
    )]
    async fn execute_command(
        command: ControllerCommand,
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) -> Result<ControllerResult, ServoFailure> {
        let mut uart_rx = uart_rx.lock().await;
        let mut uart_tx = uart_tx.lock().await;
        let result = match command {
            ControllerCommand::StartMotor {
                id,
                counter_clockwise,
                speed,
                acceleration,
            } => start_motor(
                &mut uart_rx,
                &mut uart_tx,
                id,
                counter_clockwise,
                speed,
                acceleration,
            )
            .await
            .map(|()| ControllerResult::Accepted),
            ControllerCommand::StopMotor { id } => stop_motor(&mut uart_rx, &mut uart_tx, id)
                .await
                .map(|()| ControllerResult::Accepted),
            ControllerCommand::SetPosition {
                id,
                position,
                acceleration,
                torque_limit,
            } => move_position(
                &mut uart_rx,
                &mut uart_tx,
                id,
                position,
                acceleration,
                torque_limit,
            )
            .await
            .map(|()| ControllerResult::Accepted),
            ControllerCommand::GetPosition { id } => read_position(&mut uart_rx, &mut uart_tx, id)
                .await
                .map(|position| ControllerResult::Position(PositionResult { id, position })),
            ControllerCommand::GetPositions { ids } => {
                read_positions(&mut uart_rx, &mut uart_tx, ids)
                    .await
                    .map(ControllerResult::Positions)
            }
            ControllerCommand::ScanServos { start_id, end_id } => {
                scan_servos(&mut uart_rx, &mut uart_tx, start_id, end_id)
                    .await
                    .map(|ids| ControllerResult::Servos(Box::new(ids)))
            }
        };
        if result.is_err() {
            warn!("local STS command failed; draining UART before continuing");
            resynchronize_uart(&mut uart_rx).await;
        }
        result
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "the bounded six-servo result contains local STS parser futures"
    )]
    async fn read_positions(
        uart_rx: &mut UartRx<'_, Async>,
        uart_tx: &mut UartTx<'_, Async>,
        ids: serde_json_core::heapless::Vec<u8, MAX_POSITION_BATCH>,
    ) -> Result<serde_json_core::heapless::Vec<PositionResult, MAX_POSITION_BATCH>, ServoFailure>
    {
        let mut positions = serde_json_core::heapless::Vec::new();
        for id in ids {
            let position = read_position(uart_rx, uart_tx, id).await?;
            positions
                .push(PositionResult { id, position })
                .map_err(|_| ServoFailure::InvalidArgument)?;
        }
        Ok(positions)
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "a full bus scan retains bounded local STS transaction futures"
    )]
    async fn scan_servos(
        uart_rx: &mut UartRx<'_, Async>,
        uart_tx: &mut UartTx<'_, Async>,
        start_id: u8,
        end_id: u8,
    ) -> Result<serde_json_core::heapless::Vec<u8, MAX_SCAN_RESULTS>, ServoFailure> {
        if start_id == 0 || start_id > end_id || end_id > MAX_SERVO_ID {
            return Err(ServoFailure::InvalidArgument);
        }

        let mut ids = serde_json_core::heapless::Vec::new();
        for raw_id in u16::from(start_id)..=u16::from(end_id) {
            let id = raw_id as u8;
            // This remains a defence in depth in case scan range validation is
            // changed later: never transmit to broadcast or invalid addresses.
            if !is_unicast_servo_id(id) {
                continue;
            }
            match ping_servo(uart_rx, uart_tx, id).await {
                Ok(()) => ids.push(id).map_err(|_| ServoFailure::InvalidArgument)?,
                // A timeout is the expected result for an unused address. An
                // invalid/status reply can be leftover bus data; drain it
                // before probing the next address.
                Err(ServoFailure::Transport) => {
                    resynchronize_uart(uart_rx).await;
                    return Err(ServoFailure::Transport);
                }
                Err(_) => resynchronize_uart(uart_rx).await,
            }
        }
        Ok(ids)
    }

    fn encode_success(response: &mut [u8], id: u32, result: &ControllerResult) -> Option<usize> {
        serde_json_core::to_slice(
            &JsonRpcSuccess {
                jsonrpc: "2.0",
                result,
                id,
            },
            response,
        )
        .ok()
    }

    fn encode_error(response: &mut [u8], id: Option<u32>, failure: RpcFailure) -> Option<usize> {
        serde_json_core::to_slice(
            &JsonRpcError {
                jsonrpc: "2.0",
                error: JsonRpcErrorBody {
                    code: failure.code,
                    message: failure.message,
                },
                id,
            },
            response,
        )
        .ok()
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
        reason = "the local STS ping retains the UART response parser future"
    )]
    async fn ping_servo(
        uart_rx: &mut UartRx<'_, Async>,
        uart_tx: &mut UartTx<'_, Async>,
        id: u8,
    ) -> Result<(), ServoFailure> {
        validate_servo_id(id)?;
        send_sts_packet(uart_rx, uart_tx, id, STS_INSTRUCTION_PING, &[])
            .await
            .map(|_| ())
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
        if received_checksum
            != status_checksum(id, length as u8, &checksum_bytes[..1 + payload_len])
        {
            return Err(ServoFailure::InvalidReply);
        }
        if error != 0 {
            return Err(ServoFailure::ReportedError);
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
        if !is_unicast_servo_id(id) {
            Err(ServoFailure::InvalidServoId)
        } else {
            Ok(())
        }
    }

    const fn is_unicast_servo_id(id: u8) -> bool {
        id != 0 && id <= MAX_SERVO_ID
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

    #[derive(Clone, Copy, Debug)]
    enum RawClientExit {
        ClientClosed,
        Disabled,
        NetworkError,
    }

    #[embassy_executor::task]
    #[allow(
        clippy::large_stack_frames,
        reason = "the raw listener keeps a separate TCP socket and bounded forwarding buffers"
    )]
    pub(super) async fn raw_tunnel_service(
        stack: Stack<'static>,
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) -> ! {
        let mut tcp_rx_buffer = [0_u8; TCP_BUFFER_SIZE];
        let mut tcp_tx_buffer = [0_u8; TCP_BUFFER_SIZE];
        loop {
            if !RAW_TUNNEL.is_enabled() {
                RAW_TUNNEL.command.wait().await;
                continue;
            }

            stack.wait_config_up().await;
            let mut socket = TcpSocket::new(stack, &mut tcp_rx_buffer, &mut tcp_tx_buffer);
            socket.set_nagle_enabled(false);
            socket.set_keep_alive(Some(Duration::from_secs(10)));
            socket.set_timeout(None);
            info!("raw UART debug tunnel listening on port {RAW_TUNNEL_TCP_PORT}");

            match select(
                RAW_TUNNEL.command.wait(),
                socket.accept(RAW_TUNNEL_TCP_PORT),
            )
            .await
            {
                Either::First(RawTunnelCommand::Disable) => {
                    socket.abort();
                    info!("raw UART debug tunnel disabled before a client connected");
                }
                Either::First(RawTunnelCommand::Enable) => {
                    socket.abort();
                }
                Either::Second(Ok(())) => {
                    info!("raw UART client connected: {:?}", socket.remote_endpoint());
                    let exit = raw_client_session(&mut socket, uart_rx, uart_tx).await;
                    socket.abort();
                    match exit {
                        RawClientExit::ClientClosed => {
                            info!("raw UART client disconnected; tunnel remains enabled");
                        }
                        RawClientExit::Disabled => {
                            info!("raw UART tunnel disabled while its client was connected");
                        }
                        RawClientExit::NetworkError => {
                            warn!("raw UART client connection failed; tunnel remains enabled");
                        }
                    }
                }
                Either::Second(Err(error)) => {
                    warn!("raw tunnel TCP accept failed: {error:?}");
                    socket.abort();
                    Timer::after(Duration::from_secs(1)).await;
                }
            }
        }
    }

    #[allow(
        clippy::large_stack_frames,
        reason = "raw forwarding owns bounded network and UART buffers while a client is connected"
    )]
    async fn raw_client_session(
        socket: &mut TcpSocket<'_>,
        uart_rx: &'static SharedUartRx,
        uart_tx: &'static SharedUartTx,
    ) -> RawClientExit {
        let mut network_to_uart = [0_u8; COPY_BUFFER_SIZE];
        let mut uart_to_network = [0_u8; COPY_BUFFER_SIZE];
        let mut uart_read_errors = 0_u32;
        loop {
            let uart_read = async {
                let mut uart_rx = uart_rx.lock().await;
                uart_rx.read_async(&mut uart_to_network).await
            };
            match select3(
                RAW_TUNNEL.command.wait(),
                socket.read(&mut network_to_uart),
                uart_read,
            )
            .await
            {
                Either3::First(RawTunnelCommand::Disable) => return RawClientExit::Disabled,
                Either3::First(RawTunnelCommand::Enable) => continue,
                Either3::Second(Ok(0)) => return RawClientExit::ClientClosed,
                Either3::Second(Ok(count)) => {
                    let mut uart_tx = uart_tx.lock().await;
                    if uart_tx.write_all(&network_to_uart[..count]).await.is_err() {
                        return RawClientExit::NetworkError;
                    }
                }
                Either3::Second(Err(error)) => {
                    warn!("raw tunnel TCP read error: {error:?}");
                    return RawClientExit::NetworkError;
                }
                Either3::Third(Ok(0)) => continue,
                Either3::Third(Ok(count)) => {
                    uart_read_errors = 0;
                    if socket.write_all(&uart_to_network[..count]).await.is_err() {
                        return RawClientExit::NetworkError;
                    }
                }
                Either3::Third(Err(error)) => {
                    uart_read_errors = uart_read_errors.saturating_add(1);
                    if uart_read_errors <= 3 || uart_read_errors.is_power_of_two() {
                        warn!(
                            "servo UART RX error during raw tunnel: {error:?} (consecutive: {uart_read_errors})"
                        );
                    }
                    Timer::after(Duration::from_millis(10)).await;
                }
            }
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
