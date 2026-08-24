//! Linux-local control panel for Feetech STS servos behind a fetchline bridge.
//!
//! A browser cannot make a raw TCP connection to the ESP32.  This program owns
//! that connection and exposes an HTTP/WebSocket interface to the bundled
//! browser UI.

use std::{
    env, fs,
    io::Write,
    net::SocketAddr,
    path::{Path, PathBuf},
    sync::{Arc, Mutex as StdMutex, OnceLock},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use axum::{
    Router,
    extract::{
        ConnectInfo, State, WebSocketUpgrade,
        ws::{Message, WebSocket},
    },
    http::{HeaderValue, StatusCode, header},
    response::{IntoResponse, Response},
    routing::get,
    Json,
};
use fetchline_protocol::{Command, ErrorCode, FRAME_LEN, Frame, Response as ControllerResponse};
use serde::{Deserialize, Serialize};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    sync::Mutex,
    time::timeout,
};

const DEFAULT_LISTEN_ADDRESS: &str = "0.0.0.0:8787";
const SERVO_TIMEOUT: Duration = Duration::from_millis(750);
const MAX_STALE_CONTROLLER_RESPONSES: usize = 32;
const STS_HEADER: [u8; 2] = [0xff, 0xff];
const STS_BROADCAST_ID: u8 = 0xfe;
const STS_INSTRUCTION_READ: u8 = 0x02;
const STS_INSTRUCTION_WRITE: u8 = 0x03;
const STS_MODE: u8 = 33;
const STS_TORQUE_ENABLE: u8 = 40;
const STS_ACCELERATION: u8 = 41;
const STS_TORQUE_LIMIT: u8 = 48;
const STS_PRESENT_POSITION: u8 = 56;
const MAX_LOG_FILE_BYTES: u64 = 5 * 1024 * 1024;
const LOG_HISTORY_FILES: u8 = 3;

static LOGGER: OnceLock<FileLogger> = OnceLock::new();

struct FileLogger {
    file: StdMutex<fs::File>,
    path: PathBuf,
}

impl log::Log for FileLogger {
    fn enabled(&self, metadata: &log::Metadata<'_>) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &log::Record<'_>) {
        if !self.enabled(record.metadata()) {
            return;
        }

        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|duration| format!("{}.{:03}", duration.as_secs(), duration.subsec_millis()))
            .unwrap_or_else(|_| "before-unix-epoch".to_owned());
        let line = format!("{timestamp} {:<5} {}", record.level(), record.args());
        eprintln!("{line}");

        let Ok(mut file) = self.file.lock() else {
            return;
        };
        if file
            .metadata()
            .is_ok_and(|metadata| metadata.len() >= MAX_LOG_FILE_BYTES)
            && let Ok(replacement) = rotate_log_file(&self.path)
        {
            *file = replacement;
        }
        let _ = writeln!(file, "{line}");
        let _ = file.flush();
    }

    fn flush(&self) {
        if let Ok(mut file) = self.file.lock() {
            let _ = file.flush();
        }
    }
}

#[derive(Clone)]
struct AppState {
    bridge: Arc<Mutex<Option<BridgeConnection>>>,
    config: Arc<Mutex<HostConfig>>,
    config_path: Arc<PathBuf>,
    transport: TransportMode,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            bridge: Arc::new(Mutex::new(None)),
            config: Arc::new(Mutex::new(HostConfig::default())),
            config_path: Arc::new(PathBuf::from("fetchline-host-config.json")),
            transport: TransportMode::Controller,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum TransportMode {
    /// The default: the MCU terminates STS locally and exposes controller commands.
    Controller,
    /// Test-only compatibility path activated by the protocol debug command.
    DebugRawTunnel,
}

struct BridgeConnection {
    peer: String,
    stream: TcpStream,
    transport: TransportMode,
    next_sequence: u32,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct HostConfig {
    endpoint: EndpointConfig,
    motor: MotorConfig,
    joints: Vec<JointConfig>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct EndpointConfig {
    host: String,
    port: u16,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct MotorConfig {
    id: u8,
    enabled: bool,
    #[serde(rename = "speedPercent")]
    speed_percent: u8,
    acceleration: u8,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct JointConfig {
    id: u8,
    enabled: bool,
    acceleration: u8,
    #[serde(rename = "torquePercent")]
    torque_percent: u8,
}

impl Default for HostConfig {
    fn default() -> Self {
        Self {
            endpoint: EndpointConfig {
                host: "192.168.1.123".to_owned(),
                port: 3333,
            },
            motor: MotorConfig {
                id: 1,
                enabled: true,
                speed_percent: 25,
                acceleration: 20,
            },
            joints: (2..=7)
                .map(|id| JointConfig {
                    id,
                    enabled: true,
                    acceleration: 20,
                    torque_percent: 100,
                })
                .collect(),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ClientMessage {
    Connect {
        host: String,
        port: u16,
    },
    StartMotor {
        id: u8,
        speed: u16,
        acceleration: u8,
        direction: Direction,
    },
    StopMotor {
        id: u8,
    },
    MovePosition {
        id: u8,
        position: u16,
        acceleration: u8,
        torque_limit: u16,
    },
    ReadPosition {
        id: u8,
    },
    ReadPositions {
        ids: Vec<u8>,
    },
}

impl ClientMessage {
    fn summary(&self) -> String {
        match self {
            Self::Connect { host, port } => format!("connect MCU {host}:{port}"),
            Self::StartMotor {
                id,
                speed,
                acceleration,
                direction,
            } => format!(
                "start motor servo={id} direction={direction:?} speed={speed} acceleration={acceleration}"
            ),
            Self::StopMotor { id } => format!("stop motor servo={id}"),
            Self::MovePosition {
                id,
                position,
                acceleration,
                torque_limit,
            } => format!(
                "move servo={id} position={position} acceleration={acceleration} torque_limit={torque_limit}"
            ),
            Self::ReadPosition { id } => format!("read position servo={id}"),
            Self::ReadPositions { ids } => format!("read positions servos={ids:?}"),
        }
    }
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "snake_case")]
enum Direction {
    Clockwise,
    Counterclockwise,
}

#[derive(Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum ServerMessage {
    Connected {
        address: String,
    },
    Complete {
        action: &'static str,
    },
    Position {
        id: u8,
        position: i16,
    },
    Positions {
        positions: Vec<Position>,
        errors: Vec<String>,
    },
    Error {
        message: String,
        bridge_connected: bool,
    },
}

#[derive(Serialize)]
struct Position {
    id: u8,
    position: i16,
}

#[tokio::main]
async fn main() {
    let log_path = host_log_path();
    initialize_logging(&log_path).unwrap_or_else(|error| panic!("could not start logging: {error}"));
    log::info!("fetchline host starting; log file={}", log_path.display());

    let (listen_address, transport) = startup_options();
    let listener = match TcpListener::bind(&listen_address).await {
        Ok(listener) => listener,
        Err(error) => {
            log::error!("could not bind HTTP listener {listen_address}: {error}");
            panic!("could not bind {listen_address}: {error}");
        }
    };

    log::info!(
        "HTTP control panel listening on http://{listen_address}; MCU transport={transport:?}"
    );

    let config_path = host_config_path();
    let config = load_host_config(&config_path);
    log::info!("host configuration file={}", config_path.display());
    let state = AppState {
        bridge: Arc::new(Mutex::new(None)),
        config: Arc::new(Mutex::new(config)),
        config_path: Arc::new(config_path),
        transport,
    };

    let app = app(state);

    if let Err(error) = axum::serve(
        listener,
        app.into_make_service_with_connect_info::<SocketAddr>(),
    )
    .await
    {
        log::error!("HTTP server stopped unexpectedly: {error}");
    }
}

fn startup_options() -> (String, TransportMode) {
    let mut listen_address = None;
    let mut transport = TransportMode::Controller;
    for argument in env::args().skip(1) {
        if argument == "--debug-raw-tunnel" {
            transport = TransportMode::DebugRawTunnel;
        } else if listen_address.replace(argument).is_some() {
            panic!("usage: fetchline-host [--debug-raw-tunnel] [listen-address]");
        }
    }
    (
        listen_address.unwrap_or_else(|| DEFAULT_LISTEN_ADDRESS.to_owned()),
        transport,
    )
}

fn app(state: AppState) -> Router {
    Router::new()
        .route("/", get(index))
        .route("/app.js", get(javascript))
        .route("/styles.css", get(stylesheet))
        .route("/config", get(get_config).put(put_config))
        .route("/ws", get(websocket))
        .with_state(state)
}

fn host_log_path() -> PathBuf {
    if let Some(path) = env::var_os("XDG_STATE_HOME").filter(|path| !path.is_empty()) {
        return PathBuf::from(path).join("fetchline-host/fetchline-host.log");
    }
    if let Some(home) = env::var_os("HOME").filter(|path| !path.is_empty()) {
        return PathBuf::from(home).join(".local/state/fetchline-host/fetchline-host.log");
    }
    PathBuf::from("fetchline-host.log")
}

fn initialize_logging(path: &Path) -> Result<(), String> {
    let parent = path.parent().ok_or("log path has no parent directory")?;
    fs::create_dir_all(parent).map_err(|error| format!("could not create log directory: {error}"))?;
    let file = open_log_file(path)?;
    LOGGER
        .set(FileLogger {
            file: StdMutex::new(file),
            path: path.to_owned(),
        })
        .map_err(|_| "logger was already initialized".to_owned())?;
    log::set_logger(LOGGER.get().expect("logger was initialized"))
        .map_err(|error| format!("could not register logger: {error}"))?;
    log::set_max_level(log_level_from_environment());
    Ok(())
}

fn log_level_from_environment() -> log::LevelFilter {
    match env::var("FETCHLINE_LOG").as_deref() {
        Ok("debug") => log::LevelFilter::Debug,
        Ok("trace") => log::LevelFilter::Trace,
        Ok("warn") => log::LevelFilter::Warn,
        Ok("error") => log::LevelFilter::Error,
        _ => log::LevelFilter::Info,
    }
}

fn open_log_file(path: &Path) -> Result<fs::File, String> {
    fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(path)
        .map_err(|error| format!("could not open log file {}: {error}", path.display()))
}

fn rotate_log_file(path: &Path) -> Result<fs::File, String> {
    for index in (1..LOG_HISTORY_FILES).rev() {
        let previous = log_archive_path(path, index);
        let next = log_archive_path(path, index + 1);
        if previous.exists() {
            fs::rename(&previous, &next).map_err(|error| {
                format!(
                    "could not rotate log {} to {}: {error}",
                    previous.display(),
                    next.display()
                )
            })?;
        }
    }
    let first_archive = log_archive_path(path, 1);
    fs::rename(path, &first_archive).map_err(|error| {
        format!(
            "could not rotate log {} to {}: {error}",
            path.display(),
            first_archive.display()
        )
    })?;
    open_log_file(path)
}

fn log_archive_path(path: &Path, index: u8) -> PathBuf {
    path.with_extension(format!("log.{index}"))
}

fn host_config_path() -> PathBuf {
    if let Some(path) = env::var_os("XDG_CONFIG_HOME").filter(|path| !path.is_empty()) {
        return PathBuf::from(path).join("fetchline-host/config.json");
    }
    if let Some(home) = env::var_os("HOME").filter(|path| !path.is_empty()) {
        return PathBuf::from(home).join(".config/fetchline-host/config.json");
    }
    PathBuf::from("fetchline-host-config.json")
}

fn load_host_config(path: &Path) -> HostConfig {
    match fs::read_to_string(path) {
        Ok(contents) => match serde_json::from_str::<HostConfig>(&contents) {
            Ok(config) => match validate_host_config(&config) {
                Ok(()) => config,
                Err(error) => {
                    log::warn!("ignoring invalid host configuration {}: {error}", path.display());
                    HostConfig::default()
                }
            },
            Err(error) => {
                log::warn!("ignoring invalid host configuration {}: {error}", path.display());
                HostConfig::default()
            }
        },
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => HostConfig::default(),
        Err(error) => {
            log::error!("could not read host configuration {}: {error}", path.display());
            HostConfig::default()
        }
    }
}

fn validate_host_config(config: &HostConfig) -> Result<(), String> {
    if config.endpoint.host.trim().is_empty() || config.endpoint.port == 0 {
        return Err("MCU host and TCP port are required".to_owned());
    }
    if config.motor.id == 0 || config.motor.id == STS_BROADCAST_ID {
        return Err("motor ID must be between 1 and 253".to_owned());
    }
    if config.motor.speed_percent > 100 {
        return Err("motor speed percentage must be between 0 and 100".to_owned());
    }
    if config.joints.len() != 6 {
        return Err("exactly six position servo configurations are required".to_owned());
    }
    for joint in &config.joints {
        if joint.id == 0 || joint.id == STS_BROADCAST_ID {
            return Err("servo IDs must be between 1 and 253".to_owned());
        }
        if joint.torque_percent > 100 {
            return Err("holding torque percentage must be between 0 and 100".to_owned());
        }
    }
    Ok(())
}

fn persist_host_config(path: &Path, config: &HostConfig) -> Result<(), String> {
    let parent = path.parent().ok_or("host configuration path has no parent directory")?;
    fs::create_dir_all(parent).map_err(|error| format!("could not create config directory: {error}"))?;
    let temporary = path.with_extension("json.tmp");
    let contents = serde_json::to_vec_pretty(config)
        .map_err(|error| format!("could not encode host configuration: {error}"))?;
    fs::write(&temporary, contents)
        .map_err(|error| format!("could not write host configuration: {error}"))?;
    fs::rename(&temporary, path)
        .map_err(|error| format!("could not replace host configuration: {error}"))
}

async fn get_config(State(state): State<AppState>) -> Json<HostConfig> {
    Json(state.config.lock().await.clone())
}

async fn put_config(
    State(state): State<AppState>,
    Json(mut config): Json<HostConfig>,
) -> Result<Json<HostConfig>, (StatusCode, String)> {
    config.endpoint.host = config.endpoint.host.trim().to_owned();
    if let Err(error) = validate_host_config(&config) {
        log::warn!("rejected invalid host configuration update: {error}");
        return Err((StatusCode::BAD_REQUEST, error));
    }
    if let Err(error) = persist_host_config(&state.config_path, &config) {
        log::error!("could not persist host configuration: {error}");
        return Err((StatusCode::INTERNAL_SERVER_ERROR, error));
    }
    *state.config.lock().await = config.clone();
    log::info!("host configuration updated for MCU {}:{}", config.endpoint.host, config.endpoint.port);
    Ok(Json(config))
}

async fn index() -> Response {
    static INDEX: &str = include_str!("../web/index.html");
    asset("text/html; charset=utf-8", INDEX)
}

async fn javascript() -> Response {
    static JAVASCRIPT: &str = include_str!("../web/app.js");
    asset("text/javascript; charset=utf-8", JAVASCRIPT)
}

async fn stylesheet() -> Response {
    static STYLESHEET: &str = include_str!("../web/styles.css");
    asset("text/css; charset=utf-8", STYLESHEET)
}

fn asset(content_type: &'static str, body: &'static str) -> Response {
    (
        [(header::CONTENT_TYPE, HeaderValue::from_static(content_type))],
        body,
    )
        .into_response()
}

async fn websocket(
    websocket: WebSocketUpgrade,
    ConnectInfo(browser_peer): ConnectInfo<SocketAddr>,
    State(state): State<AppState>,
) -> Response {
    websocket.on_upgrade(move |socket| {
        log::info!("browser WebSocket connected peer={browser_peer}");
        client_session(socket, state, browser_peer)
    })
}

async fn client_session(mut socket: WebSocket, state: AppState, browser_peer: SocketAddr) {
    while let Some(message) = socket.recv().await {
        let response = match message {
            Ok(Message::Text(text)) => match serde_json::from_str(&text) {
                Ok(request) => handle_request(&state, request).await,
                Err(error) => {
                    log::warn!("browser sent invalid control request peer={browser_peer}: {error}");
                    ServerMessage::Error {
                        message: format!("Invalid control request: {error}"),
                        bridge_connected: false,
                    }
                }
            },
            Ok(Message::Close(_)) => {
                log::info!("browser WebSocket closed peer={browser_peer}");
                break;
            }
            Ok(Message::Ping(_)) | Ok(Message::Pong(_)) => continue,
            Ok(Message::Binary(_)) => {
                log::warn!("browser sent unsupported binary WebSocket message peer={browser_peer}");
                ServerMessage::Error {
                    message: "Binary WebSocket messages are not supported".to_owned(),
                    bridge_connected: false,
                }
            }
            Err(error) => {
                log::warn!("browser WebSocket connection failed peer={browser_peer}: {error}");
                break;
            }
        };

        let Ok(json) = serde_json::to_string(&response) else {
            log::error!("could not serialize browser response");
            continue;
        };
        if socket.send(Message::Text(json.into())).await.is_err() {
            log::info!("browser WebSocket closed before its response was sent peer={browser_peer}");
            break;
        }
    }
}

async fn handle_request(state: &AppState, request: ClientMessage) -> ServerMessage {
    match request {
        ClientMessage::Connect { host, port } => connect(state, host, port).await,
        request => {
            let request_summary = request.summary();
            let mut bridge = state.bridge.lock().await;
            let Some(connection) = bridge.as_mut() else {
                log::warn!("rejected servobus request without MCU connection: {request_summary}");
                return ServerMessage::Error {
                    message: "Connect to the MCU before sending servo commands".to_owned(),
                    bridge_connected: false,
                };
            };

            log::info!("servobus request peer={} {request_summary}", connection.peer);
            let started = Instant::now();
            let result = match request {
                ClientMessage::StartMotor {
                    id,
                    speed,
                    acceleration,
                    direction,
                } => start_motor(connection, id, speed, acceleration, direction).await,
                ClientMessage::StopMotor { id } => stop_motor(connection, id).await,
                ClientMessage::MovePosition {
                    id,
                    position,
                    acceleration,
                    torque_limit,
                } => move_position(connection, id, position, acceleration, torque_limit).await,
                ClientMessage::ReadPosition { id } => read_position(connection, id)
                    .await
                    .map(|position| ServerMessage::Position { id, position }),
                ClientMessage::ReadPositions { ids } => read_positions(connection, &ids).await,
                ClientMessage::Connect { .. } => unreachable!("connect is handled before locking"),
            };

            match result {
                Ok(response) => {
                    log::info!(
                        "servobus request completed peer={} elapsed_ms={} {request_summary}",
                        connection.peer,
                        started.elapsed().as_millis()
                    );
                    response
                }
                Err(error) => {
                    let bridge_connected = connection_is_usable_after(&error);
                    if bridge_connected {
                        log::warn!(
                            "servobus request failed but MCU TCP connection is retained peer={} elapsed_ms={} request={} error={error}",
                            connection.peer,
                            started.elapsed().as_millis(),
                            request_summary,
                        );
                    } else {
                        log::error!(
                            "MCU TCP connection lost peer={} elapsed_ms={} request={} error={error}",
                            connection.peer,
                            started.elapsed().as_millis(),
                            request_summary,
                        );
                        *bridge = None;
                    }
                    ServerMessage::Error {
                        message: error,
                        bridge_connected,
                    }
                }
            }
        }
    }
}

async fn connect(state: &AppState, host: String, port: u16) -> ServerMessage {
    let host = host.trim();
    if host.is_empty() {
        log::warn!("rejected MCU connection request without a host name");
        return ServerMessage::Error {
            message: "The MCU host name or IP address is required".to_owned(),
            bridge_connected: false,
        };
    }
    if port == 0 {
        log::warn!("rejected MCU connection request with port zero for host={host}");
        return ServerMessage::Error {
            message: "The MCU TCP port must be between 1 and 65535".to_owned(),
            bridge_connected: false,
        };
    }

    let peer = format!("{host}:{port}");
    // The ESP32 bridge accepts one TCP client. Reuse an existing connection to
    // the same MCU instead of attempting a second connection, which the MCU
    // correctly refuses while the first one is active.
    let mut bridge = state.bridge.lock().await;
    if bridge
        .as_ref()
        .is_some_and(|connection| connection.peer == peer)
    {
        log::info!("reusing existing MCU TCP connection peer={peer}");
        return ServerMessage::Connected { address: peer };
    }

    // Switching targets deliberately closes the previous client before opening
    // the new one, so the prior one-client MCU can accept the new connection.
    if let Some(previous) = bridge.take() {
        log::info!("closing MCU TCP connection peer={} before switching to peer={peer}", previous.peer);
    }
    let transport = state.transport;
    log::info!(
        "opening MCU TCP connection peer={peer} transport={transport:?} timeout_ms={}",
        SERVO_TIMEOUT.as_millis()
    );
    let started = Instant::now();
    match timeout(SERVO_TIMEOUT, TcpStream::connect(&peer)).await {
        Ok(Ok(stream)) => {
            if let Err(error) = stream.set_nodelay(true) {
                log::error!("MCU TCP connection setup failed peer={peer}: could not enable TCP_NODELAY: {error}");
                return ServerMessage::Error {
                    message: format!("Connected to {peer}, but could not configure TCP: {error}"),
                    bridge_connected: false,
                };
            }
            let mut connection = BridgeConnection {
                peer: peer.clone(),
                stream,
                transport: TransportMode::Controller,
                next_sequence: 1,
            };
            let setup = match transport {
                TransportMode::Controller => connection
                    .controller_request(Command::Ping)
                    .await
                    .and_then(expect_controller_ack),
                TransportMode::DebugRawTunnel => connection.open_raw_tunnel().await,
            };
            if let Err(error) = setup {
                log::warn!("MCU protocol setup failed peer={peer} transport={transport:?}: {error}");
                return ServerMessage::Error {
                    message: error,
                    bridge_connected: false,
                };
            }
            *bridge = Some(connection);
            log::info!(
                "MCU TCP connection established peer={peer} transport={transport:?} elapsed_ms={}",
                started.elapsed().as_millis()
            );
            ServerMessage::Connected { address: peer }
        }
        Ok(Err(error)) => {
            log::warn!(
                "MCU TCP connection failed peer={peer} elapsed_ms={}: {error}",
                started.elapsed().as_millis()
            );
            ServerMessage::Error {
                message: format!("Could not connect to {peer}: {error}"),
                bridge_connected: false,
            }
        }
        Err(_) => {
            log::warn!(
                "MCU TCP connection timed out peer={peer} timeout_ms={}",
                SERVO_TIMEOUT.as_millis()
            );
            ServerMessage::Error {
                message: format!("Timed out connecting to {peer}"),
                bridge_connected: false,
            }
        }
    }
}

async fn start_motor(
    connection: &mut BridgeConnection,
    id: u8,
    speed: u16,
    acceleration: u8,
    direction: Direction,
) -> Result<ServerMessage, String> {
    validate_id(id)?;
    if speed > 4095 {
        return Err("Motor speed must be between 0 and 4095".to_owned());
    }

    if connection.transport == TransportMode::Controller {
        connection
            .controller_request(Command::StartMotor {
                id,
                counter_clockwise: matches!(direction, Direction::Counterclockwise),
                speed,
                acceleration,
            })
            .await
            .and_then(expect_controller_ack)?;
        return Ok(ServerMessage::Complete {
            action: "motor_started",
        });
    }
    raw_start_motor(connection, id, speed, acceleration, direction).await
}

async fn raw_start_motor(
    connection: &mut BridgeConnection,
    id: u8,
    speed: u16,
    acceleration: u8,
    direction: Direction,
) -> Result<ServerMessage, String> {

    // Mode is non-volatile on STS servos. Re-sending it is deliberate: a power
    // cycle or another tool may have returned this servo to position mode.
    connection.write_register(id, STS_MODE, &[1]).await?;
    connection
        .write_register(id, STS_TORQUE_ENABLE, &[1])
        .await?;

    let signed_speed = match direction {
        Direction::Clockwise => speed,
        Direction::Counterclockwise => speed | 0x8000,
    };
    let mut command = [0_u8; 7];
    command[0] = acceleration;
    command[5..7].copy_from_slice(&signed_speed.to_le_bytes());
    connection
        .write_register(id, STS_ACCELERATION, &command)
        .await?;
    Ok(ServerMessage::Complete {
        action: "motor_started",
    })
}

async fn stop_motor(connection: &mut BridgeConnection, id: u8) -> Result<ServerMessage, String> {
    validate_id(id)?;
    if connection.transport == TransportMode::Controller {
        connection
            .controller_request(Command::StopMotor { id })
            .await
            .and_then(expect_controller_ack)?;
        return Ok(ServerMessage::Complete {
            action: "motor_stopped",
        });
    }
    raw_stop_motor(connection, id).await
}

async fn raw_stop_motor(
    connection: &mut BridgeConnection,
    id: u8,
) -> Result<ServerMessage, String> {
    let command = [0_u8; 7];
    // In continuous mode a goal speed of zero commands a controlled stop while
    // leaving torque enabled.
    connection
        .write_register(id, STS_ACCELERATION, &command)
        .await?;
    Ok(ServerMessage::Complete {
        action: "motor_stopped",
    })
}

async fn move_position(
    connection: &mut BridgeConnection,
    id: u8,
    position: u16,
    acceleration: u8,
    torque_limit: u16,
) -> Result<ServerMessage, String> {
    validate_id(id)?;
    if position > 4095 {
        return Err("Position must be between 0 and 4095".to_owned());
    }
    if torque_limit > 1000 {
        return Err("Torque limit must be between 0 and 1000".to_owned());
    }

    if connection.transport == TransportMode::Controller {
        connection
            .controller_request(Command::SetPosition {
                id,
                position,
                acceleration,
                torque_limit,
            })
            .await
            .and_then(expect_controller_ack)?;
        return Ok(ServerMessage::Complete {
            action: "position_commanded",
        });
    }
    raw_move_position(connection, id, position, acceleration, torque_limit).await
}

async fn raw_move_position(
    connection: &mut BridgeConnection,
    id: u8,
    position: u16,
    acceleration: u8,
    torque_limit: u16,
) -> Result<ServerMessage, String> {

    // Enable holding torque, then set the RAM torque limit. The limit remains
    // in effect after the servo reaches the requested position.
    connection
        .write_register(id, STS_TORQUE_ENABLE, &[1])
        .await?;
    connection
        .write_register(id, STS_TORQUE_LIMIT, &torque_limit.to_le_bytes())
        .await?;

    // This is the STS WritePosEx payload at address 41:
    // acceleration, goal position, goal time (zero), maximum speed (zero).
    // A speed of zero asks the servo to use its configured maximum speed.
    let mut command = [0_u8; 7];
    command[0] = acceleration;
    command[1..3].copy_from_slice(&position.to_le_bytes());
    connection
        .write_register(id, STS_ACCELERATION, &command)
        .await?;
    Ok(ServerMessage::Complete {
        action: "position_commanded",
    })
}

async fn read_position(connection: &mut BridgeConnection, id: u8) -> Result<i16, String> {
    validate_id(id)?;
    if connection.transport == TransportMode::Controller {
        return match connection.controller_request(Command::ReadPosition { id }).await? {
            ControllerResponse::Position {
                id: response_id,
                position,
            } if response_id == id => Ok(position),
            ControllerResponse::Position {
                id: response_id, ..
            } => Err(format!(
                "MCU returned a position for servo {response_id}, but servo {id} was requested"
            )),
            response => Err(format!("MCU returned an unexpected response to position read: {response:?}")),
        };
    }
    raw_read_position(connection, id).await
}

async fn raw_read_position(connection: &mut BridgeConnection, id: u8) -> Result<i16, String> {
    let bytes = connection
        .read_register(id, STS_PRESENT_POSITION, 2)
        .await?;
    let raw = u16::from_le_bytes([bytes[0], bytes[1]]);
    Ok(decode_signed_15(raw))
}

async fn read_positions(
    connection: &mut BridgeConnection,
    ids: &[u8],
) -> Result<ServerMessage, String> {
    if ids.len() > 6 {
        return Err("At most six position servos can be read at once".to_owned());
    }
    let mut positions = Vec::with_capacity(ids.len());
    let mut errors = Vec::new();
    for &id in ids {
        match read_position(connection, id).await {
            Ok(position) => {
                log::debug!("servobus position read peer={} servo={id} position={position}", connection.peer);
                positions.push(Position { id, position });
            }
            Err(error) if connection_is_usable_after(&error) => {
                log::warn!(
                    "servobus position read failed but MCU TCP connection is retained peer={} servo={id} error={error}",
                    connection.peer
                );
                errors.push(format!("Servo {id}: {error}"));
            }
            Err(error) => return Err(error),
        }
    }
    Ok(ServerMessage::Positions { positions, errors })
}

/// Controller-mode responses carry a sequence number, so a delayed response
/// can be discarded safely. Raw debug tunnelling retains the legacy behavior.
/// Broken TCP I/O is the only case that requires reconnecting.
fn connection_is_usable_after(error: &str) -> bool {
    !error.starts_with("Timed out sending command to the MCU")
        && !error.starts_with("Could not send command to the MCU")
        && !error.starts_with("Could not read STS servo reply")
        && !error.starts_with("Could not send a controller command to the MCU")
        && !error.starts_with("Could not read an MCU controller response")
        && !error.starts_with("MCU sent an invalid controller frame")
}

fn validate_id(id: u8) -> Result<(), String> {
    if id == 0 || id == STS_BROADCAST_ID {
        Err("Servo ID must be between 1 and 253".to_owned())
    } else {
        Ok(())
    }
}

impl BridgeConnection {
    async fn controller_request(&mut self, command: Command) -> Result<ControllerResponse, String> {
        if self.transport != TransportMode::Controller {
            return Err("the MCU connection is in debug raw-tunnel mode".to_owned());
        }
        let sequence = self.next_sequence;
        self.next_sequence = self.next_sequence.wrapping_add(1);
        let frame = Frame::command(sequence, command).encode();
        write_controller_frame(&mut self.stream, &frame).await?;

        for _ in 0..MAX_STALE_CONTROLLER_RESPONSES {
            let frame = read_controller_frame(&mut self.stream).await?;
            let response_sequence = frame.sequence();
            if response_sequence != sequence {
                log::warn!(
                    "discarding stale MCU controller response peer={} expected_sequence={sequence} response_sequence={response_sequence}",
                    self.peer
                );
                continue;
            }
            return match frame.as_response() {
                Ok(ControllerResponse::Error { code, detail }) => {
                    Err(controller_error_message(code, detail))
                }
                Ok(response) => Ok(response),
                Err(error) => Err(format!("MCU sent an invalid controller response: {error:?}")),
            };
        }
        Err("MCU sent too many stale controller responses".to_owned())
    }

    async fn open_raw_tunnel(&mut self) -> Result<(), String> {
        match self.controller_request(Command::OpenRawTunnel).await? {
            ControllerResponse::RawTunnelReady => {
                self.transport = TransportMode::DebugRawTunnel;
                log::warn!("MCU debug raw tunnel enabled peer={}", self.peer);
                Ok(())
            }
            response => Err(format!(
                "MCU returned an unexpected response while enabling the debug raw tunnel: {response:?}"
            )),
        }
    }

    async fn write_register(&mut self, id: u8, address: u8, data: &[u8]) -> Result<(), String> {
        let mut parameters = Vec::with_capacity(data.len() + 1);
        parameters.push(address);
        parameters.extend_from_slice(data);
        self.write_packet(id, STS_INSTRUCTION_WRITE, &parameters)
            .await?;
        self.read_status(id, 0).await.map(|_| ())
    }

    async fn read_register(&mut self, id: u8, address: u8, length: u8) -> Result<Vec<u8>, String> {
        self.write_packet(id, STS_INSTRUCTION_READ, &[address, length])
            .await?;
        self.read_status(id, length as usize).await
    }

    async fn write_packet(
        &mut self,
        id: u8,
        instruction: u8,
        parameters: &[u8],
    ) -> Result<(), String> {
        let length = parameters
            .len()
            .checked_add(2)
            .ok_or_else(|| "STS command is too long".to_owned())?;
        let length = u8::try_from(length).map_err(|_| "STS command is too long".to_owned())?;
        let mut packet = Vec::with_capacity(parameters.len() + 6);
        packet.extend_from_slice(&STS_HEADER);
        packet.extend_from_slice(&[id, length, instruction]);
        packet.extend_from_slice(parameters);
        packet.push(checksum(&packet[2..]));
        log::debug!(
            "servobus packet write peer={} servo={id} instruction=0x{instruction:02x} parameter_bytes={}",
            self.peer,
            parameters.len()
        );
        write_with_timeout(&mut self.stream, &packet).await
    }

    async fn read_status(
        &mut self,
        expected_id: u8,
        data_length: usize,
    ) -> Result<Vec<u8>, String> {
        find_header(&mut self.stream).await?;
        let mut fixed = [0_u8; 2];
        read_exact_with_timeout(&mut self.stream, &mut fixed).await?;
        let id = fixed[0];
        let length = fixed[1] as usize;
        if id != expected_id {
            return Err(format!(
                "STS reply came from servo {id}, but servo {expected_id} was addressed"
            ));
        }
        if !(2..=66).contains(&length) {
            return Err(format!("STS reply has an invalid length ({length})"));
        }

        let mut body = vec![0_u8; length];
        read_exact_with_timeout(&mut self.stream, &mut body).await?;
        let checksum_index = body.len() - 1;
        let expected_checksum = checksum_for_status(id, fixed[1], &body[..checksum_index]);
        if body[checksum_index] != expected_checksum {
            return Err("STS reply checksum did not match".to_owned());
        }
        let error = body[0];
        if error != 0 {
            return Err(format!("Servo {id} reported STS error 0x{error:02x}"));
        }
        let payload = &body[1..checksum_index];
        if payload.len() != data_length {
            return Err(format!(
                "Servo {id} returned {} data bytes; expected {data_length}",
                payload.len()
            ));
        }
        Ok(payload.to_vec())
    }
}

fn expect_controller_ack(response: ControllerResponse) -> Result<(), String> {
    match response {
        ControllerResponse::Ack => Ok(()),
        response => Err(format!("MCU returned an unexpected controller response: {response:?}")),
    }
}

fn controller_error_message(code: ErrorCode, detail: u16) -> String {
    match code {
        ErrorCode::UnsupportedCommand => "The MCU does not support this controller command".to_owned(),
        ErrorCode::InvalidServoId => "The MCU rejected the servo ID".to_owned(),
        ErrorCode::InvalidArgument => "The MCU rejected an out-of-range controller argument".to_owned(),
        ErrorCode::ServoTimeout => "The MCU timed out waiting for an STS servo reply".to_owned(),
        ErrorCode::InvalidServoReply => "The MCU received an invalid STS servo reply".to_owned(),
        ErrorCode::ServoReportedError => format!("Servo reported STS error 0x{detail:02x}"),
        ErrorCode::ServoTransport => "The MCU could not communicate with the STS UART".to_owned(),
        ErrorCode::InvalidRequest => "The MCU rejected the controller request".to_owned(),
    }
}

async fn write_controller_frame(stream: &mut TcpStream, bytes: &[u8; FRAME_LEN]) -> Result<(), String> {
    timeout(SERVO_TIMEOUT, stream.write_all(bytes))
        .await
        .map_err(|_| "Timed out sending a controller command to the MCU".to_owned())?
        .map_err(|error| format!("Could not send a controller command to the MCU: {error}"))
}

async fn read_controller_frame(stream: &mut TcpStream) -> Result<Frame, String> {
    let mut bytes = [0_u8; FRAME_LEN];
    timeout(SERVO_TIMEOUT, stream.read_exact(&mut bytes))
        .await
        .map_err(|_| "Timed out waiting for an MCU controller response".to_owned())?
        .map_err(|error| format!("Could not read an MCU controller response: {error}"))?;
    Frame::decode(bytes).map_err(|error| format!("MCU sent an invalid controller frame: {error:?}"))
}

async fn find_header(stream: &mut TcpStream) -> Result<(), String> {
    let mut previous = 0_u8;
    for _ in 0..128 {
        let mut byte = [0_u8; 1];
        read_exact_with_timeout(stream, &mut byte).await?;
        if previous == 0xff && byte[0] == 0xff {
            return Ok(());
        }
        previous = byte[0];
    }
    Err("Could not find an STS response header".to_owned())
}

async fn write_with_timeout(stream: &mut TcpStream, bytes: &[u8]) -> Result<(), String> {
    timeout(SERVO_TIMEOUT, stream.write_all(bytes))
        .await
        .map_err(|_| "Timed out sending command to the MCU".to_owned())?
        .map_err(|error| format!("Could not send command to the MCU: {error}"))
}

async fn read_exact_with_timeout(stream: &mut TcpStream, bytes: &mut [u8]) -> Result<(), String> {
    timeout(SERVO_TIMEOUT, stream.read_exact(bytes))
        .await
        .map_err(|_| "Timed out waiting for an STS servo reply".to_owned())?
        .map_err(|error| format!("Could not read STS servo reply: {error}"))
        .map(|_| ())
}

fn checksum(bytes: &[u8]) -> u8 {
    !bytes.iter().fold(0_u8, |sum, byte| sum.wrapping_add(*byte))
}

fn checksum_for_status(id: u8, length: u8, body_without_checksum: &[u8]) -> u8 {
    let mut sum = id.wrapping_add(length);
    for byte in body_without_checksum {
        sum = sum.wrapping_add(*byte);
    }
    !sum
}

fn decode_signed_15(value: u16) -> i16 {
    if value & 0x8000 != 0 {
        -((value & 0x7fff) as i16)
    } else {
        value as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axum::{
        body::{Body, to_bytes},
        http::{Request, StatusCode},
    };
    use tempfile::tempdir;
    use tower::ServiceExt;

    #[test]
    fn creates_sts_checksums() {
        // ID 1, length 4, write, address 40, value 1.
        assert_eq!(checksum(&[1, 4, STS_INSTRUCTION_WRITE, 40, 1]), 206);
    }

    #[test]
    fn decodes_sts_signed_values() {
        assert_eq!(decode_signed_15(2048), 2048);
        assert_eq!(decode_signed_15(0x8001), -1);
        assert_eq!(decode_signed_15(0xffff), -32767);
    }

    #[test]
    fn accepts_only_unicast_servo_ids() {
        assert!(validate_id(1).is_ok());
        assert!(validate_id(253).is_ok());
        assert!(validate_id(0).is_err());
        assert!(validate_id(254).is_err());
    }

    #[test]
    fn servo_timeouts_do_not_require_reconnecting_the_bridge() {
        assert!(connection_is_usable_after(
            "Timed out waiting for an STS servo reply"
        ));
        assert!(connection_is_usable_after(
            "STS reply checksum did not match"
        ));
        assert!(!connection_is_usable_after(
            "Could not send command to the MCU: Connection reset by peer"
        ));
    }

    #[test]
    fn checks_status_packets() {
        // Status response: id 7, length 4, no error, low/high position.
        assert_eq!(checksum_for_status(7, 4, &[0, 0x34, 0x12]), 174);
    }

    #[test]
    fn persists_and_loads_host_configuration() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("config.json");
        let mut config = HostConfig::default();
        config.endpoint.host = "192.168.178.146".to_owned();
        config.joints[1].enabled = false;

        persist_host_config(&path, &config).unwrap();
        let loaded = load_host_config(&path);
        assert_eq!(loaded.endpoint.host, "192.168.178.146");
        assert!(!loaded.joints[1].enabled);
    }

    #[test]
    fn invalid_saved_configuration_falls_back_to_defaults() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("config.json");
        fs::write(&path, "not JSON").unwrap();

        let loaded = load_host_config(&path);
        assert_eq!(loaded.endpoint.host, HostConfig::default().endpoint.host);
        assert_eq!(loaded.joints.len(), 6);
    }

    #[test]
    fn rotates_host_logs_without_losing_recent_archives() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("fetchline-host.log");
        fs::write(&path, "current").unwrap();
        fs::write(log_archive_path(&path, 1), "previous").unwrap();
        fs::write(log_archive_path(&path, 2), "older").unwrap();

        let mut replacement = rotate_log_file(&path).unwrap();
        replacement.write_all(b"new").unwrap();
        replacement.flush().unwrap();

        assert_eq!(fs::read_to_string(&path).unwrap(), "new");
        assert_eq!(fs::read_to_string(log_archive_path(&path, 1)).unwrap(), "current");
        assert_eq!(fs::read_to_string(log_archive_path(&path, 2)).unwrap(), "previous");
        assert_eq!(fs::read_to_string(log_archive_path(&path, 3)).unwrap(), "older");
    }

    #[tokio::test]
    async fn configuration_api_persists_shared_settings() {
        let directory = tempdir().unwrap();
        let path = directory.path().join("config.json");
        let state = test_state(path.clone());
        let mut config = HostConfig::default();
        config.endpoint.host = "192.168.178.146".to_owned();
        config.motor.enabled = false;

        let request = Request::builder()
            .method("PUT")
            .uri("/config")
            .header("content-type", "application/json")
            .body(Body::from(serde_json::to_vec(&config).unwrap()))
            .unwrap();
        let response = app(state.clone()).oneshot(request).await.unwrap();
        assert_eq!(response.status(), StatusCode::OK);

        let request = Request::builder().uri("/config").body(Body::empty()).unwrap();
        let response = app(state).oneshot(request).await.unwrap();
        assert_eq!(response.status(), StatusCode::OK);
        let body = to_bytes(response.into_body(), usize::MAX).await.unwrap();
        let loaded: HostConfig = serde_json::from_slice(&body).unwrap();
        assert_eq!(loaded.endpoint.host, "192.168.178.146");
        assert!(!loaded.motor.enabled);
        assert_eq!(load_host_config(&path).endpoint.host, "192.168.178.146");
    }

    #[tokio::test]
    async fn configuration_api_rejects_invalid_servo_settings() {
        let directory = tempdir().unwrap();
        let state = test_state(directory.path().join("config.json"));
        let mut config = HostConfig::default();
        config.joints.clear();
        let request = Request::builder()
            .method("PUT")
            .uri("/config")
            .header("content-type", "application/json")
            .body(Body::from(serde_json::to_vec(&config).unwrap()))
            .unwrap();

        let response = app(state).oneshot(request).await.unwrap();
        assert_eq!(response.status(), StatusCode::BAD_REQUEST);
    }

    #[tokio::test]
    async fn motor_start_selects_continuous_mode_then_writes_speed() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 1, 4, 3, 33, 1, 213]
            );
            stream.write_all(&status_packet(1, &[])).await.unwrap();
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 1, 4, 3, 40, 1, 206]
            );
            stream.write_all(&status_packet(1, &[])).await.unwrap();
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 1, 10, 3, 41, 20, 0, 0, 0, 0, 0, 4, 176]
            );
            stream.write_all(&status_packet(1, &[])).await.unwrap();
        });

        let mut bridge = raw_bridge(address).await;
        start_motor(&mut bridge, 1, 1024, 20, Direction::Clockwise)
            .await
            .unwrap();
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn controller_mode_uses_a_high_level_command_and_discards_a_stale_response() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let request = receive_controller_frame(&mut stream).await;
            assert_eq!(
                request.as_command(),
                Ok(Command::SetPosition {
                    id: 5,
                    position: 1625,
                    acceleration: 20,
                    torque_limit: 1000,
                })
            );
            stream
                .write_all(&Frame::response(0, ControllerResponse::Ack).encode())
                .await
                .unwrap();
            stream
                .write_all(&Frame::response(request.sequence(), ControllerResponse::Ack).encode())
                .await
                .unwrap();
        });

        let mut bridge = controller_bridge(address).await;
        move_position(&mut bridge, 5, 1625, 20, 1000)
            .await
            .unwrap();
        mcu.await.unwrap();
    }

    #[tokio::test]
    async fn debug_raw_tunnel_is_enabled_by_an_explicit_controller_command() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let request = receive_controller_frame(&mut stream).await;
            assert_eq!(request.as_command(), Ok(Command::OpenRawTunnel));
            stream
                .write_all(
                    &Frame::response(request.sequence(), ControllerResponse::RawTunnelReady)
                        .encode(),
                )
                .await
                .unwrap();
        });

        let mut bridge = controller_bridge(address).await;
        bridge.open_raw_tunnel().await.unwrap();
        assert_eq!(bridge.transport, TransportMode::DebugRawTunnel);
        mcu.await.unwrap();
    }

    #[tokio::test]
    async fn position_read_decodes_the_servo_response() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 2, 4, 2, 56, 2, 189]
            );
            stream
                .write_all(&status_packet(2, &[0x34, 0x12]))
                .await
                .unwrap();
        });

        let mut bridge = raw_bridge(address).await;
        assert_eq!(read_position(&mut bridge, 2).await.unwrap(), 0x1234);
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn position_read_rejects_a_reply_from_another_servo() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            receive_packet(&mut stream).await;
            stream.write_all(&status_packet(3, &[0, 0])).await.unwrap();
        });

        let mut bridge = bridge_for(address).await;
        let error = read_position(&mut bridge, 2).await.unwrap_err();
        assert!(error.contains("came from servo 3"));
        assert!(connection_is_usable_after(&error));
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn position_read_rejects_a_bad_checksum_without_dropping_tcp() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            receive_packet(&mut stream).await;
            let mut packet = status_packet(2, &[0, 0]);
            *packet.last_mut().unwrap() ^= 0xff;
            stream.write_all(&packet).await.unwrap();
        });

        let mut bridge = bridge_for(address).await;
        let error = read_position(&mut bridge, 2).await.unwrap_err();
        assert_eq!(error, "STS reply checksum did not match");
        assert!(connection_is_usable_after(&error));
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn position_read_reports_a_servo_status_error() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            receive_packet(&mut stream).await;
            stream
                .write_all(&status_packet_with_error(2, 0x20, &[0, 0]))
                .await
                .unwrap();
        });

        let mut bridge = bridge_for(address).await;
        let error = read_position(&mut bridge, 2).await.unwrap_err();
        assert_eq!(error, "Servo 2 reported STS error 0x20");
        assert!(connection_is_usable_after(&error));
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn position_read_drops_tcp_after_a_truncated_reply() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            receive_packet(&mut stream).await;
            stream.write_all(&[0xff, 0xff, 2, 4, 0]).await.unwrap();
        });

        let mut bridge = bridge_for(address).await;
        let error = read_position(&mut bridge, 2).await.unwrap_err();
        assert!(error.starts_with("Could not read STS servo reply"));
        assert!(!connection_is_usable_after(&error));
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn position_refresh_continues_after_one_servo_times_out() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let servo = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 2, 4, 2, 56, 2, 189]
            );
            stream.write_all(&status_packet(2, &[2, 0])).await.unwrap();

            // Intentionally do not answer servo 3. The next packet proves the
            // host retained the TCP bridge and continued with servo 4.
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 3, 4, 2, 56, 2, 188]
            );
            assert_eq!(
                receive_packet(&mut stream).await,
                vec![0xff, 0xff, 4, 4, 2, 56, 2, 187]
            );
            stream.write_all(&status_packet(4, &[4, 0])).await.unwrap();
        });

        let mut bridge = raw_bridge(address).await;
        let ServerMessage::Positions { positions, errors } =
            read_positions(&mut bridge, &[2, 3, 4]).await.unwrap()
        else {
            panic!("position refresh should return a positions response");
        };
        assert_eq!(positions.len(), 2);
        assert_eq!(positions[0].id, 2);
        assert_eq!(positions[1].id, 4);
        assert_eq!(errors.len(), 1);
        assert!(errors[0].starts_with("Servo 3: Timed out"));
        servo.await.unwrap();
    }

    #[tokio::test]
    async fn reconnecting_to_the_same_mcu_reuses_its_single_tcp_client() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let state = AppState::default();
        let controller = tokio::spawn(async move {
            let (mut stream, _) = listener.accept().await.unwrap();
            let frame = receive_controller_frame(&mut stream).await;
            assert_eq!(frame.as_command(), Ok(Command::Ping));
            stream
                .write_all(&Frame::response(frame.sequence(), ControllerResponse::Ack).encode())
                .await
                .unwrap();
        });

        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        controller.await.unwrap();

        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
    }

    #[tokio::test]
    async fn connection_refusal_is_reported_without_a_bridge() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        drop(listener);

        let response = connect(&AppState::default(), "127.0.0.1".to_owned(), port).await;
        let ServerMessage::Error {
            message,
            bridge_connected,
        } = response
        else {
            panic!("a closed local port must refuse the connection");
        };
        assert!(message.starts_with("Could not connect"));
        assert!(!bridge_connected);
    }

    fn test_state(config_path: PathBuf) -> AppState {
        AppState {
            bridge: Arc::new(Mutex::new(None)),
            config: Arc::new(Mutex::new(HostConfig::default())),
            config_path: Arc::new(config_path),
            transport: TransportMode::Controller,
        }
    }

    async fn bridge_for(address: SocketAddr) -> BridgeConnection {
        raw_bridge(address).await
    }

    async fn raw_bridge(address: SocketAddr) -> BridgeConnection {
        BridgeConnection {
            peer: address.to_string(),
            stream: TcpStream::connect(address).await.unwrap(),
            transport: TransportMode::DebugRawTunnel,
            next_sequence: 1,
        }
    }

    async fn controller_bridge(address: SocketAddr) -> BridgeConnection {
        BridgeConnection {
            peer: address.to_string(),
            stream: TcpStream::connect(address).await.unwrap(),
            transport: TransportMode::Controller,
            next_sequence: 1,
        }
    }

    async fn receive_packet(stream: &mut TcpStream) -> Vec<u8> {
        let mut header_and_fixed = [0_u8; 4];
        stream.read_exact(&mut header_and_fixed).await.unwrap();
        assert_eq!(&header_and_fixed[..2], STS_HEADER);
        let length = header_and_fixed[3] as usize;
        let mut remaining = vec![0_u8; length];
        stream.read_exact(&mut remaining).await.unwrap();
        [header_and_fixed.to_vec(), remaining].concat()
    }

    async fn receive_controller_frame(stream: &mut TcpStream) -> Frame {
        let mut bytes = [0_u8; FRAME_LEN];
        stream.read_exact(&mut bytes).await.unwrap();
        Frame::decode(bytes).unwrap()
    }

    fn status_packet(id: u8, payload: &[u8]) -> Vec<u8> {
        status_packet_with_error(id, 0, payload)
    }

    fn status_packet_with_error(id: u8, error: u8, payload: &[u8]) -> Vec<u8> {
        let length = u8::try_from(payload.len() + 2).unwrap();
        let mut packet = vec![0xff, 0xff, id, length, error];
        packet.extend_from_slice(payload);
        packet.push(checksum_for_status(id, length, &packet[4..]));
        packet
    }
}
