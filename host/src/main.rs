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
        ws::{Message as BrowserMessage, WebSocket},
    },
    http::{HeaderValue, StatusCode, header},
    response::{IntoResponse, Response},
    routing::get,
    Json,
};
use fetchline_protocol::{CONTROLLER_WEBSOCKET_PATH, RAW_TUNNEL_TCP_PORT};
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    sync::Mutex,
    time::{sleep, timeout},
};
use tokio_tungstenite::{WebSocketStream, client_async, tungstenite::Message as ControllerMessage};

const DEFAULT_LISTEN_ADDRESS: &str = "0.0.0.0:8787";
const SERVO_TIMEOUT: Duration = Duration::from_millis(750);
// A newly accepted controller connection makes the MCU abort the prior
// session. Allow a short retry window while its TCP reset is flushed and the
// released transport socket returns to listening mode.
const CONTROLLER_CONNECT_RETRY_WINDOW: Duration = Duration::from_secs(3);
const CONTROLLER_CONNECT_RETRY_INTERVAL: Duration = Duration::from_millis(100);
// Each unused STS address gets a 50 ms deadline on the MCU. A 1–253 scan also
// includes UART recovery time, so it needs a longer controller response wait.
const SERVO_SCAN_TIMEOUT: Duration = Duration::from_secs(20);
const MAX_STALE_CONTROLLER_RESPONSES: usize = 32;
const STS_HEADER: [u8; 2] = [0xff, 0xff];
const STS_BROADCAST_ID: u8 = 0xfe;
const MAX_SERVO_ID: u8 = STS_BROADCAST_ID - 1;
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
    transport: BridgeTransport,
    next_request_id: u32,
}

enum BridgeTransport {
    Controller(Box<WebSocketStream<TcpStream>>),
    DebugRaw(TcpStream),
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
    ScanServos {
        start_id: u8,
        end_id: u8,
    },
    SetServoId {
        current_id: u8,
        new_id: u8,
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
            Self::ScanServos { start_id, end_id } => {
                format!("scan servobus start_id={start_id} end_id={end_id}")
            }
            Self::SetServoId { current_id, new_id } => {
                format!("change servo ID current_id={current_id} new_id={new_id}")
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Deserialize)]
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
    ServoScan {
        start_id: u8,
        end_id: u8,
        ids: Vec<u8>,
    },
    ServoIdChanged {
        previous_id: u8,
        new_id: u8,
    },
    Error {
        message: String,
        bridge_connected: bool,
    },
}

#[derive(Deserialize, Serialize)]
struct Position {
    id: u8,
    position: i16,
}

#[derive(Deserialize)]
struct ServoScanResult {
    ids: Vec<u8>,
}

#[derive(Deserialize)]
struct ServoIdChangeResult {
    #[serde(rename = "previousId")]
    previous_id: u8,
    #[serde(rename = "newId")]
    new_id: u8,
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
    if !is_unicast_servo_id(config.motor.id) {
        return Err("motor ID must be between 1 and 253".to_owned());
    }
    if config.motor.speed_percent > 100 {
        return Err("motor speed percentage must be between 0 and 100".to_owned());
    }
    if config.joints.len() != 6 {
        return Err("exactly six position servo configurations are required".to_owned());
    }
    for joint in &config.joints {
        if !is_unicast_servo_id(joint.id) {
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
            Ok(BrowserMessage::Text(text)) => match serde_json::from_str(&text) {
                Ok(request) => handle_request(&state, request).await,
                Err(error) => {
                    log::warn!("browser sent invalid control request peer={browser_peer}: {error}");
                    ServerMessage::Error {
                        message: format!("Invalid control request: {error}"),
                        bridge_connected: false,
                    }
                }
            },
            Ok(BrowserMessage::Close(_)) => {
                log::info!("browser WebSocket closed peer={browser_peer}");
                break;
            }
            Ok(BrowserMessage::Ping(_)) | Ok(BrowserMessage::Pong(_)) => continue,
            Ok(BrowserMessage::Binary(_)) => {
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
        if socket.send(BrowserMessage::Text(json.into())).await.is_err() {
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
                ClientMessage::ScanServos { start_id, end_id } => {
                    scan_servos(connection, start_id, end_id).await
                }
                ClientMessage::SetServoId { current_id, new_id } => {
                    set_servo_id(connection, current_id, new_id).await
                }
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
    // Reuse a live WebSocket session to the selected MCU. A TCP peer can vanish
    // without the host immediately learning about it, so verify a reused
    // controller session before reporting it as connected.
    let mut bridge = state.bridge.lock().await;
    if bridge
        .as_ref()
        .is_some_and(|connection| connection.peer == peer)
    {
        let reuse_result = match bridge.as_mut().expect("checked for an existing bridge") {
            connection if connection.is_controller() => connection
                .controller_request("system.ping", serde_json::json!({}))
                .await
                .and_then(expect_controller_ready),
            // Raw debug mode is intentionally a transparent TCP stream, so it
            // has no controller RPC health check to send here.
            _ => Ok(()),
        };
        match reuse_result {
            Ok(()) => {
                log::info!("reusing healthy MCU TCP connection peer={peer}");
                return ServerMessage::Connected { address: peer };
            }
            Err(error) => {
                log::warn!("discarding unhealthy MCU TCP connection peer={peer}: {error}");
            }
        }
    }

    // Switching targets, or replacing an unhealthy session, deliberately
    // closes the previous controller connection before opening a new one.
    if let Some(previous) = bridge.take() {
        log::info!("closing MCU TCP connection peer={} before switching to peer={peer}", previous.peer);
    }
    let transport = state.transport;
    log::info!(
        "opening MCU TCP connection peer={peer} transport={transport:?} timeout_ms={}",
        SERVO_TIMEOUT.as_millis()
    );
    let started = Instant::now();
    match connect_controller(&peer).await {
        Ok(websocket) => {
            let mut connection = BridgeConnection {
                peer: peer.clone(),
                transport: BridgeTransport::Controller(Box::new(websocket)),
                next_request_id: 1,
            };
            let setup = match transport {
                TransportMode::Controller => connection
                    .controller_request("system.ping", serde_json::json!({}))
                    .await
                    .and_then(expect_controller_ready),
                TransportMode::DebugRawTunnel => connection.open_raw_tunnel(host).await,
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
                "MCU JSON-RPC connection established peer={peer} transport={transport:?} elapsed_ms={}",
                started.elapsed().as_millis()
            );
            ServerMessage::Connected { address: peer }
        }
        Err(error) => {
            log::warn!(
                "MCU JSON-RPC connection failed peer={peer} elapsed_ms={}: {error}",
                started.elapsed().as_millis()
            );
            ServerMessage::Error {
                message: error,
                bridge_connected: false,
            }
        }
    }
}

async fn connect_controller(peer: &str) -> Result<WebSocketStream<TcpStream>, String> {
    let started = Instant::now();
    loop {
        match connect_controller_once(peer).await {
            Ok(websocket) => return Ok(websocket),
            Err(error)
                if controller_connection_is_retryable(&error)
                    && started.elapsed() < CONTROLLER_CONNECT_RETRY_WINDOW =>
            {
                log::debug!(
                    "retrying controller connection peer={peer} elapsed_ms={} error={error}",
                    started.elapsed().as_millis()
                );
                sleep(CONTROLLER_CONNECT_RETRY_INTERVAL).await;
            }
            Err(error) => return Err(error),
        }
    }
}

async fn connect_controller_once(peer: &str) -> Result<WebSocketStream<TcpStream>, String> {
    let stream = timeout(SERVO_TIMEOUT, TcpStream::connect(peer))
        .await
        .map_err(|_| format!("Timed out connecting to {peer}"))?
        .map_err(|error| format!("Could not connect to {peer}: {error}"))?;
    stream
        .set_nodelay(true)
        .map_err(|error| format!("Connected to {peer}, but could not configure TCP: {error}"))?;
    let endpoint = format!("ws://{peer}{CONTROLLER_WEBSOCKET_PATH}");
    timeout(SERVO_TIMEOUT, client_async(endpoint, stream))
        .await
        .map_err(|_| format!("Timed out opening the JSON-RPC WebSocket to {peer}"))?
        .map(|(websocket, _)| websocket)
        .map_err(|error| format!("Could not open the JSON-RPC WebSocket to {peer}: {error}"))
}

fn controller_connection_is_retryable(error: &str) -> bool {
    error.starts_with("Timed out connecting")
        || error.starts_with("Could not connect")
        || error.starts_with("Timed out opening the JSON-RPC WebSocket")
        || error.starts_with("Could not open the JSON-RPC WebSocket")
}

async fn connect_raw_tunnel(peer: &str) -> Result<TcpStream, String> {
    // The MCU starts a separate async listener after it acknowledges
    // debug.enableRawTunnel. Retrying briefly avoids a TCP race between that
    // response reaching the host and the listener's first poll.
    let started = Instant::now();
    let stream = loop {
        let elapsed = started.elapsed();
        let Some(remaining) = SERVO_TIMEOUT.checked_sub(elapsed) else {
            return Err(format!("Timed out connecting to raw tunnel {peer}"));
        };
        let attempt_timeout = remaining.min(Duration::from_millis(100));
        match timeout(attempt_timeout, TcpStream::connect(peer)).await {
            Ok(Ok(stream)) => break stream,
            Ok(Err(error)) => {
                log::debug!("raw tunnel {peer} is not ready yet: {error}");
            }
            Err(_) => {
                log::debug!("raw tunnel {peer} connection attempt timed out");
            }
        }
        sleep(Duration::from_millis(25)).await;
    };
    stream
        .set_nodelay(true)
        .map_err(|error| format!("Connected to raw tunnel {peer}, but could not configure TCP: {error}"))?;
    Ok(stream)
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

    if connection.is_controller() {
        connection
            .controller_request(
                "motor.start",
                serde_json::json!({
                    "id": id,
                    "speed": speed,
                    "acceleration": acceleration,
                    "direction": match direction {
                        Direction::Clockwise => "clockwise",
                        Direction::Counterclockwise => "counterclockwise",
                    },
                }),
            )
            .await
            .and_then(expect_controller_accepted)?;
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
    if connection.is_controller() {
        connection
            .controller_request("motor.stop", serde_json::json!({ "id": id }))
            .await
            .and_then(expect_controller_accepted)?;
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

    if connection.is_controller() {
        connection
            .controller_request(
                "servo.setPosition",
                serde_json::json!({
                    "id": id,
                    "position": position,
                    "acceleration": acceleration,
                    "torqueLimit": torque_limit,
                }),
            )
            .await
            .and_then(expect_controller_accepted)?;
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
    if connection.is_controller() {
        let response = connection
            .controller_request("servo.getPosition", serde_json::json!({ "id": id }))
            .await?;
        let position: Position = serde_json::from_value(response)
            .map_err(|error| format!("MCU returned an invalid position result: {error}"))?;
        if position.id != id {
            return Err(format!(
                "MCU returned a position for servo {}, but servo {id} was requested",
                position.id
            ));
        }
        return Ok(position.position);
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

async fn scan_servos(
    connection: &mut BridgeConnection,
    start_id: u8,
    end_id: u8,
) -> Result<ServerMessage, String> {
    validate_scan_range(start_id, end_id)?;
    if !connection.is_controller() {
        return Err("Servo search is available only in JSON-RPC controller mode, not through the raw debug tunnel".to_owned());
    }

    let response = connection
        .controller_request_with_timeout(
            "servo.scan",
            serde_json::json!({ "startId": start_id, "endId": end_id }),
            SERVO_SCAN_TIMEOUT,
        )
        .await?;
    let result: ServoScanResult = serde_json::from_value(response)
        .map_err(|error| format!("MCU returned an invalid servo-scan result: {error}"))?;
    for &id in &result.ids {
        if id < start_id || id > end_id || !is_unicast_servo_id(id) {
            return Err(format!("MCU returned an invalid scanned servo ID {id}"));
        }
    }
    Ok(ServerMessage::ServoScan {
        start_id,
        end_id,
        ids: result.ids,
    })
}

async fn set_servo_id(
    connection: &mut BridgeConnection,
    current_id: u8,
    new_id: u8,
) -> Result<ServerMessage, String> {
    validate_id_change(current_id, new_id)?;
    if !connection.is_controller() {
        return Err("Changing a servo ID is available only in JSON-RPC controller mode, not through the raw debug tunnel".to_owned());
    }

    let response = connection
        .controller_request(
            "servo.setId",
            serde_json::json!({ "currentId": current_id, "newId": new_id }),
        )
        .await?;
    let result: ServoIdChangeResult = serde_json::from_value(response)
        .map_err(|error| format!("MCU returned an invalid servo-ID change result: {error}"))?;
    if result.previous_id != current_id || result.new_id != new_id {
        return Err("MCU confirmed different servo IDs than requested".to_owned());
    }
    Ok(ServerMessage::ServoIdChanged {
        previous_id: result.previous_id,
        new_id: result.new_id,
    })
}

/// Controller-mode responses carry a sequence number, so a delayed response
/// can be discarded safely. Raw debug tunnelling retains the legacy behavior.
/// Broken TCP I/O is the only case that requires reconnecting.
fn connection_is_usable_after(error: &str) -> bool {
    const BROKEN_CONNECTION_PREFIXES: [&str; 13] = [
        "Timed out sending command to the MCU",
        "Could not send command to the MCU",
        "Could not read STS servo reply",
        "Timed out sending a JSON-RPC command to the MCU",
        "Could not send a JSON-RPC command to the MCU",
        "Timed out waiting for an MCU JSON-RPC response",
        "MCU closed the JSON-RPC WebSocket",
        "Could not read an MCU JSON-RPC response",
        "MCU sent invalid JSON-RPC",
        "MCU sent a JSON-RPC response with an unsupported version",
        "Could not answer MCU WebSocket ping",
        "MCU sent an unsupported WebSocket message",
        "MCU sent too many stale JSON-RPC responses",
    ];
    !BROKEN_CONNECTION_PREFIXES
        .iter()
        .any(|prefix| error.starts_with(prefix))
}

fn validate_id(id: u8) -> Result<(), String> {
    if !is_unicast_servo_id(id) {
        Err("Servo ID must be between 1 and 253".to_owned())
    } else {
        Ok(())
    }
}

fn is_unicast_servo_id(id: u8) -> bool {
    id != 0 && id <= MAX_SERVO_ID
}

fn validate_scan_range(start_id: u8, end_id: u8) -> Result<(), String> {
    if start_id == 0 || start_id > end_id || end_id > MAX_SERVO_ID {
        Err("Servo search IDs must be between 1 and 253, with a start no greater than the end".to_owned())
    } else {
        Ok(())
    }
}

fn validate_id_change(current_id: u8, new_id: u8) -> Result<(), String> {
    validate_id(current_id)?;
    validate_id(new_id)?;
    if current_id == new_id {
        Err("The current and new servo IDs must differ".to_owned())
    } else {
        Ok(())
    }
}

impl BridgeConnection {
    fn is_controller(&self) -> bool {
        matches!(self.transport, BridgeTransport::Controller(_))
    }

    fn raw_stream_mut(&mut self) -> Result<&mut TcpStream, String> {
        match &mut self.transport {
            BridgeTransport::DebugRaw(stream) => Ok(stream),
            BridgeTransport::Controller(_) => {
                Err("the MCU connection is in JSON-RPC controller mode".to_owned())
            }
        }
    }

    async fn controller_request(&mut self, method: &str, params: Value) -> Result<Value, String> {
        self.controller_request_with_timeout(method, params, SERVO_TIMEOUT)
            .await
    }

    async fn controller_request_with_timeout(
        &mut self,
        method: &str,
        params: Value,
        response_timeout: Duration,
    ) -> Result<Value, String> {
        let request_id = self.next_request_id;
        self.next_request_id = self.next_request_id.wrapping_add(1);
        let request = serde_json::to_string(&serde_json::json!({
            "jsonrpc": "2.0",
            "id": request_id,
            "method": method,
            "params": params,
        }))
        .map_err(|error| format!("Could not encode JSON-RPC request: {error}"))?;
        let websocket = match &mut self.transport {
            BridgeTransport::Controller(websocket) => websocket,
            BridgeTransport::DebugRaw(_) => {
                return Err("the MCU connection is in debug raw-tunnel mode".to_owned());
            }
        };
        timeout(
            SERVO_TIMEOUT,
            websocket.send(ControllerMessage::Text(request.into())),
        )
        .await
        .map_err(|_| "Timed out sending a JSON-RPC command to the MCU".to_owned())?
        .map_err(|error| format!("Could not send a JSON-RPC command to the MCU: {error}"))?;

        for _ in 0..MAX_STALE_CONTROLLER_RESPONSES {
            let message = timeout(response_timeout, websocket.next())
                .await
                .map_err(|_| "Timed out waiting for an MCU JSON-RPC response".to_owned())?
                .ok_or_else(|| "MCU closed the JSON-RPC WebSocket".to_owned())?
                .map_err(|error| format!("Could not read an MCU JSON-RPC response: {error}"))?;
            match message {
                ControllerMessage::Text(text) => {
                    let response: ControllerJsonRpcResponse = serde_json::from_str(&text)
                        .map_err(|error| format!("MCU sent invalid JSON-RPC: {error}"))?;
                    if response.jsonrpc != "2.0" {
                        return Err("MCU sent a JSON-RPC response with an unsupported version".to_owned());
                    }
                    if response.id != Some(request_id) {
                        log::warn!(
                            "discarding stale MCU JSON-RPC response peer={} expected_id={request_id} response_id={:?}",
                            self.peer,
                            response.id
                        );
                        continue;
                    }
                    return match (response.result, response.error) {
                        (Some(result), None) => Ok(result),
                        (None, Some(error)) => Err(format!(
                            "MCU JSON-RPC error {}: {}",
                            error.code, error.message
                        )),
                        _ => Err("MCU returned an invalid JSON-RPC result/error combination".to_owned()),
                    };
                }
                ControllerMessage::Ping(payload) => {
                    websocket
                        .send(ControllerMessage::Pong(payload))
                        .await
                        .map_err(|error| format!("Could not answer MCU WebSocket ping: {error}"))?;
                }
                ControllerMessage::Pong(_) => continue,
                ControllerMessage::Close(_) => {
                    return Err("MCU closed the JSON-RPC WebSocket".to_owned());
                }
                ControllerMessage::Binary(_) | ControllerMessage::Frame(_) => {
                    return Err("MCU sent an unsupported WebSocket message".to_owned());
                }
            }
        }
        Err("MCU sent too many stale JSON-RPC responses".to_owned())
    }

    async fn open_raw_tunnel(&mut self, host: &str) -> Result<(), String> {
        let result = self
            .controller_request("debug.enableRawTunnel", serde_json::json!({}))
            .await?;
        let port = result
            .get("port")
            .and_then(Value::as_u64)
            .and_then(|port| u16::try_from(port).ok())
            .ok_or_else(|| "MCU did not return a raw tunnel TCP port".to_owned())?;
        if port != RAW_TUNNEL_TCP_PORT {
            return Err(format!("MCU returned unexpected raw tunnel TCP port {port}"));
        }
        let raw_peer = format!("{host}:{port}");
        let stream = connect_raw_tunnel(&raw_peer).await?;
        self.transport = BridgeTransport::DebugRaw(stream);
        log::warn!(
            "MCU debug raw tunnel enabled controller_peer={} raw_peer={raw_peer}; it remains enabled until debug.disableRawTunnel",
            self.peer
        );
        Ok(())
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
        write_with_timeout(self.raw_stream_mut()?, &packet).await
    }

    async fn read_status(
        &mut self,
        expected_id: u8,
        data_length: usize,
    ) -> Result<Vec<u8>, String> {
        let stream = self.raw_stream_mut()?;
        find_header(stream).await?;
        let mut fixed = [0_u8; 2];
        read_exact_with_timeout(stream, &mut fixed).await?;
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
        read_exact_with_timeout(stream, &mut body).await?;
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

#[derive(Deserialize)]
struct ControllerJsonRpcResponse {
    jsonrpc: String,
    id: Option<u32>,
    #[serde(default)]
    result: Option<Value>,
    #[serde(default)]
    error: Option<ControllerJsonRpcError>,
}

#[derive(Deserialize)]
struct ControllerJsonRpcError {
    code: i32,
    message: String,
}

fn expect_controller_ready(result: Value) -> Result<(), String> {
    if result.get("ready").and_then(Value::as_bool) == Some(true) {
        Ok(())
    } else {
        Err("MCU returned an unexpected result to system.ping".to_owned())
    }
}

fn expect_controller_accepted(result: Value) -> Result<(), String> {
    if result.get("accepted").and_then(Value::as_bool) == Some(true) {
        Ok(())
    } else {
        Err("MCU rejected the controller command without an accepted result".to_owned())
    }
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
    use tokio::sync::oneshot;
    use tokio_tungstenite::accept_async;
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
        assert!(validate_id(255).is_err());
    }

    #[test]
    fn limits_servo_scans_to_unicast_addresses() {
        assert!(validate_scan_range(1, 253).is_ok());
        assert!(validate_scan_range(0, 10).is_err());
        assert!(validate_scan_range(1, 254).is_err());
        assert!(validate_scan_range(10, 1).is_err());
    }

    #[test]
    fn permits_only_safe_servo_id_changes() {
        assert!(validate_id_change(1, 253).is_ok());
        assert!(validate_id_change(0, 5).is_err());
        assert!(validate_id_change(5, 254).is_err());
        assert!(validate_id_change(5, 5).is_err());
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
        assert!(!connection_is_usable_after(
            "Could not send a JSON-RPC command to the MCU: IO error: Broken pipe (os error 32)"
        ));
        assert!(!connection_is_usable_after(
            "Timed out waiting for an MCU JSON-RPC response"
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
    async fn browser_host_and_controller_complete_a_servo_scan_end_to_end() {
        let mcu_listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let mcu_address = mcu_listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (stream, _) = mcu_listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();

            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "system.ping");
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ready": true }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();

            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "servo.scan");
            assert_eq!(request["params"]["startId"], 1);
            assert_eq!(request["params"]["endId"], 10);
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ids": [5] }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();

            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "servo.setId");
            assert_eq!(request["params"]["currentId"], 5);
            assert_eq!(request["params"]["newId"], 6);
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "previousId": 5, "newId": 6 }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        let (host_address, shutdown, host) = spawn_test_host(AppState::default()).await;
        let stream = TcpStream::connect(host_address).await.unwrap();
        let endpoint = format!("ws://{host_address}/ws");
        let (mut browser, _) = client_async(endpoint, stream).await.unwrap();

        browser
            .send(ControllerMessage::Text(
                serde_json::json!({
                    "type": "connect",
                    "host": "127.0.0.1",
                    "port": mcu_address.port()
                })
                .to_string()
                .into(),
            ))
            .await
            .unwrap();
        assert_eq!(receive_browser_response(&mut browser).await["type"], "connected");

        browser
            .send(ControllerMessage::Text(
                serde_json::json!({
                    "type": "scan_servos",
                    "start_id": 1,
                    "end_id": 10
                })
                .to_string()
                .into(),
            ))
            .await
            .unwrap();
        let response = receive_browser_response(&mut browser).await;
        assert_eq!(response["type"], "servo_scan");
        assert_eq!(response["start_id"], 1);
        assert_eq!(response["end_id"], 10);
        assert_eq!(response["ids"], serde_json::json!([5]));

        browser
            .send(ControllerMessage::Text(
                serde_json::json!({
                    "type": "set_servo_id",
                    "current_id": 5,
                    "new_id": 6
                })
                .to_string()
                .into(),
            ))
            .await
            .unwrap();
        let response = receive_browser_response(&mut browser).await;
        assert_eq!(response["type"], "servo_id_changed");
        assert_eq!(response["previous_id"], 5);
        assert_eq!(response["new_id"], 6);

        drop(browser);
        shutdown.send(()).unwrap();
        host.await.unwrap();
        mcu.await.unwrap();
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
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["jsonrpc"], "2.0");
            assert_eq!(request["method"], "servo.setPosition");
            assert_eq!(request["params"]["id"], 5);
            assert_eq!(request["params"]["position"], 1625);
            assert_eq!(request["params"]["acceleration"], 20);
            assert_eq!(request["params"]["torqueLimit"], 1000);
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({ "jsonrpc": "2.0", "id": 0, "result": { "accepted": true } })
                        .to_string()
                        .into(),
                ))
                .await
                .unwrap();
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({ "jsonrpc": "2.0", "id": request["id"], "result": { "accepted": true } })
                        .to_string()
                        .into(),
                ))
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
    async fn controller_mode_scans_servo_ids_on_the_mcu() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "servo.scan");
            assert_eq!(request["params"]["startId"], 1);
            assert_eq!(request["params"]["endId"], 10);
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ids": [2, 5, 7] }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        let mut bridge = controller_bridge(address).await;
        let ServerMessage::ServoScan {
            start_id,
            end_id,
            ids,
        } = scan_servos(&mut bridge, 1, 10).await.unwrap()
        else {
            panic!("servo scan should return a scan response");
        };
        assert_eq!((start_id, end_id), (1, 10));
        assert_eq!(ids, vec![2, 5, 7]);
        mcu.await.unwrap();
    }

    #[tokio::test]
    async fn controller_mode_changes_a_servo_id_on_the_mcu() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "servo.setId");
            assert_eq!(request["params"]["currentId"], 5);
            assert_eq!(request["params"]["newId"], 6);
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "previousId": 5, "newId": 6 }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        let mut bridge = controller_bridge(address).await;
        let ServerMessage::ServoIdChanged {
            previous_id,
            new_id,
        } = set_servo_id(&mut bridge, 5, 6).await.unwrap()
        else {
            panic!("servo ID change should return its old and new IDs");
        };
        assert_eq!((previous_id, new_id), (5, 6));
        mcu.await.unwrap();
    }

    #[tokio::test]
    async fn raw_tunnel_enable_is_an_explicit_json_rpc_command() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let mcu = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "debug.enableRawTunnel");
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "port": RAW_TUNNEL_TCP_PORT, "active": true }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        let mut bridge = controller_bridge(address).await;
        let result = bridge
            .controller_request("debug.enableRawTunnel", serde_json::json!({}))
            .await
            .unwrap();
        assert_eq!(result["port"], RAW_TUNNEL_TCP_PORT);
        assert_eq!(result["active"], true);
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
    async fn reconnecting_to_the_same_mcu_health_checks_its_websocket_client() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let state = AppState::default();
        let controller = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            for _ in 0..2 {
                let request = receive_json_rpc_request(&mut websocket).await;
                assert_eq!(request["method"], "system.ping");
                websocket
                    .send(ControllerMessage::Text(
                        serde_json::json!({
                            "jsonrpc": "2.0",
                            "id": request["id"],
                            "result": { "ready": true }
                        })
                        .to_string()
                        .into(),
                    ))
                    .await
                    .unwrap();
            }
        });

        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        controller.await.unwrap();
    }

    #[tokio::test]
    async fn reconnecting_replaces_a_stale_controller_websocket() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let state = AppState::default();
        let controller = tokio::spawn(async move {
            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "system.ping");
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ready": true }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
            drop(websocket);

            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "system.ping");
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ready": true }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        controller.await.unwrap();
    }

    #[tokio::test]
    async fn reconnecting_waits_for_a_controller_listener_returning_after_handover() {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let port = listener.local_addr().unwrap().port();
        let state = AppState::default();
        let controller = tokio::spawn(async move {
            // The first TCP connection simulates an MCU socket that has
            // accepted a replacement but has not completed its WebSocket
            // handover yet. The host must retry instead of surfacing this
            // transient state to the browser.
            let (first, _) = listener.accept().await.unwrap();
            sleep(SERVO_TIMEOUT + CONTROLLER_CONNECT_RETRY_INTERVAL).await;
            drop(first);

            let (stream, _) = listener.accept().await.unwrap();
            let mut websocket = accept_async(stream).await.unwrap();
            let request = receive_json_rpc_request(&mut websocket).await;
            assert_eq!(request["method"], "system.ping");
            websocket
                .send(ControllerMessage::Text(
                    serde_json::json!({
                        "jsonrpc": "2.0",
                        "id": request["id"],
                        "result": { "ready": true }
                    })
                    .to_string()
                    .into(),
                ))
                .await
                .unwrap();
        });

        assert!(matches!(
            connect(&state, "127.0.0.1".to_owned(), port).await,
            ServerMessage::Connected { .. }
        ));
        controller.await.unwrap();
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

    async fn spawn_test_host(
        state: AppState,
    ) -> (SocketAddr, oneshot::Sender<()>, tokio::task::JoinHandle<()>) {
        let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
        let address = listener.local_addr().unwrap();
        let (shutdown, shutdown_requested) = oneshot::channel();
        let host = tokio::spawn(async move {
            axum::serve(
                listener,
                app(state).into_make_service_with_connect_info::<SocketAddr>(),
            )
            .with_graceful_shutdown(async move {
                let _ = shutdown_requested.await;
            })
            .await
            .unwrap();
        });
        (address, shutdown, host)
    }

    async fn bridge_for(address: SocketAddr) -> BridgeConnection {
        raw_bridge(address).await
    }

    async fn raw_bridge(address: SocketAddr) -> BridgeConnection {
        BridgeConnection {
            peer: address.to_string(),
            transport: BridgeTransport::DebugRaw(TcpStream::connect(address).await.unwrap()),
            next_request_id: 1,
        }
    }

    async fn controller_bridge(address: SocketAddr) -> BridgeConnection {
        let stream = TcpStream::connect(address).await.unwrap();
        let endpoint = format!("ws://{address}{CONTROLLER_WEBSOCKET_PATH}");
        let (websocket, _) = client_async(endpoint, stream).await.unwrap();
        BridgeConnection {
            peer: address.to_string(),
            transport: BridgeTransport::Controller(Box::new(websocket)),
            next_request_id: 1,
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

    async fn receive_json_rpc_request(websocket: &mut WebSocketStream<TcpStream>) -> Value {
        let message = websocket.next().await.unwrap().unwrap();
        let ControllerMessage::Text(text) = message else {
            panic!("JSON-RPC request must be a WebSocket text message");
        };
        serde_json::from_str(&text).unwrap()
    }

    async fn receive_browser_response(websocket: &mut WebSocketStream<TcpStream>) -> Value {
        let message = websocket.next().await.unwrap().unwrap();
        let ControllerMessage::Text(text) = message else {
            panic!("browser response must be a WebSocket text message");
        };
        serde_json::from_str(&text).unwrap()
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
