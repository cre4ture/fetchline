//! Linux-local control panel for Feetech STS servos behind a fetchline bridge.
//!
//! A browser cannot make a raw TCP connection to the ESP32.  This program owns
//! that connection and exposes an HTTP/WebSocket interface to the bundled
//! browser UI.

use std::{env, sync::Arc, time::Duration};

use axum::{
    Router,
    extract::{
        State, WebSocketUpgrade,
        ws::{Message, WebSocket},
    },
    http::{HeaderValue, header},
    response::{IntoResponse, Response},
    routing::get,
};
use serde::{Deserialize, Serialize};
use tokio::{
    io::{AsyncReadExt, AsyncWriteExt},
    net::{TcpListener, TcpStream},
    sync::Mutex,
    time::timeout,
};

const DEFAULT_LISTEN_ADDRESS: &str = "0.0.0.0:8787";
const SERVO_TIMEOUT: Duration = Duration::from_millis(750);
const STS_HEADER: [u8; 2] = [0xff, 0xff];
const STS_BROADCAST_ID: u8 = 0xfe;
const STS_INSTRUCTION_READ: u8 = 0x02;
const STS_INSTRUCTION_WRITE: u8 = 0x03;
const STS_MODE: u8 = 33;
const STS_TORQUE_ENABLE: u8 = 40;
const STS_ACCELERATION: u8 = 41;
const STS_TORQUE_LIMIT: u8 = 48;
const STS_PRESENT_POSITION: u8 = 56;

#[derive(Clone, Default)]
struct AppState {
    bridge: Arc<Mutex<Option<BridgeConnection>>>,
}

struct BridgeConnection {
    peer: String,
    stream: TcpStream,
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
    let listen_address = env::args()
        .nth(1)
        .unwrap_or_else(|| DEFAULT_LISTEN_ADDRESS.to_owned());
    let listener = TcpListener::bind(&listen_address)
        .await
        .unwrap_or_else(|error| panic!("could not bind {listen_address}: {error}"));

    println!("Fetchline host UI listening on http://{listen_address}");
    println!("Open http://<LAN-IP-of-this-PC>:8787 from a device on the local network.");

    let app = Router::new()
        .route("/", get(index))
        .route("/app.js", get(javascript))
        .route("/styles.css", get(stylesheet))
        .route("/ws", get(websocket))
        .with_state(AppState::default());

    axum::serve(listener, app)
        .await
        .expect("local web server stopped unexpectedly");
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

async fn websocket(websocket: WebSocketUpgrade, State(state): State<AppState>) -> Response {
    websocket.on_upgrade(move |socket| client_session(socket, state))
}

async fn client_session(mut socket: WebSocket, state: AppState) {
    while let Some(message) = socket.recv().await {
        let response = match message {
            Ok(Message::Text(text)) => match serde_json::from_str(&text) {
                Ok(request) => handle_request(&state, request).await,
                Err(error) => ServerMessage::Error {
                    message: format!("Invalid control request: {error}"),
                    bridge_connected: false,
                },
            },
            Ok(Message::Close(_)) => break,
            Ok(Message::Ping(_)) | Ok(Message::Pong(_)) => continue,
            Ok(Message::Binary(_)) => ServerMessage::Error {
                message: "Binary WebSocket messages are not supported".to_owned(),
                bridge_connected: false,
            },
            Err(error) => {
                eprintln!("browser connection closed: {error}");
                break;
            }
        };

        let Ok(json) = serde_json::to_string(&response) else {
            eprintln!("could not serialize browser response");
            continue;
        };
        if socket.send(Message::Text(json.into())).await.is_err() {
            break;
        }
    }
}

async fn handle_request(state: &AppState, request: ClientMessage) -> ServerMessage {
    match request {
        ClientMessage::Connect { host, port } => connect(state, host, port).await,
        request => {
            let mut bridge = state.bridge.lock().await;
            let Some(connection) = bridge.as_mut() else {
                return ServerMessage::Error {
                    message: "Connect to the MCU before sending servo commands".to_owned(),
                    bridge_connected: false,
                };
            };

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
                Ok(response) => response,
                Err(error) => {
                    let bridge_connected = connection_is_usable_after(&error);
                    if bridge_connected {
                        eprintln!(
                            "servo command failed; retaining MCU connection {}: {error}",
                            connection.peer
                        );
                    } else {
                        eprintln!("dropping MCU connection {}: {error}", connection.peer);
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
        return ServerMessage::Error {
            message: "The MCU host name or IP address is required".to_owned(),
            bridge_connected: false,
        };
    }
    if port == 0 {
        return ServerMessage::Error {
            message: "The MCU TCP port must be between 1 and 65535".to_owned(),
            bridge_connected: false,
        };
    }

    let peer = format!("{host}:{port}");
    match timeout(SERVO_TIMEOUT, TcpStream::connect(&peer)).await {
        Ok(Ok(stream)) => {
            if let Err(error) = stream.set_nodelay(true) {
                return ServerMessage::Error {
                    message: format!("Connected to {peer}, but could not configure TCP: {error}"),
                    bridge_connected: false,
                };
            }
            let mut bridge = state.bridge.lock().await;
            *bridge = Some(BridgeConnection {
                peer: peer.clone(),
                stream,
            });
            ServerMessage::Connected { address: peer }
        }
        Ok(Err(error)) => ServerMessage::Error {
            message: format!("Could not connect to {peer}: {error}"),
            bridge_connected: false,
        },
        Err(_) => ServerMessage::Error {
            message: format!("Timed out connecting to {peer}"),
            bridge_connected: false,
        },
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
            Ok(position) => positions.push(Position { id, position }),
            Err(error) if connection_is_usable_after(&error) => {
                errors.push(format!("Servo {id}: {error}"));
            }
            Err(error) => return Err(error),
        }
    }
    Ok(ServerMessage::Positions { positions, errors })
}

/// Servo replies are independent of the TCP transport. Retain the bridge after
/// a servo timeout or malformed/error reply so the remaining servos remain
/// controllable. Broken TCP I/O is the only case that requires reconnecting.
fn connection_is_usable_after(error: &str) -> bool {
    !error.starts_with("Timed out sending command to the MCU")
        && !error.starts_with("Could not send command to the MCU")
        && !error.starts_with("Could not read STS servo reply")
}

fn validate_id(id: u8) -> Result<(), String> {
    if id == 0 || id == STS_BROADCAST_ID {
        Err("Servo ID must be between 1 and 253".to_owned())
    } else {
        Ok(())
    }
}

impl BridgeConnection {
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

        let stream = TcpStream::connect(address).await.unwrap();
        let mut bridge = BridgeConnection {
            peer: address.to_string(),
            stream,
        };
        start_motor(&mut bridge, 1, 1024, 20, Direction::Clockwise)
            .await
            .unwrap();
        servo.await.unwrap();
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

        let stream = TcpStream::connect(address).await.unwrap();
        let mut bridge = BridgeConnection {
            peer: address.to_string(),
            stream,
        };
        assert_eq!(read_position(&mut bridge, 2).await.unwrap(), 0x1234);
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

        let stream = TcpStream::connect(address).await.unwrap();
        let mut bridge = BridgeConnection {
            peer: address.to_string(),
            stream,
        };
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

    async fn receive_packet(stream: &mut TcpStream) -> Vec<u8> {
        let mut header_and_fixed = [0_u8; 4];
        stream.read_exact(&mut header_and_fixed).await.unwrap();
        assert_eq!(&header_and_fixed[..2], STS_HEADER);
        let length = header_and_fixed[3] as usize;
        let mut remaining = vec![0_u8; length];
        stream.read_exact(&mut remaining).await.unwrap();
        [header_and_fixed.to_vec(), remaining].concat()
    }

    fn status_packet(id: u8, payload: &[u8]) -> Vec<u8> {
        let length = u8::try_from(payload.len() + 2).unwrap();
        let mut packet = vec![0xff, 0xff, id, length, 0];
        packet.extend_from_slice(payload);
        packet.push(checksum_for_status(id, length, &packet[4..]));
        packet
    }
}
