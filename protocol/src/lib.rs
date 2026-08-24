#![no_std]
//! Version 1 of the Fetchline controller protocol.
//!
//! The protocol uses fixed-size frames over TCP.  TCP retains the byte order,
//! while the frame sequence makes delayed responses unambiguous to clients.
//! Servo packets never leave the MCU in controller mode.

/// TCP port used by the controller API and the explicitly requested debug tunnel.
pub const CONTROLLER_TCP_PORT: u16 = 3333;
/// Number of bytes in every version 1 protocol frame.
pub const FRAME_LEN: usize = 16;

const MAGIC: [u8; 2] = *b"FL";
const VERSION: u8 = 1;

const COMMAND_PING: u8 = 0x01;
const COMMAND_START_MOTOR: u8 = 0x10;
const COMMAND_STOP_MOTOR: u8 = 0x11;
const COMMAND_SET_POSITION: u8 = 0x12;
const COMMAND_READ_POSITION: u8 = 0x13;
const COMMAND_OPEN_RAW_TUNNEL: u8 = 0x7e;

const RESPONSE_ACK: u8 = 0x80;
const RESPONSE_POSITION: u8 = 0x81;
const RESPONSE_ERROR: u8 = 0x82;
const RESPONSE_RAW_TUNNEL_READY: u8 = 0x83;

/// A controller command sent from a host to the MCU.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    /// Verify that the MCU is running this controller protocol version.
    Ping,
    /// Select continuous mode and start the motor.
    StartMotor {
        /// STS servo ID.
        id: u8,
        /// `true` selects counter-clockwise rotation.
        counter_clockwise: bool,
        /// STS speed in the range 0 through 4095.
        speed: u16,
        /// STS acceleration in the range 0 through 254.
        acceleration: u8,
    },
    /// Command a controlled stop for a continuous motor.
    StopMotor {
        /// STS servo ID.
        id: u8,
    },
    /// Set the target of a position servo.
    SetPosition {
        /// STS servo ID.
        id: u8,
        /// Target in STS position units.
        position: u16,
        /// STS acceleration in the range 0 through 254.
        acceleration: u8,
        /// RAM torque limit in the range 0 through 1000.
        torque_limit: u16,
    },
    /// Read the present position of a servo.
    ReadPosition {
        /// STS servo ID.
        id: u8,
    },
    /// Turn this TCP session into the raw UART debug tunnel until it disconnects.
    OpenRawTunnel,
}

/// A controller result sent from the MCU to the host.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Response {
    /// The request completed successfully.
    Ack,
    /// A servo position read completed successfully.
    Position {
        /// STS servo ID.
        id: u8,
        /// Signed STS present-position value.
        position: i16,
    },
    /// The caller may now use this TCP stream as a raw UART tunnel.
    RawTunnelReady,
    /// The request failed on the MCU.
    Error {
        /// Stable error class.
        code: ErrorCode,
        /// Error-specific detail, such as the STS status byte.
        detail: u16,
    },
}

/// Stable MCU error classes returned in [`Response::Error`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u16)]
pub enum ErrorCode {
    /// The command byte is not defined by this protocol version.
    UnsupportedCommand = 1,
    /// The selected servo ID is invalid.
    InvalidServoId = 2,
    /// A command argument is outside its documented range.
    InvalidArgument = 3,
    /// No complete STS status packet arrived before the MCU-local deadline.
    ServoTimeout = 4,
    /// The STS reply framing, ID, checksum, or length was invalid.
    InvalidServoReply = 5,
    /// The servo returned a non-zero STS status byte.
    ServoReportedError = 6,
    /// UART I/O failed while executing the command.
    ServoTransport = 7,
    /// The frame was syntactically valid but not a host request.
    InvalidRequest = 8,
}

impl ErrorCode {
    /// Decodes the numeric error class stored in a response frame.
    #[must_use]
    pub const fn from_u16(value: u16) -> Option<Self> {
        match value {
            1 => Some(Self::UnsupportedCommand),
            2 => Some(Self::InvalidServoId),
            3 => Some(Self::InvalidArgument),
            4 => Some(Self::ServoTimeout),
            5 => Some(Self::InvalidServoReply),
            6 => Some(Self::ServoReportedError),
            7 => Some(Self::ServoTransport),
            8 => Some(Self::InvalidRequest),
            _ => None,
        }
    }
}

/// Failure to decode a protocol frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// The frame does not start with the Fetchline magic bytes.
    BadMagic,
    /// The peer uses an unsupported protocol version.
    UnsupportedVersion,
    /// The command or response byte is not recognised for the requested conversion.
    UnknownMessage,
    /// A field that is represented as a byte is not valid for its command.
    InvalidField,
}

/// A wire-format protocol frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    kind: u8,
    sequence: u32,
    servo_id: u8,
    option: u8,
    value: u16,
    extra: u16,
    detail: u16,
}

impl Frame {
    /// Encodes a command with its caller-selected sequence number.
    #[must_use]
    pub const fn command(sequence: u32, command: Command) -> Self {
        match command {
            Command::Ping => Self {
                kind: COMMAND_PING,
                sequence,
                servo_id: 0,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
            Command::StartMotor {
                id,
                counter_clockwise,
                speed,
                acceleration,
            } => Self {
                kind: COMMAND_START_MOTOR,
                sequence,
                servo_id: id,
                option: if counter_clockwise { 1 } else { 0 },
                value: speed,
                extra: acceleration as u16,
                detail: 0,
            },
            Command::StopMotor { id } => Self {
                kind: COMMAND_STOP_MOTOR,
                sequence,
                servo_id: id,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
            Command::SetPosition {
                id,
                position,
                acceleration,
                torque_limit,
            } => Self {
                kind: COMMAND_SET_POSITION,
                sequence,
                servo_id: id,
                option: acceleration,
                value: position,
                extra: torque_limit,
                detail: 0,
            },
            Command::ReadPosition { id } => Self {
                kind: COMMAND_READ_POSITION,
                sequence,
                servo_id: id,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
            Command::OpenRawTunnel => Self {
                kind: COMMAND_OPEN_RAW_TUNNEL,
                sequence,
                servo_id: 0,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
        }
    }

    /// Encodes a response for a previously received command sequence.
    #[must_use]
    pub const fn response(sequence: u32, response: Response) -> Self {
        match response {
            Response::Ack => Self {
                kind: RESPONSE_ACK,
                sequence,
                servo_id: 0,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
            Response::Position { id, position } => Self {
                kind: RESPONSE_POSITION,
                sequence,
                servo_id: id,
                option: 0,
                value: position as u16,
                extra: 0,
                detail: 0,
            },
            Response::RawTunnelReady => Self {
                kind: RESPONSE_RAW_TUNNEL_READY,
                sequence,
                servo_id: 0,
                option: 0,
                value: 0,
                extra: 0,
                detail: 0,
            },
            Response::Error { code, detail } => Self {
                kind: RESPONSE_ERROR,
                sequence,
                servo_id: 0,
                option: 0,
                value: code as u16,
                extra: 0,
                detail,
            },
        }
    }

    /// Serializes the frame to its fixed-size TCP representation.
    #[must_use]
    pub const fn encode(self) -> [u8; FRAME_LEN] {
        let sequence = self.sequence.to_le_bytes();
        let value = self.value.to_le_bytes();
        let extra = self.extra.to_le_bytes();
        let detail = self.detail.to_le_bytes();
        [
            MAGIC[0],
            MAGIC[1],
            VERSION,
            self.kind,
            sequence[0],
            sequence[1],
            sequence[2],
            sequence[3],
            self.servo_id,
            self.option,
            value[0],
            value[1],
            extra[0],
            extra[1],
            detail[0],
            detail[1],
        ]
    }

    /// Parses and validates the shared envelope of a wire-format frame.
    pub const fn decode(bytes: [u8; FRAME_LEN]) -> Result<Self, DecodeError> {
        if bytes[0] != MAGIC[0] || bytes[1] != MAGIC[1] {
            return Err(DecodeError::BadMagic);
        }
        if bytes[2] != VERSION {
            return Err(DecodeError::UnsupportedVersion);
        }
        Ok(Self {
            kind: bytes[3],
            sequence: u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
            servo_id: bytes[8],
            option: bytes[9],
            value: u16::from_le_bytes([bytes[10], bytes[11]]),
            extra: u16::from_le_bytes([bytes[12], bytes[13]]),
            detail: u16::from_le_bytes([bytes[14], bytes[15]]),
        })
    }

    /// Returns the command represented by this frame.
    pub const fn as_command(self) -> Result<Command, DecodeError> {
        match self.kind {
            COMMAND_PING => Ok(Command::Ping),
            COMMAND_START_MOTOR if self.option <= 1 && self.extra <= u8::MAX as u16 => {
                Ok(Command::StartMotor {
                    id: self.servo_id,
                    counter_clockwise: self.option == 1,
                    speed: self.value,
                    acceleration: self.extra as u8,
                })
            }
            COMMAND_START_MOTOR => Err(DecodeError::InvalidField),
            COMMAND_STOP_MOTOR => Ok(Command::StopMotor { id: self.servo_id }),
            COMMAND_SET_POSITION => Ok(Command::SetPosition {
                id: self.servo_id,
                position: self.value,
                acceleration: self.option,
                torque_limit: self.extra,
            }),
            COMMAND_READ_POSITION => Ok(Command::ReadPosition { id: self.servo_id }),
            COMMAND_OPEN_RAW_TUNNEL => Ok(Command::OpenRawTunnel),
            _ => Err(DecodeError::UnknownMessage),
        }
    }

    /// Returns the response represented by this frame.
    pub const fn as_response(self) -> Result<Response, DecodeError> {
        match self.kind {
            RESPONSE_ACK => Ok(Response::Ack),
            RESPONSE_POSITION => Ok(Response::Position {
                id: self.servo_id,
                position: self.value as i16,
            }),
            RESPONSE_RAW_TUNNEL_READY => Ok(Response::RawTunnelReady),
            RESPONSE_ERROR => match ErrorCode::from_u16(self.value) {
                Some(code) => Ok(Response::Error {
                    code,
                    detail: self.detail,
                }),
                None => Err(DecodeError::InvalidField),
            },
            _ => Err(DecodeError::UnknownMessage),
        }
    }

    /// The sequence number that correlates a response with its command.
    #[must_use]
    pub const fn sequence(self) -> u32 {
        self.sequence
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn position_command_round_trips() {
        let frame = Frame::command(
            42,
            Command::SetPosition {
                id: 7,
                position: 1234,
                acceleration: 20,
                torque_limit: 1000,
            },
        );
        let decoded = Frame::decode(frame.encode()).unwrap();
        assert_eq!(decoded.sequence(), 42);
        assert_eq!(
            decoded.as_command(),
            Ok(Command::SetPosition {
                id: 7,
                position: 1234,
                acceleration: 20,
                torque_limit: 1000,
            })
        );
    }

    #[test]
    fn error_response_round_trips() {
        let frame = Frame::response(
            9,
            Response::Error {
                code: ErrorCode::ServoReportedError,
                detail: 0x20,
            },
        );
        let decoded = Frame::decode(frame.encode()).unwrap();
        assert_eq!(
            decoded.as_response(),
            Ok(Response::Error {
                code: ErrorCode::ServoReportedError,
                detail: 0x20,
            })
        );
    }

    #[test]
    fn rejects_a_bad_envelope_and_invalid_motor_direction() {
        let mut bytes = Frame::command(1, Command::OpenRawTunnel).encode();
        bytes[0] = 0;
        assert_eq!(Frame::decode(bytes), Err(DecodeError::BadMagic));

        let mut bytes = Frame::command(
            1,
            Command::StartMotor {
                id: 1,
                counter_clockwise: false,
                speed: 100,
                acceleration: 20,
            },
        )
        .encode();
        bytes[9] = 2;
        assert_eq!(
            Frame::decode(bytes).unwrap().as_command(),
            Err(DecodeError::InvalidField)
        );
    }
}
