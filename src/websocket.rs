//! Minimal RFC 6455 server support for the MCU JSON-RPC endpoint.
//!
//! The controller accepts one unfragmented text message at a time. Browser and
//! standard WebSocket clients mask client frames as required by RFC 6455.

#![allow(
    clippy::large_stack_frames,
    reason = "the bounded handshake and text buffers are intentionally retained by the TCP session"
)]

use base64ct::{Base64, Encoding};
use core::{convert::TryFrom, iter::Iterator};
use embassy_net::tcp::TcpSocket;
use embedded_io_async::{Read, Write};
use sha1::{Digest, Sha1};

use fetchline_protocol::CONTROLLER_WEBSOCKET_PATH;

const HTTP_REQUEST_MAX_LEN: usize = 1024;
const WEBSOCKET_GUID: &[u8] = b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

#[derive(Clone, Copy, Debug)]
pub enum Error {
    Closed,
    Io,
    InvalidHandshake,
    InvalidFrame,
    MessageTooLarge,
    InvalidText,
}

pub async fn upgrade(socket: &mut TcpSocket<'_>) -> Result<(), Error> {
    let mut request = [0_u8; HTTP_REQUEST_MAX_LEN];
    let request_len = read_http_request(socket, &mut request).await?;
    let request =
        core::str::from_utf8(&request[..request_len]).map_err(|_| Error::InvalidHandshake)?;
    let key = validate_handshake(request)?;

    let mut key_and_guid = [0_u8; 64];
    let key_bytes = key.as_bytes();
    let total_len = key_bytes.len() + WEBSOCKET_GUID.len();
    if total_len > key_and_guid.len() {
        return Err(Error::InvalidHandshake);
    }
    key_and_guid[..key_bytes.len()].copy_from_slice(key_bytes);
    key_and_guid[key_bytes.len()..total_len].copy_from_slice(WEBSOCKET_GUID);
    let digest = Sha1::digest(&key_and_guid[..total_len]);
    let mut accept = [0_u8; 28];
    let accept =
        Base64::encode(digest.as_ref(), &mut accept).map_err(|_| Error::InvalidHandshake)?;

    const PREFIX: &[u8] = b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: ";
    const SUFFIX: &[u8] = b"\r\n\r\n";
    let mut response = [0_u8; 160];
    let response_len = PREFIX.len() + accept.len() + SUFFIX.len();
    response[..PREFIX.len()].copy_from_slice(PREFIX);
    response[PREFIX.len()..PREFIX.len() + accept.len()].copy_from_slice(accept.as_bytes());
    response[PREFIX.len() + accept.len()..response_len].copy_from_slice(SUFFIX);
    socket
        .write_all(&response[..response_len])
        .await
        .map_err(|_| Error::Io)
}

pub async fn read_text<'a>(
    socket: &mut TcpSocket<'_>,
    buffer: &'a mut [u8],
) -> Result<&'a str, Error> {
    loop {
        let mut header = [0_u8; 2];
        read_exact(socket, &mut header).await?;
        if header[0] & 0x70 != 0 || header[0] & 0x80 == 0 || header[1] & 0x80 == 0 {
            return Err(Error::InvalidFrame);
        }
        let opcode = header[0] & 0x0f;
        let payload_len = read_payload_len(socket, header[1] & 0x7f).await?;
        if payload_len > buffer.len() {
            return Err(Error::MessageTooLarge);
        }
        let mut mask = [0_u8; 4];
        read_exact(socket, &mut mask).await?;
        read_exact(socket, &mut buffer[..payload_len]).await?;
        for (index, byte) in buffer[..payload_len].iter_mut().enumerate() {
            *byte ^= mask[index % mask.len()];
        }

        match opcode {
            0x01 => {
                return core::str::from_utf8(&buffer[..payload_len])
                    .map_err(|_| Error::InvalidText);
            }
            0x08 => {
                write_frame(socket, 0x08, &[]).await?;
                return Err(Error::Closed);
            }
            0x09 => write_frame(socket, 0x0a, &buffer[..payload_len]).await?,
            0x0a => continue,
            _ => return Err(Error::InvalidFrame),
        }
    }
}

pub async fn write_text(socket: &mut TcpSocket<'_>, text: &[u8]) -> Result<(), Error> {
    write_frame(socket, 0x01, text).await
}

async fn read_http_request(socket: &mut TcpSocket<'_>, buffer: &mut [u8]) -> Result<usize, Error> {
    let mut length = 0;
    while length < buffer.len() {
        read_exact(socket, &mut buffer[length..length + 1]).await?;
        length += 1;
        if length >= 4 && buffer[length - 4..length] == *b"\r\n\r\n" {
            return Ok(length);
        }
    }
    Err(Error::InvalidHandshake)
}

fn validate_handshake(request: &str) -> Result<&str, Error> {
    let mut lines = request.split("\r\n");
    let request_line = lines.next().ok_or(Error::InvalidHandshake)?;
    let expected_request = if CONTROLLER_WEBSOCKET_PATH == "/" {
        "GET / HTTP/1.1"
    } else {
        "GET /rpc HTTP/1.1"
    };
    if request_line != expected_request {
        return Err(Error::InvalidHandshake);
    }

    let mut has_upgrade = false;
    let mut has_connection_upgrade = false;
    let mut has_version = false;
    let mut key = None;
    for line in lines {
        let Some((name, value)) = line.split_once(':') else {
            continue;
        };
        let value = value.trim();
        if name.eq_ignore_ascii_case("upgrade") && value.eq_ignore_ascii_case("websocket") {
            has_upgrade = true;
        } else if name.eq_ignore_ascii_case("connection")
            && value
                .split(',')
                .any(|part| part.trim().eq_ignore_ascii_case("upgrade"))
        {
            has_connection_upgrade = true;
        } else if name.eq_ignore_ascii_case("sec-websocket-version") && value == "13" {
            has_version = true;
        } else if name.eq_ignore_ascii_case("sec-websocket-key") {
            key = Some(value);
        }
    }
    let key = key.ok_or(Error::InvalidHandshake)?;
    let mut decoded_key = [0_u8; 16];
    if Base64::decode(key, &mut decoded_key).is_err() {
        return Err(Error::InvalidHandshake);
    }
    if !has_upgrade || !has_connection_upgrade || !has_version {
        return Err(Error::InvalidHandshake);
    }
    Ok(key)
}

async fn read_payload_len(socket: &mut TcpSocket<'_>, first_len: u8) -> Result<usize, Error> {
    match first_len {
        0..=125 => Ok(first_len as usize),
        126 => {
            let mut bytes = [0_u8; 2];
            read_exact(socket, &mut bytes).await?;
            Ok(u16::from_be_bytes(bytes) as usize)
        }
        _ => Err(Error::MessageTooLarge),
    }
}

async fn write_frame(socket: &mut TcpSocket<'_>, opcode: u8, payload: &[u8]) -> Result<(), Error> {
    let mut header = [0_u8; 4];
    header[0] = 0x80 | opcode;
    let header_len = if payload.len() <= 125 {
        header[1] = payload.len() as u8;
        2
    } else if let Ok(length) = u16::try_from(payload.len()) {
        header[1] = 126;
        header[2..4].copy_from_slice(&length.to_be_bytes());
        4
    } else {
        return Err(Error::MessageTooLarge);
    };
    socket
        .write_all(&header[..header_len])
        .await
        .map_err(|_| Error::Io)?;
    socket.write_all(payload).await.map_err(|_| Error::Io)
}

async fn read_exact(socket: &mut TcpSocket<'_>, buffer: &mut [u8]) -> Result<(), Error> {
    socket.read_exact(buffer).await.map_err(|_| Error::Closed)
}
