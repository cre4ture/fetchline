#![no_std]
//! Transport constants for the Fetchline controller API.
//!
//! The public controller API is JSON-RPC 2.0 carried by WebSocket text frames.
//! STS servo packets are always terminated on the MCU. The raw TCP endpoint is
//! deliberately separate and exists only after a JSON-RPC debug command enables it.

/// TCP port used by the JSON-RPC-over-WebSocket controller API.
pub const CONTROLLER_TCP_PORT: u16 = 3333;
/// TCP port that becomes reachable only while the raw tunnel is explicitly enabled.
pub const RAW_TUNNEL_TCP_PORT: u16 = 3334;
/// WebSocket path carrying JSON-RPC 2.0 text messages.
pub const CONTROLLER_WEBSOCKET_PATH: &str = "/rpc";
