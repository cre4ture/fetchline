# Fetchline Controller Protocol v1

The MCU accepts one TCP client on port 3333. In its default mode, every TCP
message is a fixed 16-byte Controller Protocol frame. All multi-byte fields are
little-endian.

| Byte range | Field |
| --- | --- |
| 0–1 | ASCII magic `FL` |
| 2 | Protocol version (`1`) |
| 3 | Command or response code |
| 4–7 | Request sequence (`u32`) |
| 8 | Servo ID / response servo ID |
| 9 | Command option |
| 10–11 | Primary value (`u16`) |
| 12–13 | Secondary value (`u16`) |
| 14–15 | Error detail (`u16`) |

The client selects a new sequence for each request. The MCU always copies that
sequence into its response. Clients must discard a response whose sequence does
not match the request they are awaiting.

## Requests

| Code | Command | Fields |
| --- | --- | --- |
| `0x01` | `Ping` | None |
| `0x10` | `StartMotor` | ID; option `0` clockwise or `1` counter-clockwise; primary speed (`0..4095`); secondary acceleration (`0..254`) |
| `0x11` | `StopMotor` | ID |
| `0x12` | `SetPosition` | ID; option acceleration (`0..254`); primary position (`0..4095`); secondary RAM torque limit (`0..1000`) |
| `0x13` | `ReadPosition` | ID |
| `0x7e` | `OpenRawTunnel` | Debug only; no fields |

## Responses

| Code | Response | Fields |
| --- | --- | --- |
| `0x80` | `Ack` | None |
| `0x81` | `Position` | ID; primary signed 16-bit STS present position |
| `0x82` | `Error` | Primary stable error code; detail is command-specific |
| `0x83` | `RawTunnelReady` | None |

Error codes are `1` unsupported command, `2` invalid servo ID, `3` invalid
argument, `4` local STS timeout, `5` invalid STS reply, `6` servo-reported STS
error (detail is the status byte), `7` UART transport failure, and `8` invalid
request.

## Debug raw tunnel

`OpenRawTunnel` is deliberately a command, not another listening port. The MCU
replies with `RawTunnelReady` and immediately changes only that TCP session into
the old unframed bidirectional UART tunnel. The client must use the same socket
for raw bytes. Disconnecting it restores the normal Controller Protocol for the
next TCP client.
