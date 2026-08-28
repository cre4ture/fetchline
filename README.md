# fetchline

Bare-metal Rust firmware for the EGBO mini ESP32-C3 board with its built-in
0.42-inch OLED. It connects a Feetech FE-URT-2 adapter to Wi-Fi and exposes a
versioned, high-level controller API for STS/SMS servos.

The normal control path terminates the time-sensitive STS protocol on the MCU:

```text
Browser <-> Linux host <-> controller API / Wi-Fi <-> ESP32-C3 <-> UART1 <-> FE-URT-2 <-> STS servos
```

The MCU owns the UART transaction, local response deadline, and recovery from
late STS replies. Wi-Fi carries only controller requests and JSON-RPC results,
so a delayed network packet cannot be mistaken for a later servo response.

## Linux host control panel

The `host/` directory contains a Linux PC application with a browser UI for
direct manual control. It owns the one controller-API connection to the MCU and
serves the UI on the local machine, so no browser extension and no virtual COM
driver is required.

It controls Feetech **STS/SMS-compatible** servos with the normal STS protocol:

- Servo 1 is a continuous motor control: clockwise, counter-clockwise, and
  stop. Its maximum speed (percentage of the STS raw range) and acceleration
  profile are configurable. Starting it selects the servo's continuous mode,
  which is a persistent setting in the servo.
- Servos 2–7 each have configurable IDs, live position sliders, maximum
  acceleration, and holding torque limit. Each servo can be disabled; disabled
  servos are never read or commanded. Current positions are read when the MCU
  connects and when **Update positions** is pressed. A missing or faulty servo
  is reported individually and does not disconnect the remaining servo controls.
- **Find connected servos** searches a selectable address range directly on the
  MCU. It defaults to IDs 1–10 and accepts 1–255; the MCU probes the bus with
  its local STS deadline and returns every responding ID. Addresses 254 and 255
  are skipped because they are reserved by STS.
- The MCU address, IDs, enabled state, and all control limits are stored by
  the Linux host in `~/.config/fetchline-host/config.json` (or
  `$XDG_CONFIG_HOME/fetchline-host/config.json`). Every browser opening the
  host UI therefore receives the same configuration after a reload.

Run it on the Linux PC with a recent stable Rust toolchain:

```sh
just host
```

The application listens on all network interfaces by default. Open
`http://<Linux-PC-IP>:8787` from a device on the local network, then enter the
IP address shown on the MCU OLED. An alternative listen address can be given as
the first argument, for example:

```sh
just host 127.0.0.1:9000
```

## Host diagnostics

The host app writes diagnostics to
`~/.local/state/fetchline-host/fetchline-host.log` (or
`$XDG_STATE_HOME/fetchline-host/fetchline-host.log`). Its current log plus up
to three numbered archives are retained, each up to 5 MiB. View the latest
entries with:

```sh
just host-logs
```

The normal log records host startup, browser WebSocket connect/disconnect,
MCU controller-API connection attempts and failures, and every servo action
with its elapsed time. The MCU reports local STS timeouts, invalid replies, and
servo errors as structured JSON-RPC errors. Delayed API responses carry a
JSON-RPC request ID and are discarded by the host rather than being associated
with a later action. For controller diagnostics, start the panel with:

```sh
just host-debug
```

Packet payloads are intentionally not logged. This avoids filling the log
during live slider movement while retaining the IDs, actions, errors, and timing
needed to distinguish a host/MCU network failure from a servo-bus failure.

Only one program may use the MCU controller connection. The host has no
authentication: every device that can reach its port can command physical
actuators. Keep it on a trusted LAN, or firewall the port / use a VPN.

### Controller API

Port `3333` exposes [JSON-RPC 2.0](https://www.jsonrpc.org/specification) in
WebSocket text frames at `ws://<mcu-ip>:3333/rpc`. Requests use numeric
JSON-RPC IDs; the reply carries the same ID. The API methods are
`system.ping`, `motor.start`, `motor.stop`, `servo.setPosition`,
`servo.getPosition`, `servo.getPositions`, `servo.scan`, `debug.enableRawTunnel`, and
`debug.disableRawTunnel`. The complete method and parameter reference is in
[`protocol/README.md`](protocol/README.md).

For example, a position command is:

```json
{"jsonrpc":"2.0","id":42,"method":"servo.setPosition","params":{"id":5,"position":1625,"acceleration":20,"torqueLimit":1000}}
```

The raw UART tunnel exists only for special tests. Calling
`debug.enableRawTunnel` opens the separate TCP port `3334`; it does **not**
change or close the controller WebSocket. The RAW port stays open across any
number of raw-client disconnects and reconnects. Only
`debug.disableRawTunnel` closes it, including an active raw client. While it
is enabled, normal motor and servo methods fail with JSON-RPC error `-32010`.
The included host can enable it and connect its raw test path with:

```sh
just host-debug-tunnel
```

Do not use the debug tunnel for normal actuation: it intentionally restores the
old raw request/reply timing characteristics.

It uses DHCP, reconnects Wi-Fi automatically, accepts one TCP client at a time,
and keeps UART1 fixed at 1,000,000 baud, 8 data bits, no parity, and 1 stop bit.
After DHCP completes, the OLED shows the assigned IPv4 address across two lines
and keeps it visible while clients connect and disconnect. Startup or
configuration diagnostics are shown only until an address is available. The
assigned address is also printed to the USB serial monitor.

## Hardware configuration

| Function | Configuration |
| --- | --- |
| MCU | ESP32-C3, RISC-V, 160 MHz |
| Flash | 4 MB |
| OLED | SSD1315-compatible, 72 x 40 pixels |
| OLED I2C | address `0x3c`, SDA GPIO5, SCL GPIO6 |
| Servo UART | UART1, 1,000,000 baud, 8N1 |
| Servo UART RX | GPIO20, board pin `RX` |
| Servo UART TX | GPIO21, board pin `TX` |
| Network endpoint | JSON-RPC 2.0 WebSocket server at `ws://<IP>:3333/rpc` |

The OLED address is presumed from this board family because it was not listed
by the seller. If the screen stays blank, scan the I2C bus and try `0x3d` in
`src/lib.rs`.

## FE-URT-2 wiring

Set the FE-URT-2 TTL level selector to **3.3 V** before connecting it to the
ESP32-C3.

| ESP32-C3 board | FE-URT-2 UART header | Purpose |
| --- | --- | --- |
| `TX` / GPIO21 | `TXD` | Commands from Wi-Fi to the servos |
| `RX` / GPIO20 | `RXD` | Servo replies to Wi-Fi |
| `GND` | `GND` | Common signal reference |
| `5V` | `5V` | FE-URT-2 logic supply only |

Unlike a conventional UART adapter, the FE-URT-2's MCU header labels describe
the MCU pins that connect to them. Connect TX to TX and RX to RX, exactly as in
Feetech's Arduino diagram. The adapter performs the required TTL half-duplex bus
direction switching. See Feetech's official
[MCU wiring instructions](https://www.feetechrc.com/wp-content/6-how-does-single-chip-microcomputer-control-serial-port-steering-gear.html).

### Power safety

- Power the servos through the FE-URT-2 screw terminals with a supply suitable
  for the exact servo model. Do **not** power servos from the ESP32-C3 3.3 V or
  5 V pin.
- The servo supply can deliver substantial current. Set its voltage and current
  limit before attaching a servo, and start testing without a mechanical load.
- In MCU mode, power the FE-URT-2 logic from the ESP board's USB 5 V pin and
  leave the FE-URT-2 USB-C connector disconnected.
- For a direct FE-URT-2 USB test, first disconnect the ESP board's TXD, RXD,
  GND, and 5 V wires. Do not join two independently powered 5 V outputs.

Feetech lists the FE-URT-2 as a Type-C USB-to-TTL/RS485 programmer with a UART
header. Its supported range reaches 1 Mbps; see the
[official Feetech debugging-board listing](https://www.feetechrc.com/serial-port-series-steering-gear_50681).

## Prerequisites

Install Rust with [rustup](https://rustup.rs/), then install the Espressif
flashing tool:

```sh
cargo install espflash --locked
```

The checked-in toolchain configuration installs the stable compiler, `rust-src`,
and the `riscv32imc-unknown-none-elf` target. On Linux, the user running
`espflash` must have access to the board's serial device.

## Configure, build, and flash

Wi-Fi credentials are never compiled into the firmware. They are provisioned
once over USB into the final 64 KB of the board's 4 MB flash; normal firmware
updates leave this configuration area untouched.

Flash the firmware, then provision the network over USB:

```sh
just firmware-flash
just provision-wifi /dev/ttyACM0
```

`just` is a command runner. Install it on Linux, for example with
`cargo install just`, then run `just` to list all available targets. The host
and provisioning targets automatically select the Linux-native Rust target,
which avoids accidentally building them for the ESP32-C3.

The provisioner asks for the SSID and password and writes only the reserved
configuration sector at `0x3f0000`. Never use `espflash erase-flash`, because
that intentionally deletes this configuration. ESP32-C3 supports 2.4 GHz Wi-Fi,
not a 5 GHz-only network. If automatic download mode fails, hold **BOOT**, tap
**RST**, release **BOOT**, and run the command again.

After DHCP completes, the USB log contains a line similar to:

```text
Wi-Fi ready: IP 192.168.1.123/24, JSON-RPC WebSocket port 3333
```

Reserve that address for the board in the router's DHCP settings so controller
clients can reconnect to a stable endpoint.

## Raw UART debug tunnel

The old transparent UART bridge is no longer the default service. It is a
debug-only listener enabled by the JSON-RPC method `debug.enableRawTunnel`.
When enabled, port `3334` accepts one raw TCP client at a time. A client can
disconnect and reconnect later without disabling the listener. Use
`debug.disableRawTunnel` to close it explicitly. Generic virtual-COM tools
cannot use it by merely opening port `3333`; they must use `3334` after the
debug command has enabled that port. `just host-debug-tunnel` provides a
controlled browser-based raw test path.

## Security

Port `3333` has no authentication or encryption. Controller commands can move
physical actuators, and its debug command can open unrestricted raw UART port
`3334`. Use this firmware only on a trusted private LAN or through a VPN. Never
expose or port-forward either port to the public internet.

## Development checks

```sh
just check
```

Useful references:

- [Rust on ESP Book](https://docs.espressif.com/projects/rust/book/)
- [`esp-hal` documentation for ESP32-C3](https://docs.espressif.com/projects/rust/esp-hal/latest/esp32c3/esp_hal/)
- [Feetech Arduino servo library](https://github.com/ftservo/FTServo_Arduino)

## License

Licensed under the [MIT License](LICENSE).
