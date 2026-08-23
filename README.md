# fetchline

Bare-metal Rust firmware for the EGBO mini ESP32-C3 board with its built-in
0.42-inch OLED. It connects a Feetech FE-URT-2 adapter to Wi-Fi, allowing
Windows servo software to use a virtual COM port as though the STS servo bus
were connected locally.

The bridge is transparent and binary-safe:

```text
Windows COM port <-> raw TCP port 3333 <-> ESP32-C3 UART1 <-> FE-URT-2 <-> STS servos
```

## Linux host control panel

The `host/` directory contains a Linux PC application with a browser UI for
direct manual control. It owns the one raw-TCP connection to the MCU and serves
the UI on the local machine, so no browser extension and no virtual COM driver
is required.

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
MCU TCP connection attempts and failures, and every servo action with its
elapsed time. Individual servo timeouts, corrupt replies, and servo-reported
STS errors are logged with the servo ID while retaining the MCU TCP connection
when possible. For individual STS packet metadata, start the panel with:

```sh
just host-debug
```

Packet payloads are intentionally not logged. This avoids filling the log
during live slider movement while retaining the IDs, instructions, errors, and
timing needed to distinguish a host/MCU network failure from a servo-bus
failure.

Only one program may use the MCU TCP bridge. Close the virtual COM software and
any other `fetchline-host` page before connecting this panel. The host has no
authentication: every device that can reach its port can command physical
actuators. Keep it on a trusted LAN, or firewall the port / use a VPN.

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
| Network endpoint | raw TCP server, port 3333 |

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
Wi-Fi ready: IP 192.168.1.123/24, raw TCP port 3333
```

Reserve that address for the board in the router's DHCP settings so the Windows
virtual-COM configuration remains stable.

## Windows virtual COM port

[HW VSP3 Single](https://www.hw-group.com/software/hw-vsp3-virtual-serial-port)
is one Windows virtual serial port driver that can redirect a COM port to an
IP address and TCP port. Install it as an administrator, then:

1. Check connectivity in PowerShell:

   ```powershell
   Test-NetConnection 192.168.1.123 -Port 3333
   ```

2. Open **Virtual Serial Port** in HW VSP3 and choose an unused port such as
   `COM9`.
3. Enter the ESP32-C3's DHCP address and port `3333`.
4. Use normal client mode, turn off **TCP server mode**, and click **Create COM**.
5. Leave **NVT**, **NVT filter**, and **NVT port setup** disabled. Fetchline uses
   transparent raw TCP, not Telnet or RFC 2217 control sequences.
6. In the Feetech application, open that COM port at **1,000,000 baud, 8N1**.

The firmware's UART parameters are fixed; changing the baud rate in Windows
does not reconfigure the remote UART. A detailed example of creating a COM port
from an IP address and port is available in the
[Teltonika HW VSP3 guide](https://wiki.teltonika-networks.com/view/Connect_Serial_Devices_as_Virtual_COM_Ports_using_TRB145_and_HW_VSP3).

Only one Windows application can open a COM port, and fetchline accepts only one
TCP client. Close Feetech tools, terminals, or previous VSP connections that may
already own it before troubleshooting a connection.

## Security

Port 3333 has no authentication or encryption. Every byte received is sent to
physical actuators. Use this firmware only on a trusted private LAN or through a
VPN. Never expose or port-forward TCP 3333 to the public internet.

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
