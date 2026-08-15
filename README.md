# fetchline

Bare-metal Rust starter project for the EGBO mini ESP32-C3 development board
with a built-in 0.42-inch OLED.

The firmware initializes the onboard display and draws a small `FETCHLINE /
ESP32-C3 / READY` test screen. It also writes a heartbeat to the USB serial
monitor every five seconds.

## Board configuration

| Function | Configuration |
| --- | --- |
| MCU | ESP32-C3 (RISC-V, 160 MHz) |
| Flash | 4 MB |
| OLED | SSD1315 (reported), 72 x 40 pixels |
| OLED address | `0x3c` (presumed) |
| OLED SDA | GPIO5 |
| OLED SCL | GPIO6 |
| Boot button | GPIO9 |

The pin assignments come from the supplied product image, and the product
overview reports an SSD1315 controller. The firmware uses the mature
`ssd1306` Rust crate because the SSD1315 supports the command subset used here
and that crate provides the required 72 x 40 panel geometry and column offset.

The address is not stated in the supplied information. SSD1315 supports both
`0x3c` and `0x3d`; this project starts with the address commonly used by this
board family. If the screen stays blank, scan the I²C bus and change
`OLED_I2C_ADDRESS` in `src/lib.rs` if necessary.

## Prerequisites

Install Rust with [rustup](https://rustup.rs/), then install the Espressif
flashing tool:

```sh
cargo install espflash --locked
```

The checked-in `rust-toolchain.toml` installs the stable compiler, `rust-src`,
and the `riscv32imc-unknown-none-elf` target automatically.

On Linux, your user must have permission to access the board's serial device.
Depending on the distribution, that can require membership in the `dialout` or
`uucp` group, or an appropriate udev rule.

## Build and flash

Connect the board through its USB-C data port, then run:

```sh
cargo run --release
```

The Cargo runner flashes the ESP32-C3 and opens a serial monitor. If automatic
download mode does not work, hold **BOOT**, tap **RST**, release **BOOT**, and
run the command again.

Build without touching hardware with:

```sh
cargo build --release
```

## Next steps

The starter deliberately does not enable Wi-Fi or Bluetooth. Add those through
the `esp-radio` ecosystem when the application needs them; this keeps the
initial firmware and dependency set small.

Useful references:

- [Rust on ESP Book](https://docs.espressif.com/projects/rust/book/)
- [`esp-hal` documentation for ESP32-C3](https://docs.espressif.com/projects/rust/esp-hal/latest/esp32c3/esp_hal/)
- [`ssd1306` driver documentation](https://docs.rs/ssd1306/0.10.0/ssd1306/)
