#![no_std]

/// Fixed connections and display properties of the EGBO ESP32-C3 OLED board.
pub mod board {
    /// Display controller reported by the product listing.
    pub const OLED_CONTROLLER: &str = "SSD1315";
    /// GPIO connected to the OLED's I²C data line.
    pub const OLED_SDA_GPIO: u8 = 5;
    /// GPIO connected to the OLED's I²C clock line.
    pub const OLED_SCL_GPIO: u8 = 6;
    /// Presumed 7-bit I²C address used by the onboard display.
    pub const OLED_I2C_ADDRESS: u8 = 0x3c;
    /// Visible OLED width in pixels.
    pub const OLED_WIDTH: u32 = 72;
    /// Visible OLED height in pixels.
    pub const OLED_HEIGHT: u32 = 40;
    /// GPIO receiving data from the FE-URT-2 TXD pin.
    pub const SERVO_UART_RX_GPIO: u8 = 20;
    /// GPIO transmitting data to the FE-URT-2 RXD pin.
    pub const SERVO_UART_TX_GPIO: u8 = 21;
    /// Fixed Feetech STS bus baud rate.
    pub const SERVO_UART_BAUD: u32 = 1_000_000;
    /// Raw TCP port exposed to virtual-COM software.
    pub const BRIDGE_TCP_PORT: u16 = 3333;
}
