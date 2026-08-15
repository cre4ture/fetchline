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
}
