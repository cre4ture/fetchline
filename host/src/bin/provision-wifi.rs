use std::{env, fs, io::{self, Write}, process::Command};

const CONFIG_OFFSET: &str = "0x3f0000";
const CONFIG_SIZE: usize = 4096;
const SSID_MAX_LEN: usize = 32;
const PASSWORD_MAX_LEN: usize = 63;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = env::args().nth(1).unwrap_or_else(|| "/dev/ttyACM0".to_owned());
    let ssid = prompt("Wi-Fi SSID: ")?;
    let password = prompt("Wi-Fi password (input is visible): ")?;
    if ssid.is_empty() || ssid.len() > SSID_MAX_LEN || password.len() > PASSWORD_MAX_LEN {
        return Err("SSID must be 1–32 bytes and password 0–63 bytes".into());
    }

    let mut image = [0xff_u8; CONFIG_SIZE];
    image[..4].copy_from_slice(b"FLWC");
    image[4] = 1;
    image[5] = ssid.len() as u8;
    image[6] = password.len() as u8;
    image[12..12 + ssid.len()].copy_from_slice(ssid.as_bytes());
    image[12 + ssid.len()..12 + ssid.len() + password.len()].copy_from_slice(password.as_bytes());
    let checksum = checksum(&image[..8], &image[12..12 + ssid.len() + password.len()]);
    image[8..12].copy_from_slice(&checksum.to_le_bytes());

    let path = env::temp_dir().join("fetchline-wifi-config.bin");
    fs::write(&path, image)?;
    let result = Command::new("espflash")
        .args(["write-bin", "--port", &port, "--chip", "esp32c3", "--before", "usb-reset", "--after", "hard-reset", "--non-interactive", CONFIG_OFFSET])
        .arg(&path)
        .status()?;
    fs::remove_file(path)?;
    if result.success() { Ok(()) } else { Err("Wi-Fi provisioning failed".into()) }
}

fn prompt(label: &str) -> io::Result<String> {
    print!("{label}");
    io::stdout().flush()?;
    let mut value = String::new();
    io::stdin().read_line(&mut value)?;
    Ok(value.trim_end_matches(['\r', '\n']).to_owned())
}

fn checksum(header: &[u8], payload: &[u8]) -> u32 {
    header.iter().chain(payload).fold(0x811c_9dc5_u32, |hash, byte| {
        (hash ^ u32::from(*byte)).wrapping_mul(0x0100_0193)
    })
}
