use std::{
    env, fs,
    io::{self, Write},
    process::Command,
};

const CONFIG_OFFSET: &str = "0x3f0000";
const CONFIG_SIZE: usize = 4096;
const SSID_MAX_LEN: usize = 32;
const PASSWORD_MAX_LEN: usize = 63;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let port = env::args().nth(1).unwrap_or_else(|| "/dev/ttyACM0".to_owned());
    let ssid = prompt("Wi-Fi SSID: ")?;
    let password = prompt("Wi-Fi password (input is visible): ")?;
    let image = config_image(&ssid, &password)?;

    let path = env::temp_dir().join("fetchline-wifi-config.bin");
    fs::write(&path, image)?;
    let result = Command::new("espflash")
        .args(["write-bin", "--port", &port, "--chip", "esp32c3", "--before", "usb-reset", "--after", "hard-reset", "--non-interactive", CONFIG_OFFSET])
        .arg(&path)
        .status()?;
    fs::remove_file(path)?;
    if result.success() { Ok(()) } else { Err("Wi-Fi provisioning failed".into()) }
}

fn config_image(ssid: &str, password: &str) -> Result<[u8; CONFIG_SIZE], &'static str> {
    if ssid.is_empty() || ssid.len() > SSID_MAX_LEN || password.len() > PASSWORD_MAX_LEN {
        return Err("SSID must be 1–32 bytes and password 0–63 bytes");
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
    Ok(image)
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn creates_a_valid_wifi_configuration_image() {
        let image = config_image("test-network", "secret").unwrap();
        assert_eq!(&image[..4], b"FLWC");
        assert_eq!(image[4], 1);
        assert_eq!(image[5], 12);
        assert_eq!(image[6], 6);
        assert_eq!(&image[12..24], b"test-network");
        assert_eq!(&image[24..30], b"secret");
        assert!(image[30..].iter().all(|byte| *byte == 0xff));
        let stored_checksum = u32::from_le_bytes(image[8..12].try_into().unwrap());
        assert_eq!(stored_checksum, checksum(&image[..8], &image[12..30]));
    }

    #[test]
    fn rejects_an_empty_or_overlong_ssid() {
        assert!(config_image("", "password").is_err());
        assert!(config_image(&"s".repeat(SSID_MAX_LEN + 1), "password").is_err());
    }

    #[test]
    fn accepts_the_maximum_password_length_but_rejects_more() {
        assert!(config_image("ssid", &"p".repeat(PASSWORD_MAX_LEN)).is_ok());
        assert!(config_image("ssid", &"p".repeat(PASSWORD_MAX_LEN + 1)).is_err());
    }
}
