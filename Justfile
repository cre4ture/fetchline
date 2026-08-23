set shell := ["bash", "-cu"]

host_target := `rustc -vV | sed -n 's/^host: //p'`

# List available recipes.
default:
    @just --list

# Build the ESP32-C3 firmware.
firmware-build:
    cargo build --release

# Flash the ESP32-C3 firmware and open its serial monitor.
firmware-flash:
    cargo run --release

# Start the LAN-accessible web control panel. Pass another address if needed.
host address="0.0.0.0:8787":
    cargo run --manifest-path host/Cargo.toml --target "{{host_target}}" --release --bin fetchline-host -- "{{address}}"

# Store Wi-Fi credentials in the reserved flash sector over USB.
provision-wifi port="/dev/ttyACM0":
    cargo run --manifest-path host/Cargo.toml --target "{{host_target}}" --bin provision-wifi -- "{{port}}"

# Run formatting, firmware, lint, and native-host checks.
check:
    cargo fmt --all --check
    cargo build --release
    cargo clippy --workspace --all-features -- -D warnings
    cargo check --manifest-path host/Cargo.toml --target "{{host_target}}"
