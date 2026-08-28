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

# Start the panel with per-packet STS diagnostics in the host log.
host-debug address="0.0.0.0:8787":
    FETCHLINE_LOG=debug cargo run --manifest-path host/Cargo.toml --target "{{host_target}}" --release --bin fetchline-host -- "{{address}}"

# Testing only: enable the MCU raw STS listener on port 3334, then connect this host to it.
# The listener remains active after this host disconnects until debug.disableRawTunnel is sent.
host-debug-tunnel address="0.0.0.0:8787":
    FETCHLINE_LOG=debug cargo run --manifest-path host/Cargo.toml --target "{{host_target}}" --release --bin fetchline-host -- --debug-raw-tunnel "{{address}}"

# Show the most recent host diagnostics.
host-logs:
    tail -n 200 "${XDG_STATE_HOME:-$HOME/.local/state}/fetchline-host/fetchline-host.log"

# Store Wi-Fi credentials in the reserved flash sector over USB.
provision-wifi port="/dev/ttyACM0":
    cargo run --manifest-path host/Cargo.toml --target "{{host_target}}" --bin provision-wifi -- "{{port}}"

# Run formatting, firmware, lint, native firmware-logic, and host checks.
check:
    cargo fmt --all --check
    cargo build --release
    cargo clippy --workspace --all-features -- -D warnings
    cargo test --manifest-path firmware-tests/Cargo.toml --target "{{host_target}}"
    cargo check --manifest-path host/Cargo.toml --target "{{host_target}}"
    cargo test --manifest-path host/Cargo.toml --target "{{host_target}}"
