#!/usr/bin/env python3
"""Find STS servos through a directly connected FE-URT-2 USB adapter.

The scanner sends only per-servo STS PING packets. It never uses the broadcast
address and never writes to servo registers, so it cannot move or reconfigure a
servo. The caller must select the exact serial device explicitly.
"""

import argparse
import os
import select
import sys
import termios
import time


MIN_SERVO_ID = 1
MAX_SERVO_ID = 253
STS_HEADER = b"\xff\xff"
STS_PING = 0x01


def checksum(values: bytes) -> int:
    """Return the STS checksum for packet bytes excluding its header/checksum."""
    return (~sum(values)) & 0xFF


def ping_packet(servo_id: int) -> bytes:
    body = bytes((servo_id, 2, STS_PING))
    return STS_HEADER + body + bytes((checksum(body),))


def write_all(descriptor: int, payload: bytes) -> None:
    """Write one complete packet, handling a partial non-blocking write."""
    offset = 0
    while offset < len(payload):
        _, writable, _ = select.select([], [descriptor], [], 1)
        if not writable:
            raise TimeoutError("timed out writing an STS ping")
        offset += os.write(descriptor, payload[offset:])


def extract_status(buffer: bytearray, expected_id: int) -> int | None:
    """Return the STS status-error byte for the expected ID, if a frame is complete."""
    while True:
        header_offset = buffer.find(STS_HEADER)
        if header_offset < 0:
            buffer[:] = buffer[-1:] if buffer.endswith(b"\xff") else b""
            return None
        if header_offset:
            del buffer[:header_offset]
        if len(buffer) < 4:
            return None

        length = buffer[3]
        if not 2 <= length <= 66:
            del buffer[0]
            continue
        frame_length = 4 + length
        if len(buffer) < frame_length:
            return None

        frame = bytes(buffer[:frame_length])
        del buffer[:frame_length]
        if frame[2] != expected_id or checksum(frame[2:-1]) != frame[-1]:
            continue
        return frame[4]


def read_status(descriptor: int, expected_id: int, timeout_seconds: float) -> int | None:
    """Wait up to the local deadline for a valid status response."""
    buffer = bytearray()
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        readable, _, _ = select.select([descriptor], [], [], deadline - time.monotonic())
        if not readable:
            break
        chunk = os.read(descriptor, 256)
        if not chunk:
            continue
        buffer.extend(chunk)
        status_error = extract_status(buffer, expected_id)
        if status_error is not None:
            return status_error
    return None


def drain_late_bytes(descriptor: int, quiet_seconds: float = 0.002) -> None:
    """Discard trailing data so it cannot be associated with the next ID."""
    deadline = time.monotonic() + quiet_seconds
    while time.monotonic() < deadline:
        readable, _, _ = select.select([descriptor], [], [], deadline - time.monotonic())
        if not readable:
            return
        os.read(descriptor, 256)


def configure_serial_port(descriptor: int) -> None:
    """Configure the FE-URT-2's STS link: 1,000,000 baud, 8 data bits, no parity, 1 stop."""
    attributes = termios.tcgetattr(descriptor)
    attributes[0] = termios.IGNPAR
    attributes[1] = 0
    attributes[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attributes[3] = 0
    attributes[4] = termios.B1000000
    attributes[5] = termios.B1000000
    attributes[6][termios.VMIN] = 0
    attributes[6][termios.VTIME] = 0
    termios.tcsetattr(descriptor, termios.TCSANOW, attributes)
    termios.tcflush(descriptor, termios.TCIOFLUSH)


def scan(device: str, start_id: int, end_id: int, timeout_seconds: float) -> tuple[list[int], list[tuple[int, int]]]:
    descriptor = os.open(device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        termios.tcflush(descriptor, termios.TCIOFLUSH)
        configure_serial_port(descriptor)
        found_ids: list[int] = []
        status_errors: list[tuple[int, int]] = []
        for servo_id in range(start_id, end_id + 1):
            write_all(descriptor, ping_packet(servo_id))
            status_error = read_status(descriptor, servo_id, timeout_seconds)
            if status_error == 0:
                found_ids.append(servo_id)
            elif status_error is not None:
                status_errors.append((servo_id, status_error))
            drain_late_bytes(descriptor)
        return found_ids, status_errors
    finally:
        os.close(descriptor)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", required=True, help="exact FE-URT-2 serial device, preferably /dev/serial/by-id/...")
    parser.add_argument("--start-id", type=int, default=MIN_SERVO_ID, help="first STS ID to probe (default: 1)")
    parser.add_argument("--end-id", type=int, default=MAX_SERVO_ID, help="last STS ID to probe (default: 253)")
    parser.add_argument("--timeout-ms", type=int, default=50, help="local reply deadline per ID in milliseconds (default: 50)")
    parsed = parser.parse_args()
    if not MIN_SERVO_ID <= parsed.start_id <= MAX_SERVO_ID:
        parser.error(f"--start-id must be between {MIN_SERVO_ID} and {MAX_SERVO_ID}")
    if not MIN_SERVO_ID <= parsed.end_id <= MAX_SERVO_ID:
        parser.error(f"--end-id must be between {MIN_SERVO_ID} and {MAX_SERVO_ID}")
    if parsed.start_id > parsed.end_id:
        parser.error("--start-id must not exceed --end-id")
    if parsed.timeout_ms <= 0:
        parser.error("--timeout-ms must be positive")
    return parsed


def main() -> int:
    parsed = arguments()
    try:
        found_ids, status_errors = scan(
            parsed.device,
            parsed.start_id,
            parsed.end_id,
            parsed.timeout_ms / 1000,
        )
    except (OSError, TimeoutError, termios.error) as error:
        print(f"STS scan failed: {error}", file=sys.stderr)
        return 2

    print(f"Found servo IDs: {', '.join(map(str, found_ids)) if found_ids else 'none'}")
    if status_errors:
        details = ", ".join(f"{servo_id} (status 0x{error:02x})" for servo_id, error in status_errors)
        print(f"Responding IDs with STS status errors: {details}")
    print(f"Scanned IDs {parsed.start_id}-{parsed.end_id} at 1000000 baud, 8N1.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
