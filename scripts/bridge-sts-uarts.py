#!/usr/bin/env python3
"""Temporarily bridge an MCU UART to a FE-URT2 and log STS traffic.

The bridge connects two *separate* USB UART adapters in software:

    MCU UART adapter <-> this process <-> FE-URT2 adapter <-> STS bus

It does not generate STS commands itself.  Use the normal Fetchline control
panel while it is running, for example to scan servo ID 5.  Every forwarded
byte sequence is printed with its direction, which makes it possible to check
that the MCU transmits the expected STS packets and receives servo replies.

This is a diagnostic tool.  It forwards every command, including motion
commands, so only use it with hardware that is safe to command and stop it
explicitly when the test is complete.
"""

import argparse
import os
import select
import signal
import sys
import termios
import time


DEFAULT_BAUD = 1_000_000
ECHO_GRACE_SECONDS = 0.010


class LocalEchoFilter:
    """Drop a serial adapter's own TX echo without hiding a servo response.

    Some USB UART adapters expose bytes they just transmitted on their RX
    stream.  Forwarding those bytes back to the other adapter can create an
    endless loop.  STS replies differ from the request checksum/error byte, so
    a short exact-prefix check safely preserves a real reply even though it
    shares the header and ID with the command that caused it.
    """

    def __init__(self) -> None:
        self._expected = bytearray()
        self._candidate = bytearray()
        self._deadline = 0.0

    def wrote(self, payload: bytes) -> None:
        self._expected.extend(payload)
        self._deadline = time.monotonic() + ECHO_GRACE_SECONDS

    def filter(self, payload: bytes) -> bytes:
        forwarded = bytearray()
        for byte in payload:
            if not self._expected:
                forwarded.append(byte)
                continue

            self._candidate.append(byte)
            position = len(self._candidate) - 1
            if byte != self._expected[position]:
                # It looks like a real reply rather than an echoed request.
                forwarded.extend(self._candidate)
                self._candidate.clear()
                self._expected.clear()
                continue

            if len(self._candidate) == len(self._expected):
                # Exact local echo: discard it, then continue with any later
                # bytes in this same USB read.
                self._candidate.clear()
                self._expected.clear()

        return bytes(forwarded)

    def flush_expired(self) -> bytes:
        if not self._candidate or time.monotonic() < self._deadline:
            return b""
        forwarded = bytes(self._candidate)
        self._candidate.clear()
        self._expected.clear()
        return forwarded


def configure_port(descriptor: int, baud: int) -> None:
    if baud != DEFAULT_BAUD:
        raise ValueError("only 1000000 baud is supported for STS diagnostics")
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


def write_all(descriptor: int, payload: bytes) -> None:
    offset = 0
    while offset < len(payload):
        _, writable, _ = select.select([], [descriptor], [], 1)
        if not writable:
            raise TimeoutError("timed out forwarding UART data")
        offset += os.write(descriptor, payload[offset:])


def log_frame(direction: str, payload: bytes) -> None:
    print(f"{direction}: {payload.hex(' ')}", flush=True)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mcu-device",
        required=True,
        help="exact USB UART device wired to the MCU UART, preferably /dev/serial/by-id/...",
    )
    parser.add_argument(
        "--feurt-device",
        required=True,
        help="exact USB UART device for the FE-URT2, preferably /dev/serial/by-id/...",
    )
    parser.add_argument(
        "--duration-seconds",
        type=float,
        default=30,
        help="bridge lifetime; use 0 to run until Ctrl-C (default: 30)",
    )
    parser.add_argument("--quiet", action="store_true", help="forward without printing every frame")
    parsed = parser.parse_args()
    if parsed.duration_seconds < 0:
        parser.error("--duration-seconds must be zero or positive")
    if parsed.mcu_device == parsed.feurt_device:
        parser.error("--mcu-device and --feurt-device must be different devices")
    return parsed


def run(parsed: argparse.Namespace) -> tuple[int, int]:
    mcu_descriptor = os.open(parsed.mcu_device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    feurt_descriptor = os.open(parsed.feurt_device, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    mcu_to_feurt_frames = 0
    feurt_to_mcu_frames = 0
    running = True

    def stop(_signum: int, _frame: object) -> None:
        nonlocal running
        running = False

    previous_sigint = signal.signal(signal.SIGINT, stop)
    previous_sigterm = signal.signal(signal.SIGTERM, stop)
    try:
        configure_port(mcu_descriptor, DEFAULT_BAUD)
        configure_port(feurt_descriptor, DEFAULT_BAUD)
        mcu_echo = LocalEchoFilter()
        feurt_echo = LocalEchoFilter()
        deadline = None if parsed.duration_seconds == 0 else time.monotonic() + parsed.duration_seconds
        print(
            f"Bridging {parsed.mcu_device} <-> {parsed.feurt_device} at {DEFAULT_BAUD} baud. "
            "Use Ctrl-C to stop.",
            flush=True,
        )

        while running and (deadline is None or time.monotonic() < deadline):
            readable, _, _ = select.select([mcu_descriptor, feurt_descriptor], [], [], 0.002)
            for source in readable:
                payload = os.read(source, 512)
                if not payload:
                    continue
                if source == mcu_descriptor:
                    payload = mcu_echo.filter(payload)
                    if payload:
                        write_all(feurt_descriptor, payload)
                        feurt_echo.wrote(payload)
                        mcu_to_feurt_frames += 1
                        if not parsed.quiet:
                            log_frame("MCU -> FE-URT2", payload)
                else:
                    payload = feurt_echo.filter(payload)
                    if payload:
                        write_all(mcu_descriptor, payload)
                        mcu_echo.wrote(payload)
                        feurt_to_mcu_frames += 1
                        if not parsed.quiet:
                            log_frame("FE-URT2 -> MCU", payload)

            for payload, destination, echo_filter, direction in (
                (mcu_echo.flush_expired(), feurt_descriptor, feurt_echo, "MCU -> FE-URT2"),
                (feurt_echo.flush_expired(), mcu_descriptor, mcu_echo, "FE-URT2 -> MCU"),
            ):
                if payload:
                    write_all(destination, payload)
                    echo_filter.wrote(payload)
                    if direction == "MCU -> FE-URT2":
                        mcu_to_feurt_frames += 1
                    else:
                        feurt_to_mcu_frames += 1
                    if not parsed.quiet:
                        log_frame(direction, payload)
    finally:
        signal.signal(signal.SIGINT, previous_sigint)
        signal.signal(signal.SIGTERM, previous_sigterm)
        os.close(mcu_descriptor)
        os.close(feurt_descriptor)
    return mcu_to_feurt_frames, feurt_to_mcu_frames


def main() -> int:
    parsed = arguments()
    try:
        mcu_to_feurt, feurt_to_mcu = run(parsed)
    except (OSError, TimeoutError, ValueError, termios.error) as error:
        print(f"UART bridge failed: {error}", file=sys.stderr)
        return 2
    print(f"Forwarded frames: MCU -> FE-URT2={mcu_to_feurt}, FE-URT2 -> MCU={feurt_to_mcu}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
