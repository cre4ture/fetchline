#!/usr/bin/env python3
"""Check the MCU's newest-controller-session-wins policy over WebSocket."""

import argparse
import base64
import json
import os
import socket
import struct
import sys


def read_exact(connection: socket.socket, length: int) -> bytes:
    data = bytearray()
    while len(data) < length:
        chunk = connection.recv(length - len(data))
        if not chunk:
            raise ConnectionError("connection closed")
        data.extend(chunk)
    return bytes(data)


def open_websocket(host: str, port: int) -> socket.socket:
    connection = socket.create_connection((host, port), timeout=2)
    connection.settimeout(2)
    nonce = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        "GET /rpc HTTP/1.1\r\n"
        f"Host: {host}:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {nonce}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    connection.sendall(request.encode("ascii"))
    response = bytearray()
    while b"\r\n\r\n" not in response:
        response.extend(connection.recv(1024))
    if not response.startswith(b"HTTP/1.1 101"):
        raise RuntimeError(f"WebSocket upgrade failed: {response.decode('ascii', 'replace')}")
    return connection


def send_text(connection: socket.socket, value: dict) -> None:
    payload = json.dumps(value, separators=(",", ":")).encode("utf-8")
    if len(payload) >= 126:
        raise ValueError("test message is unexpectedly large")
    mask = os.urandom(4)
    masked = bytes(byte ^ mask[index % 4] for index, byte in enumerate(payload))
    connection.sendall(bytes((0x81, 0x80 | len(payload))) + mask + masked)


def receive_frame(connection: socket.socket) -> tuple[int, bytes]:
    first, second = read_exact(connection, 2)
    opcode = first & 0x0F
    length = second & 0x7F
    if length == 126:
        length = struct.unpack("!H", read_exact(connection, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", read_exact(connection, 8))[0]
    payload = read_exact(connection, length)
    return opcode, payload


def ping(connection: socket.socket, request_id: int) -> None:
    send_text(
        connection,
        {"jsonrpc": "2.0", "id": request_id, "method": "system.ping", "params": {}},
    )
    opcode, payload = receive_frame(connection)
    if opcode != 1:
        raise RuntimeError(f"expected a text reply, received WebSocket opcode {opcode}")
    reply = json.loads(payload)
    if reply.get("id") != request_id or reply.get("result") != {"ready": True}:
        raise RuntimeError(f"unexpected ping reply: {reply}")


def first_session_was_closed(connection: socket.socket) -> bool:
    try:
        opcode, _ = receive_frame(connection)
    except (ConnectionError, OSError):
        return True
    except socket.timeout:
        return False
    return opcode == 8


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True, help="MCU IP address or hostname")
    parser.add_argument("--port", type=int, default=3333, help="controller API port (default: 3333)")
    arguments = parser.parse_args()

    first = open_websocket(arguments.host, arguments.port)
    try:
        ping(first, 1)
        second = open_websocket(arguments.host, arguments.port)
        try:
            ping(second, 2)
            if not first_session_was_closed(first):
                raise RuntimeError("the first controller session remained open after takeover")
        finally:
            second.close()
    finally:
        first.close()

    print("PASS: the newest controller session accepted commands and closed the prior session")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ConnectionError, OSError, RuntimeError, ValueError, json.JSONDecodeError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        raise SystemExit(1)
