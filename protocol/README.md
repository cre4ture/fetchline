# Fetchline Controller API

The public controller API is [JSON-RPC 2.0](https://www.jsonrpc.org/specification)
in WebSocket text frames on `ws://<mcu>:3333/rpc`. The MCU terminates all STS
transactions locally; Wi-Fi carries only high-level commands and results.

Requests use numeric JSON-RPC IDs. The host uses the ID to discard late replies
instead of ever associating them with a newer servo command.

| Method | Named parameters | Result |
| --- | --- | --- |
| `system.ping` | none | `{ "ready": true }` |
| `motor.start` | `id`, `speed`, `acceleration`, `direction` | `{ "accepted": true }` |
| `motor.stop` | `id` | `{ "accepted": true }` |
| `servo.setPosition` | `id`, `position`, `acceleration`, `torqueLimit` | `{ "accepted": true }` |
| `servo.getPosition` | `id` | `{ "id": 5, "position": 1625 }` |
| `servo.getPositions` | `ids` (at most six IDs) | `{ "positions": [...] }` |
| `servo.scan` | `startId`, `endId` (1–255, inclusive) | `{ "ids": [2, 5, 7] }` |
| `debug.enableRawTunnel` | none | `{ "port": 3334, "active": true }` |
| `debug.disableRawTunnel` | none | `{ "active": false }` |

While the raw tunnel is active, normal motor and servo methods return the
JSON-RPC server error `-32010`. `system.ping` and the `debug.*` methods remain
available.

`debug.enableRawTunnel` opens the separate raw TCP endpoint `3334`. That port
remains open after a raw client disconnects and accepts a later client again.
Only `debug.disableRawTunnel` closes it, including any active raw connection.
Only one raw client is allowed at a time.

`servo.scan` probes every usable STS address in the inclusive range directly
on the MCU and returns the IDs that replied. The MCU keeps the 50 ms local STS
deadline for each address, so a full range can take roughly 15 seconds. STS
addresses `254` (broadcast) and `255` (invalid) are skipped even when they are
part of the requested 1–255 range; this avoids a broadcast reply collision.
