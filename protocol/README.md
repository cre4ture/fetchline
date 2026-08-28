# Fetchline Controller API

The public controller API is [JSON-RPC 2.0](https://www.jsonrpc.org/specification)
in WebSocket text frames on `ws://<mcu>:3333/rpc`. The MCU terminates all STS
transactions locally; Wi-Fi carries only high-level commands and results.

Requests use numeric JSON-RPC IDs. The host uses the ID to discard late replies
instead of ever associating them with a newer servo command.

## Controller connection policy

The MCU reserves three TCP/WebSocket transport slots. One remains listening
while a newly connected host takes over, even if a prior peer vanished without
a clean TCP close. Once its WebSocket upgrade succeeds, the newest session
becomes the only session allowed to execute controller commands; prior sessions
are closed. The extra transport slots do not add controller authority: at most
one STS command is executed at a time.

The MCU serializes takeovers until a retiring socket is listening again, and
the host retries this short TCP transition automatically. This keeps a listener
available instead of exposing a reconnect race to the control UI.

| Method | Named parameters | Result |
| --- | --- | --- |
| `system.ping` | none | `{ "ready": true }` |
| `motor.start` | `id`, `speed`, `acceleration`, `direction` | `{ "accepted": true }` |
| `motor.stop` | `id` | `{ "accepted": true }` |
| `servo.setPosition` | `id`, `position`, `acceleration`, `torqueLimit` | `{ "accepted": true }` |
| `servo.getPosition` | `id` | `{ "id": 5, "position": 1625 }` |
| `servo.getPositions` | `ids` (at most six IDs) | `{ "positions": [...] }` |
| `servo.scan` | `startId`, `endId` (1–253, inclusive) | `{ "ids": [2, 5, 7] }` |
| `servo.setId` | `currentId`, `newId` (distinct IDs from 1–253) | `{ "previousId": 5, "newId": 6 }` |
| `debug.enableRawTunnel` | none | `{ "port": 3334, "active": true }` |
| `debug.disableRawTunnel` | none | `{ "active": false }` |

While the raw tunnel is active, normal motor and servo methods return the
JSON-RPC server error `-32010`. `system.ping` and the `debug.*` methods remain
available.

`debug.enableRawTunnel` opens the separate raw TCP endpoint `3334`. That port
remains open after a raw client disconnects and accepts a later client again.
Only `debug.disableRawTunnel` closes it, including any active raw connection.
Only one raw client is allowed at a time.

`servo.scan` probes every STS address in the inclusive 1–253 range directly on
the MCU and returns the IDs that replied. The MCU keeps the 50 ms local STS
deadline for each address, so a full range can take roughly 15 seconds. Address
`254` is the STS broadcast address and `255` is invalid, so the API rejects
both rather than risking a broadcast reply collision.

`servo.setId` first PINGs `currentId`, then makes sure `newId` does not answer.
It writes the STS persistent ID register and PINGs `newId` to confirm the
change. If the target already answers, the command returns JSON-RPC server
error `-32005` and does not write to the bus. A successful ID change does not
change the host's configured position-control IDs automatically.
