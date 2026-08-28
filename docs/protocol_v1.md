# Protocol v1

The host runtime, mock MCU, and STM32 firmware share this packet format.

## Command Packet

| Field | Size | Description |
| --- | ---: | --- |
| header | 1 byte | Constant `0xA5` |
| version | 1 byte | Protocol version, currently `1` |
| sequence | 1 byte | Incrementing sequence number, wraps at 255 |
| command_id | 1 byte | `0 STOP`, `1 TURN_LEFT`, `2 TURN_RIGHT`, `3 FORWARD`, `4 HEARTBEAT`, `5 READ_ENCODERS`, `6 SET_WHEEL_SPEED` |
| payload_len | 1 byte | `0` for commands 0–5; `4` for `SET_WHEEL_SPEED` |
| payload | N bytes | Command-specific payload |
| checksum | 1 byte | Additive sum of all previous bytes modulo 256 |

`SET_WHEEL_SPEED` uses a 10-byte command packet. Its four-byte payload is:

| Payload offset | Field | Size | Description |
| ---: | --- | ---: | --- |
| 0–1 | left_counts_per_s | 2 bytes | Signed little-endian int16 |
| 2–3 | right_counts_per_s | 2 bytes | Signed little-endian int16 |

The payload values are validated against the firmware counts-per-second limit
before they are accepted.

## State Packet

The MCU returns a fixed 10-byte state packet after each accepted or rejected
motion command. `READ_ENCODERS` uses the same command packet shape but returns
an encoder packet instead of a state packet.

| Offset | Field | Size | Description |
| ---: | --- | ---: | --- |
| 0 | header | 1 byte | Constant `0xA5` |
| 1 | version | 1 byte | Protocol version, currently `1` |
| 2 | sequence | 1 byte | Echoes the host command sequence |
| 3 | packet_type | 1 byte | Constant `0x80` for MCU state |
| 4 | state | 1 byte | MCU state enum |
| 5 | error | 1 byte | Error enum |
| 6 | motor_intent | 1 byte | `0 STOP`, `1 FORWARD`, `2 TURN_LEFT`, `3 TURN_RIGHT` |
| 7 | uptime_lo | 1 byte | Low byte of uptime counter |
| 8 | uptime_hi | 1 byte | High byte of uptime counter |
| 9 | checksum | 1 byte | Sum of bytes 0-8 modulo 256 |

## Encoder Packet

`READ_ENCODERS` returns a fixed 14-byte snapshot packet. It is separate from the
state packet so existing serial, motor intent, and fault certification tools do
not change packet length.

| Offset | Field | Size | Description |
| ---: | --- | ---: | --- |
| 0 | header | 1 byte | Constant `0xA5` |
| 1 | version | 1 byte | Protocol version, currently `1` |
| 2 | sequence | 1 byte | Echoes the `READ_ENCODERS` command sequence |
| 3 | packet_type | 1 byte | Constant `0x81` for encoder snapshot |
| 4-7 | left_count | 4 bytes | Signed little-endian left encoder count |
| 8-11 | right_count | 4 bytes | Signed little-endian right encoder count |
| 12 | flags | 1 byte | Reserved; currently `0` |
| 13 | checksum | 1 byte | Sum of bytes 0-12 modulo 256 |

## MCU States

| State | Meaning |
| --- | --- |
| `IDLE` | No valid command stream yet |
| `ARMED` | Heartbeat seen, ready for command stream |
| `TRACKING` | Executing accepted motion primitive |
| `SAFE_STOP` | Command stopped by host, timeout, or validation failure |
| `FAULT` | Forced or unrecoverable controller-side fault |

## Motor Intent

`motor_intent` is dry-run telemetry. It does not drive a motor pin by itself;
it tells the host which motion primitive the MCU state machine would command
once a motor driver and power stage are installed. Any stop, timeout, invalid
packet, or sequence error must report `STOP`.
