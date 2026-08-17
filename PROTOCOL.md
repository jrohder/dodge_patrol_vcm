# Nano ↔ ESP32 UART Protocol — v1

> **Synced from** [`vcm_extended_io`](https://github.com/jrohder/vcm_extended_io).
> `src/proto/protocol.{h,cpp}` is **byte-identical** to the Nano shared module
> (`veio::proto`).

Binary, fixed-frame protocol between the Nano R4 extended I/O module and the
ESP32 Vehicle Control Module (VCM). The Nano only reports raw hardware
observations and executes simple output commands; every interpretation
(speed, gear, steering %, failsafe policy) belongs to the ESP32.

The reference implementation lives in [`src/proto/protocol.h`](src/proto/protocol.h)
/ [`protocol.cpp`](src/proto/protocol.cpp). It has **no Arduino
dependencies** — drop both files unchanged into either firmware to get the
frame encoder, the resynchronizing parser and all payload structs with
compile-time size checks.

## Physical layer

| Property   | Value                                                        |
|------------|--------------------------------------------------------------|
| Transport  | UART, 8N1                                                    |
| Nano pins  | D1 = TX1 (Nano → ESP32), D0 = RX1 (ESP32 → Nano)             |
| ESP32 pins | GPIO 1 = TX → Nano D0, GPIO 2 = RX ← Nano D1. **Not** DevKit silkscreen TX/RX (GPIO 43/44). |
| Baud       | 460800 default (`nano_r4` build), 921600 optional (`nano_r4_921600` build) |
| Logic level| 5 V on the Nano side — **use a 1 kΩ / 2 kΩ divider (or a level shifter) toward the 3.3 V ESP32 RX** |

## Frame format

All multi-byte fields are **little-endian**.

| Offset | Size | Field                                    |
|--------|------|------------------------------------------|
| 0      | 1    | Sync 0 = `0x55`                          |
| 1      | 1    | Sync 1 = `0xAA`                          |
| 2      | 1    | Protocol version = `1`                   |
| 3      | 1    | Packet type                              |
| 4      | 1    | Payload length `N` (0…96)                |
| 5      | 2    | Sequence number (per sender, wraps)      |
| 7      | 4    | Sender timestamp, µs since boot (wraps ≈71 min) |
| 11     | N    | Payload                                  |
| 11+N   | 2    | CRC16-CCITT-FALSE over bytes 2…10+N      |

CRC16-CCITT-FALSE: poly `0x1021`, init `0xFFFF`, no reflection
(check value: `"123456789"` → `0x29B1`). The CRC covers the header after the
sync bytes plus the payload.

Every packet the Nano sends carries its µs timestamp and a monotonically
increasing sequence number, so the ESP32 can compute link latency and detect
missed packets.

## Packet types

| ID    | Direction    | Name         | Payload                    |
|-------|--------------|--------------|----------------------------|
| 0x01  | Nano → ESP32 | TELEMETRY    | `TelemetryPayload` (63 B), 100 Hz (200 Hz in TEST_MODE) |
| 0x02  | Nano → ESP32 | HEARTBEAT    | `HeartbeatPayload` (8 B), 1 Hz |
| 0x03  | Nano → ESP32 | DIAGNOSTIC   | `DiagnosticPayload` (23 B), 10 Hz while diagnostics active |
| 0x04  | Nano → ESP32 | COMMAND_ACK  | `CommandAckPayload` (4 B), per command |
| 0x05  | Nano → ESP32 | FAULT        | `FaultPayload` (4 B), on fault change |
| 0x06  | Nano → ESP32 | BOOT         | `VersionPayload` (31 B), once at startup |
| 0x07  | Nano → ESP32 | VERSION      | `VersionPayload` (31 B), on request |
| 0x08  | Nano → ESP32 | LOG          | reserved (event log dump)  |
| 0x10–0x15 | Nano → ESP32 | reserved | CAN, GPS, extra Hall, buttons, temperature, lighting controller |
| 0x40  | ESP32 → Nano | COMMAND      | `CommandPayload` (5 B)     |
| 0x50–0x53 | ESP32 → Nano | reserved | firmware-update transport (BEGIN / DATA / VERIFY / COMMIT) |

## TELEMETRY payload (63 bytes)

One fixed-length, time-consistent snapshot of every input and output, sent
every 10 ms. Field order matches `TelemetryPayload`:

| Field            | Type        | Meaning                                            |
|------------------|-------------|----------------------------------------------------|
| `rcPulseUs[6]`   | u16 × 6     | raw pulse width CH1…CH6 in µs (0 = never received) |
| `rcValidMask`    | u8          | bit n = CH(n+1) valid (in range + fresh)           |
| `rcAgeMs[6]`     | u8 × 6      | ms since the last valid pulse, clamped to 255      |
| `left`, `right`  | `WheelData` × 2 | see below                                      |
| `motorSenseARaw/Filt` | u16 × 2 | A0 raw average / EMA-filtered, 14-bit ADC counts  |
| `motorSenseBRaw/Filt` | u16 × 2 | A1 raw average / EMA-filtered                     |
| `batteryRaw/Filt`| u16 × 2     | A3 raw average / EMA-filtered                      |
| `batteryMin/Max` | u16 × 2     | filtered min/max since boot or counter reset       |
| `outputState`    | u8          | bits 0-1 commanded mode (0 OFF / 1 ON / 2 FLASH), bit 7 current pin level |
| `systemState`    | u8          | 0 BOOT, 1 INITIALIZING, 2 READY, 3 DIAGNOSTIC, 4 TEST_MODE, 5 FAULT |
| `faultFlags`     | u16         | fault bitfield, see below                          |

### WheelData (12 bytes per wheel)

| Field        | Type | Meaning                                                    |
|--------------|------|------------------------------------------------------------|
| `pulseCount` | i32  | signed quadrature transition count since boot/reset        |
| `freqHzX10`  | u16  | quadrature edge frequency in 0.1 Hz units (0 = stopped)    |
| `direction`  | i8   | +1 = A leads B, −1 = B leads A, 0 = stopped/unknown        |
| `periodUs`   | u32  | µs between the two most recent transitions (0 = stopped)   |
| `fault`      | u8   | bit0 = illegal transitions latched, bit1 = edge timeout (stopped) |

Frequency is derived from the period between edges (`f = 1/T`), not from
counting pulses per second — this stays smooth at very low speed. Note
`periodUs` was widened to u32 relative to the original sketch spec so that
slow wheel motion (periods > 65 ms) is still representable. The Nano knows
nothing about wheel geometry: converting counts/frequency to speed is the
ESP32's job.

### Fault flags (u16)

| Bit | Name               | Set when                                          |
|-----|--------------------|---------------------------------------------------|
| 0   | RC_SIGNAL_LOST     | all six RC channels are currently invalid         |
| 1   | UART_CRC (latched) | CRC error(s) seen on the command link             |
| 2   | HALL_LEFT (latched)| ≥8 illegal quadrature transitions within 1 s      |
| 3   | HALL_RIGHT (latched)| same, right wheel                                |
| 4   | ADC                | ADC acquisition failure/overrun                   |
| 5   | WATCHDOG_RESET (latched) | last reset was caused by the hardware WDT   |
| 6   | OUTPUT             | reserved (no output feedback fitted)              |
| 7   | INTERNAL (latched) | init/internal error (module enters FAULT state)   |
| 8   | SCHEDULER_OVERRUN (latched) | a task missed its deadline               |
| 9   | SAMPLER_OVERRUN (latched)   | the 25 kHz sampler ISR overran           |
| 10–15 | —                | reserved                                          |

Latched bits stay set until `CMD_CLEAR_FAULTS`. The Nano never reacts to
faults beyond reporting them and keeping outputs in the safe OFF default.

## COMMAND payload (5 bytes, ESP32 → Nano)

`commandId` + 4 argument bytes (zero-fill unused). Every command is answered
with COMMAND_ACK carrying the command id, a status
(0 OK / 1 unknown / 2 bad args / 3 rejected) and the sequence number of the
command frame being acknowledged.

| ID   | Name              | Arguments                                              |
|------|-------------------|--------------------------------------------------------|
| 0x01 | SET_OUTPUT        | `[output, mode, halfPeriodLo, halfPeriodHi]` — output 0 = lights/siren; mode 0 OFF / 1 ON / 2 FLASH; FLASH half-period in ms (0 → 250 ms default) |
| 0x02 | OUTPUT_TEST       | `[output]` — three 100 ms flashes, then restores the commanded mode |
| 0x03 | RESET_COUNTERS    | zeroes pulse counts, min/max, glitch and diag counters  |
| 0x04 | START_DIAGNOSTICS | `[seconds]` — 0 = single DIAGNOSTIC packet; >0 = DIAGNOSTIC state streaming at 10 Hz for that many seconds |
| 0x05 | SET_TEST_MODE     | `[1|0]` — enable/disable TEST_MODE (telemetry at 200 Hz for calibration) |
| 0x06 | REQUEST_VERSION   | replies with VERSION                                    |
| 0x07 | PING              | ack only                                                |
| 0x08 | CLEAR_FAULTS      | clears latched fault bits                               |
| 0x30–0x3F | reserved     | firmware-update commands (future ESP32-driven update)   |

### Output failsafe

If the ESP32 has commanded any output ON/FLASH and then stops sending valid
frames for **1000 ms**, the Nano reverts all outputs to OFF and logs a
failsafe event. This is a generic hardware-safe default, not a vehicle
decision; normal ESP32 operation (100 Hz command/telemetry traffic or even
occasional pings) keeps the link alive.

## HEARTBEAT payload (8 bytes)

`uptimeMs` u32, `systemState` u8, `faultFlags` u16, `protocolVersion` u8.
Sent at 1 Hz; the ESP32 should treat a missing heartbeat (plus missing
telemetry) as a Nano outage.

## DIAGNOSTIC payload (23 bytes)

Loop time avg/max (µs), CPU load %, per-subsystem 2-bit health word
(RC, Hall L/R, ADC, UART, scheduler, outputs, system — 0 OK / 1 WARNING /
2 FAULT), scheduler/sampler overruns, UART CRC/frame errors, TX drops,
commands received, RC glitches, free RAM bytes.

## VERSION / BOOT payload (31 bytes)

Firmware version (major/minor/patch), protocol version, 32-bit short git
hash, build date and time strings, board id, boot reason
(1 = power-on/external, 2 = watchdog).

## Link budget

Telemetry frame = 63 B payload + 13 B framing = 76 B → 760 bits with 8N1.

| Rate            | 460800 baud | 921600 baud |
|-----------------|-------------|-------------|
| 100 Hz telemetry| 16.5 %      | 8.2 %       |
| 200 Hz test mode| 33 %        | 16.5 %      |

Both leave ample margin for heartbeat/diagnostic/ack traffic. See the README
for the baud-rate selection procedure on real hardware.
