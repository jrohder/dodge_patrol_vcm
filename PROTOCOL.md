# Nano ↔ ESP32 UART Protocol

Shared binary protocol between the Arduino Nano R4 extended-I/O processor
([`vcm_extended_io`](https://github.com/jrohder/vcm_extended_io)) and this
ESP32-S3 Vehicle Control Module.

**Single source of truth:** `src/proto/protocol.{h,cpp}` is shared verbatim
with the Nano firmware. The ESP32 is a pure *consumer* of that module — it
does not define its own wire layout.

| | |
|---|---|
| Baud | **460800** 8N1 |
| Protocol version | **2** (`vcmproto::PROTOCOL_VERSION`) |
| Timestamps | **microseconds** (`micros()`) |
| Telemetry payload | **63 bytes** |
| CRC | CRC16-CCITT, poly `0x1021`, init `0xFFFF` |
| Endianness | little-endian |
| Nominal rate | 100 Hz telemetry stream |

---

## Frame layout

Every packet uses the same generic header:

| Offset | Type | Field | Notes |
|--------|------|-------|-------|
| 0 | u8 | `sync1` | `0xAA` |
| 1 | u8 | `sync2` | `0x55` |
| 2 | u8 | `version` | must be `2` |
| 3 | u8 | `packetType` | see Packet types |
| 4 | u8 | `payloadLen` | bytes of payload (0…96) |
| 5 | u16 | `seq` | per-sender incrementing sequence |
| 7 | u32 | `timestampUs` | sender `micros()` |
| 11 | … | `payload` | `payloadLen` bytes |
| last 2 | u16 | `crc` | CRC over header + payload |

Total frame size = `11 + payloadLen + 2`.

---

## Packet types

| Value | Name | Direction | Payload |
|-------|------|-----------|---------|
| `0x01` | `PKT_TELEMETRY` | Nano → ESP32 | `TelemetryPayload` (63 B) |
| `0x02` | `PKT_EVENT` | Nano → ESP32 | `EventPayload` (8 B) |
| `0x03` | `PKT_ACK` | Nano → ESP32 | `AckPayload` (4 B) |
| `0x10` | `PKT_COMMAND` | ESP32 → Nano | `CommandPayload` (5 B) |

---

## Telemetry payload (63 bytes)

Nano → ESP32, fixed-rate snapshot. The Nano reports **raw engineering
values only** — no vehicle dimensions, gearing, or speed limits.

| Field | Type | Notes |
|-------|------|-------|
| `rcUs[6]` | u16×6 | RC pulse widths (µs); `0` = no signal |
| `rcValidMask` | u8 | bit *n* = channel *n+1* has a live signal |
| `left` | `WheelData` | see below |
| `right` | `WheelData` | see below |
| `adc[6]` | u16×6 | raw ADC: 0=battery, 1/2=motor-wire sense, 3–5 spare |
| `digitalIn` | u8 | raw digital input bits |
| `digitalOut` | u8 | current output states |
| `faultBits` | u16 | `FaultBits` |
| `statusBits` | u8 | `StatusBits` |
| `watchdogResets` | u8 | watchdog resets since power-on |
| `rxCrcErrors` | u16 | CRC errors the Nano saw on its RX |
| `loopMaxUs` | u16 | worst-case main loop time |
| `cpuLoadPct` | u8 | Nano CPU load estimate |
| `fwMajor` / `fwMinor` | u8 / u8 | Nano firmware version |
| `uptimeMs` | u32 | Nano uptime |
| `eventCount` | u8 | entries in the Nano event log |
| `reserved[2]` | u8×2 | |

### WheelData (9 bytes)

| Field | Type | Notes |
|-------|------|-------|
| `periodUs` | **u32** | time between hall pulses in **microseconds**; `0` = stopped |
| `count` | u32 | cumulative pulse counter |
| `direction` | i8 | −1 reverse, 0 stopped, +1 forward |

Frequency is derived on the ESP32:

```
freqHz = (periodUs > 0) ? 1e6 / periodUs : 0
```

### FaultBits

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `FB_HALL_LEFT` | left hall sensor fault |
| 1 | `FB_HALL_RIGHT` | right hall sensor fault |
| 2 | `FB_RC_LOST` | RC receiver signal lost |
| 3 | `FB_ADC_FAULT` | ADC acquisition fault |
| 4 | `FB_BROWNOUT` | Nano brownout detected |
| 5 | `FB_WATCHDOG_RESET` | last reset was a watchdog reset |
| 6 | `FB_LOOP_OVERRUN` | Nano main loop overrun |

### StatusBits

| Bit | Name | Meaning |
|-----|------|---------|
| 0 | `SB_RC_PRESENT` | at least one RC channel live |
| 1 | `SB_OUTPUTS_ENABLED` | digital outputs enabled |
| 2 | `SB_TIMER_ISR_OK` | 25 kHz sampling ISR healthy |

---

## Commands (ESP32 → Nano)

`CommandPayload`: `command` (u8) + `args[4]`.

| Value | Name | Args |
|-------|------|------|
| `0x00` | `CMD_HEARTBEAT` | none — watchdog keepalive |
| `0x01` | `CMD_SET_STREAM_RATE` | `args[0]` = telemetry rate (Hz) |
| `0x02` | `CMD_RESET_COUNTERS` | zero hall pulse counters |
| `0x03` | `CMD_SET_OUTPUTS` | `args[0]` = digital output bits |
| `0x04` | `CMD_IDENTIFY` | request ACK with firmware info |

The ESP32 sends heartbeats at 10 Hz. The Nano treats missing heartbeats as
a host-watchdog fault.

---

## Events & acknowledgements

- **`PKT_EVENT`**: Nano event-log entry (`timestampUs`, `code`, `data[3]`).
  The ESP32 rings the last 8 events and exposes them on Diagnostics.
- **`PKT_ACK`**: command acknowledgement (`command`, `status`, `ackSeq`).
  `status == 0` means OK.

---

## Parser

`vcmproto::Parser` is a streaming byte-fed parser with automatic
resynchronization on the `0xAA 0x55` sync sequence. It rejects frames with
wrong protocol version, oversized payloads, or CRC mismatches, and tracks:

* frames delivered
* CRC errors
* version errors
* length errors
* resyncs

The ESP32 `NanoLink` driver feeds UART bytes into this parser and adds
sequence-gap / packet-loss accounting plus a transport jitter estimate.

---

## Diagnostics exposed to the web UI

| Metric | Source |
|--------|--------|
| Packet rate / age | ESP32 link |
| Packets received / lost | ESP32 seq tracking |
| CRC / seq / resync / version errors | shared parser + link |
| Latency jitter (µs) | inter-arrival vs Nano timestamps |
| Protocol version | `PROTOCOL_VERSION` |
| Nano FW / uptime / CPU / loop max | telemetry payload |
| Watchdog resets / Nano RX CRC | telemetry payload |
| Fault bits / status bits | telemetry payload |
| Wheel period (µs) + derived frequency | telemetry payload |
| Event ring | `PKT_EVENT` frames |

---

## Compatibility

| ESP32 speaks | Nano speaks | Result |
|--------------|-------------|--------|
| v2 | v2 | OK |
| v2 | v1 (old) | version errors → `FLT_NANO_PROTOCOL` |
| v1 (old) | v2 | will not decode (use firmware ≥ 1.1.0) |

This VCM release requires Nano extended-I/O firmware that speaks protocol
**version 2**.
