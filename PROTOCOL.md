# Nano ↔ ESP32 UART Protocol

Version: **1** (`protocol_version` byte lets both firmwares evolve
independently; the ESP32 raises fault `COM-003` on a mismatch).

- UART: 460800 baud, 8N1. ESP32 GPIO44 = RX, GPIO43 = TX.
- Byte order: little-endian, packed structs.
- Integrity: CRC16-CCITT (poly `0x1021`, init `0xFFFF`) over all bytes
  before the CRC field.
- The ESP32 tracks packets received, packets lost (sequence gaps), CRC
  errors, sequence errors and timeouts; all visible on the Diagnostics page.

## Nano → ESP32: telemetry packet (every 10 ms, 44 bytes)

| Offset | Type | Field | Description |
| --- | --- | --- | --- |
| 0 | u16 | magic | `0xAA55` (wire bytes `55 AA`) |
| 2 | u8 | version | protocol version (1) |
| 3 | u8 | fwVersion | Nano firmware `major<<4 \| minor` |
| 4 | u16 | seq | incrementing sequence number |
| 6 | u32 | timestamp | Nano `millis()` |
| 10 | u16[6] | rcUs | RC channel pulse widths (µs), 0 = no signal |
| 22 | u16 | leftFreqX10 | left wheel hall frequency, Hz × 10 |
| 24 | u16 | rightFreqX10 | right wheel hall frequency, Hz × 10 |
| 26 | i8 | leftDir | −1 reverse / 0 stopped / +1 forward |
| 27 | i8 | rightDir | |
| 28 | u32 | leftCount | cumulative hall pulses |
| 32 | u32 | rightCount | |
| 36 | u16[4] | adc | raw ADC: 0 battery divider, 1/2 motor-wire sense A/B, 3 spare |
| 44 | u16 | faultFlags | see below |
| 46 | u16 | crc | CRC16-CCITT of bytes 0–45 |

> Struct definition: `src/drivers/uart/nano_protocol.h`
> (`NanoTelemetryPacket`, `sizeof` = 48 with the trailing CRC).

### Fault flags

| Bit | Name | Meaning |
| --- | --- | --- |
| 0 | `NF_HALL_LEFT` | left hall sensor fault |
| 1 | `NF_HALL_RIGHT` | right hall sensor fault |
| 2 | `NF_RC_LOST` | RC receiver signal lost |
| 3 | `NF_ADC_FAULT` | ADC acquisition fault |
| 4 | `NF_BROWNOUT` | Nano brownout detected |

## ESP32 → Nano: command packet (10 bytes)

| Offset | Type | Field | Description |
| --- | --- | --- | --- |
| 0 | u16 | magic | `0xAA55` |
| 2 | u8 | version | protocol version |
| 3 | u8 | type | command type |
| 4 | u8[4] | payload | command-specific |
| 8 | u16 | crc | CRC16-CCITT of bytes 0–7 |

### Command types

| Type | Name | Payload |
| --- | --- | --- |
| 0 | `HEARTBEAT` | none — watchdog keepalive, sent at 10 Hz |
| 1 | `SET_STREAM_RATE` | `payload[0]` = telemetry rate in Hz |
| 2 | `RESET_COUNTERS` | none — zero the hall pulse counters |

## Framing / recovery

The ESP32 resynchronizes on the `55 AA` magic sequence, so the link
self-recovers from partial packets or noise. Packets failing CRC are
counted and dropped. If no valid packet arrives within `safety.nano_timeout`
(default 100 ms) the ESP32 raises `COM-001` and commands propulsion safe.

## Design rule

The Nano sends **raw engineering values** only. All interpretation
(speed, distance, RC mapping, pedal decoding, limits) happens on the ESP32
so that vehicle behavior stays configurable without touching the Nano.
