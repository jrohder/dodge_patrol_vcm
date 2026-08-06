/**
 * @file nano_protocol.h
 * @brief Binary UART protocol between the ESP32-S3 VCM and the Nano R4
 *        extended IO processor. Full specification in PROTOCOL.md.
 *
 * The Nano sends raw engineering values only (RC pulse widths, wheel
 * frequency/direction, pulse counters, raw ADC, fault flags). It knows
 * nothing about wheel diameter, gearing, speed limits or vehicle modes -
 * the ESP32 performs all higher-level interpretation.
 */
#pragma once

#include <cstdint>

namespace vcm {

constexpr uint16_t NANO_MAGIC = 0xAA55;        ///< little-endian on the wire: 55 AA
constexpr uint32_t NANO_BAUD = 460800;
constexpr uint8_t NANO_PROTOCOL_VERSION = 1;

/// Nano -> ESP32 telemetry, nominally every 10 ms (100 Hz).
struct __attribute__((packed)) NanoTelemetryPacket {
  uint16_t magic;      ///< NANO_MAGIC
  uint8_t version;     ///< protocol version
  uint8_t fwVersion;   ///< Nano firmware version (major<<4 | minor)
  uint16_t seq;        ///< incrementing sequence number
  uint32_t timestamp;  ///< Nano millis()
  uint16_t rcUs[6];    ///< RC channel pulse widths (us), 0 = no signal
  uint16_t leftFreqX10;   ///< left wheel hall frequency, Hz*10
  uint16_t rightFreqX10;  ///< right wheel hall frequency, Hz*10
  int8_t leftDir;         ///< -1 reverse, 0 stopped, +1 forward
  int8_t rightDir;
  uint32_t leftCount;   ///< cumulative hall pulses
  uint32_t rightCount;
  uint16_t adc[4];      ///< raw ADC: 0=battery divider, 1/2=motor wire sense, 3=spare
  uint16_t faultFlags;  ///< NanoFault bits
  uint16_t crc;         ///< CRC16-CCITT over all preceding bytes
};

/// Nano fault flag bits.
enum NanoFault : uint16_t {
  NF_HALL_LEFT = 1 << 0,
  NF_HALL_RIGHT = 1 << 1,
  NF_RC_LOST = 1 << 2,
  NF_ADC_FAULT = 1 << 3,
  NF_BROWNOUT = 1 << 4,
};

/// ESP32 -> Nano command types.
enum class NanoCommand : uint8_t {
  HEARTBEAT = 0,        ///< watchdog heartbeat; payload unused
  SET_STREAM_RATE = 1,  ///< payload[0] = telemetry rate in Hz
  RESET_COUNTERS = 2,   ///< zero the hall pulse counters
};

/// ESP32 -> Nano command packet, sent at up to 100 Hz.
struct __attribute__((packed)) NanoCommandPacket {
  uint16_t magic;    ///< NANO_MAGIC
  uint8_t version;   ///< protocol version
  uint8_t type;      ///< NanoCommand
  uint8_t payload[4];
  uint16_t crc;      ///< CRC16-CCITT over all preceding bytes
};

}  // namespace vcm
