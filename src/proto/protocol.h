/**
 * @file protocol.h
 * @brief Shared VCM extended-I/O UART protocol (version 2).
 *
 * SHARED MODULE: this file (and protocol.cpp) is the single protocol
 * definition consumed by BOTH the Nano R4 extended-I/O firmware
 * (jrohder/vcm_extended_io) and this ESP32 VCM. It is intentionally
 * portable C++ (no Arduino/ESP-IDF dependencies) so it can be dropped
 * into either project unchanged and unit-tested on the host.
 *
 * If the copy in vcm_extended_io changes, overwrite this directory with
 * it verbatim - nothing else in the ESP32 firmware defines wire layout.
 *
 * Frame layout (little-endian, packed):
 *
 *   offset 0   uint8   sync1        0xAA
 *   offset 1   uint8   sync2        0x55
 *   offset 2   uint8   version      PROTOCOL_VERSION (2)
 *   offset 3   uint8   packetType   PacketType
 *   offset 4   uint8   payloadLen   bytes of payload
 *   offset 5   uint16  seq          per-sender incrementing sequence
 *   offset 7   uint32  timestampUs  sender micros()
 *   offset 11  ...     payload      payloadLen bytes
 *   last 2     uint16  crc          CRC16-CCITT over header+payload
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace vcmproto {

constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
constexpr uint8_t PROTOCOL_VERSION = 2;
constexpr uint32_t DEFAULT_BAUD = 460800;

constexpr size_t HEADER_SIZE = 11;
constexpr size_t CRC_SIZE = 2;
constexpr size_t MAX_PAYLOAD = 96;
constexpr size_t MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + CRC_SIZE;

/// CRC16-CCITT, poly 0x1021, init 0xFFFF.
uint16_t crc16(const uint8_t* data, size_t len);

enum PacketType : uint8_t {
  PKT_TELEMETRY = 0x01,  ///< Nano -> ESP32, fixed-rate snapshot
  PKT_EVENT = 0x02,      ///< Nano -> ESP32, event log entry
  PKT_ACK = 0x03,        ///< Nano -> ESP32, command acknowledgement
  PKT_COMMAND = 0x10,    ///< ESP32 -> Nano
};

#pragma pack(push, 1)

struct Header {
  uint8_t sync1;
  uint8_t sync2;
  uint8_t version;
  uint8_t packetType;
  uint8_t payloadLen;
  uint16_t seq;
  uint32_t timestampUs;  ///< sender micros() - MICROSECONDS, not millis
};
static_assert(sizeof(Header) == HEADER_SIZE, "header layout");

/// Per-wheel hall data. periodUs is the time between hall pulses in
/// microseconds (uint32); 0 means no pulses / stopped. Frequency and
/// speed are derived by the consumer: freqHz = 1e6 / periodUs.
struct WheelData {
  uint32_t periodUs;
  uint32_t count;     ///< cumulative pulse counter
  int8_t direction;   ///< -1 reverse, 0 stopped, +1 forward
};
static_assert(sizeof(WheelData) == 9, "wheel layout");

/// Nano fault bits (TelemetryPayload::faultBits).
enum FaultBits : uint16_t {
  FB_HALL_LEFT = 1u << 0,
  FB_HALL_RIGHT = 1u << 1,
  FB_RC_LOST = 1u << 2,
  FB_ADC_FAULT = 1u << 3,
  FB_BROWNOUT = 1u << 4,
  FB_WATCHDOG_RESET = 1u << 5,   ///< last reset was a watchdog reset
  FB_LOOP_OVERRUN = 1u << 6,
};

/// Nano status bits (TelemetryPayload::statusBits).
enum StatusBits : uint8_t {
  SB_RC_PRESENT = 1u << 0,
  SB_OUTPUTS_ENABLED = 1u << 1,
  SB_TIMER_ISR_OK = 1u << 2,  ///< 25 kHz sampling ISR healthy
};

/// Fixed telemetry snapshot, streamed at the configured rate (100 Hz
/// default). 63 bytes.
struct TelemetryPayload {
  uint16_t rcUs[6];       ///< RC pulse widths (us); 0 = no signal
  uint8_t rcValidMask;    ///< bit n = channel n+1 has a live signal
  WheelData left;
  WheelData right;
  uint16_t adc[6];        ///< raw ADC: 0 battery, 1/2 motor-wire sense A/B, 3-5 spare
  uint8_t digitalIn;      ///< raw digital input bits
  uint8_t digitalOut;     ///< current output states
  uint16_t faultBits;     ///< FaultBits
  uint8_t statusBits;     ///< StatusBits
  uint8_t watchdogResets; ///< watchdog resets since power-on
  uint16_t rxCrcErrors;   ///< CRC errors seen by the Nano on its RX side
  uint16_t loopMaxUs;     ///< worst-case main loop time
  uint8_t cpuLoadPct;     ///< Nano CPU load estimate
  uint8_t fwMajor;        ///< Nano firmware version
  uint8_t fwMinor;
  uint32_t uptimeMs;
  uint8_t eventCount;     ///< entries in the Nano event log
  uint8_t reserved[2];
};
static_assert(sizeof(TelemetryPayload) == 63, "telemetry payload is 63 bytes");

/// Event log entry (PKT_EVENT).
struct EventPayload {
  uint32_t timestampUs;
  uint8_t code;
  uint8_t data[3];
};
static_assert(sizeof(EventPayload) == 8, "event layout");

/// Commands (PKT_COMMAND payload).
enum Command : uint8_t {
  CMD_HEARTBEAT = 0x00,        ///< watchdog keepalive; no args
  CMD_SET_STREAM_RATE = 0x01,  ///< args[0] = telemetry rate (Hz)
  CMD_RESET_COUNTERS = 0x02,   ///< zero hall pulse counters
  CMD_SET_OUTPUTS = 0x03,      ///< args[0] = digital output bits
  CMD_IDENTIFY = 0x04,         ///< request an ACK with firmware info
};

struct CommandPayload {
  uint8_t command;  ///< Command
  uint8_t args[4];
};
static_assert(sizeof(CommandPayload) == 5, "command layout");

/// Acknowledgement (PKT_ACK payload).
struct AckPayload {
  uint8_t command;   ///< command being acknowledged
  uint8_t status;    ///< 0 = OK, nonzero = error code
  uint16_t ackSeq;   ///< seq of the acknowledged command frame
};
static_assert(sizeof(AckPayload) == 4, "ack layout");

#pragma pack(pop)

/// A fully received, validated frame.
struct Packet {
  Header header;
  uint8_t payload[MAX_PAYLOAD];

  const TelemetryPayload* asTelemetry() const {
    return (header.packetType == PKT_TELEMETRY &&
            header.payloadLen == sizeof(TelemetryPayload))
               ? reinterpret_cast<const TelemetryPayload*>(payload)
               : nullptr;
  }
  const EventPayload* asEvent() const {
    return (header.packetType == PKT_EVENT &&
            header.payloadLen == sizeof(EventPayload))
               ? reinterpret_cast<const EventPayload*>(payload)
               : nullptr;
  }
  const AckPayload* asAck() const {
    return (header.packetType == PKT_ACK &&
            header.payloadLen == sizeof(AckPayload))
               ? reinterpret_cast<const AckPayload*>(payload)
               : nullptr;
  }
};

/**
 * @brief Byte-fed streaming parser with resynchronization.
 *
 * Feed raw UART bytes; returns true when a complete, CRC-valid frame of
 * the correct protocol version has been assembled into `out`. Corrupt or
 * foreign bytes are skipped (the parser re-locks on the sync sequence)
 * and counted in stats.
 */
class Parser {
 public:
  struct Stats {
    uint32_t frames = 0;         ///< valid frames delivered
    uint32_t crcErrors = 0;
    uint32_t versionErrors = 0;  ///< frames with a wrong protocol version
    uint32_t lengthErrors = 0;   ///< payloadLen > MAX_PAYLOAD
    uint32_t resyncs = 0;        ///< bytes/frames skipped hunting for sync
  };

  bool feed(uint8_t byte, Packet& out);
  const Stats& stats() const { return stats_; }
  void reset();

 private:
  uint8_t buf_[MAX_FRAME];
  size_t pos_ = 0;
  size_t frameLen_ = 0;  ///< total expected frame length once header is in
  Stats stats_;
};

/**
 * @brief Encode a frame into `out` (must hold MAX_FRAME bytes).
 * @return total frame length in bytes, or 0 if payloadLen > MAX_PAYLOAD.
 */
size_t encode(uint8_t* out, PacketType type, uint16_t seq,
              uint32_t timestampUs, const void* payload, uint8_t payloadLen);

}  // namespace vcmproto
