/**
 * @file protocol.h
 * @brief Binary UART protocol between the Nano R4 extended I/O module and
 *        the ESP32 Vehicle Control Module (VCM).
 *
 * This module is intentionally free of any Arduino / MCU dependency so that
 * it can be compiled on the host for unit testing and dropped unchanged into
 * the ESP32 firmware (jrohder/dodge_patrol_vcm) for the other end of the
 * link.
 *
 * Wire format (all multi-byte fields little-endian):
 *
 *   offset  size  field
 *   ------  ----  --------------------------------------------------------
 *   0       1     sync byte 0 = 0x55
 *   1       1     sync byte 1 = 0xAA
 *   2       1     protocol version (kProtocolVersion)
 *   3       1     packet type (PacketType)
 *   4       1     payload length N (0..kMaxPayload)
 *   5       2     sequence number (per-sender, wraps)
 *   7       4     sender timestamp, microseconds since boot (wraps ~71 min)
 *   11      N     payload
 *   11+N    2     CRC16-CCITT-FALSE over bytes 2 .. 10+N (header after the
 *                 sync bytes, plus payload)
 *
 * Design rules:
 *   - Fixed-size packed structs, no strings, no dynamic allocation.
 *   - The Nano only ever REPORTS raw hardware values; interpretation
 *     (speed, gear, steering %) is exclusively the ESP32's job.
 *   - Every value has exactly one owner module; the protocol just carries
 *     snapshots.
 */
#ifndef VEIO_PROTOCOL_H
#define VEIO_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

namespace veio {
namespace proto {

// ---------------------------------------------------------------------------
// Framing constants
// ---------------------------------------------------------------------------

constexpr uint8_t kSync0 = 0x55;
constexpr uint8_t kSync1 = 0xAA;
constexpr uint8_t kProtocolVersion = 1;

constexpr size_t kHeaderSize = 11;  ///< sync..timestamp inclusive
constexpr size_t kCrcSize = 2;
constexpr size_t kMaxPayload = 96;
constexpr size_t kMaxFrameSize = kHeaderSize + kMaxPayload + kCrcSize;

// ---------------------------------------------------------------------------
// Packet types
// ---------------------------------------------------------------------------

/**
 * @brief Packet type identifiers.
 *
 * 0x01-0x0F : Nano -> ESP32 (implemented)
 * 0x10-0x1F : Nano -> ESP32 (reserved for future subsystems)
 * 0x40-0x4F : ESP32 -> Nano (commands)
 * 0x50-0x5F : ESP32 -> Nano (reserved, incl. firmware update transport)
 */
enum PacketType : uint8_t {
  kPktTelemetry  = 0x01,  ///< full I/O snapshot, fixed length, 100 Hz
  kPktHeartbeat  = 0x02,  ///< 1 Hz liveness beacon
  kPktDiagnostic = 0x03,  ///< internal health counters
  kPktCommandAck = 0x04,  ///< acknowledge of a received command
  kPktFault      = 0x05,  ///< emitted whenever the fault bitfield changes
  kPktBoot       = 0x06,  ///< emitted once when the Nano reaches READY
  kPktVersion    = 0x07,  ///< reply to kCmdRequestVersion
  kPktLog        = 0x08,  ///< reserved: event log dump

  // Reserved future telemetry sources (Nano -> ESP32):
  kPktReservedCan       = 0x10,
  kPktReservedGps       = 0x11,
  kPktReservedExtraHall = 0x12,
  kPktReservedButtons   = 0x13,
  kPktReservedTemp      = 0x14,
  kPktReservedLighting  = 0x15,

  kPktCommand = 0x40,     ///< ESP32 -> Nano command

  // Reserved firmware-update transport (ESP32 -> Nano, not yet implemented;
  // IDs allocated now so both sides can plan for it):
  kPktReservedFwBegin  = 0x50,
  kPktReservedFwData   = 0x51,
  kPktReservedFwVerify = 0x52,
  kPktReservedFwCommit = 0x53,
};

// ---------------------------------------------------------------------------
// Command identifiers (payload byte 0 of kPktCommand)
// ---------------------------------------------------------------------------

enum CommandId : uint8_t {
  kCmdSetOutput        = 0x01,  ///< args: [output, mode, halfPeriodLo, halfPeriodHi]
  kCmdOutputTest       = 0x02,  ///< args: [output] - brief self-test pattern
  kCmdResetCounters    = 0x03,  ///< zero pulse counts, min/max, diag counters
  kCmdStartDiagnostics = 0x04,  ///< args: [seconds] 0 = single diagnostic packet
  kCmdSetTestMode      = 0x05,  ///< args: [1=enable, 0=disable]
  kCmdRequestVersion   = 0x06,  ///< reply with kPktVersion
  kCmdPing             = 0x07,  ///< reply with kPktCommandAck only
  kCmdClearFaults      = 0x08,  ///< clear latched fault flags
};

/** Output identifiers for kCmdSetOutput / kCmdOutputTest. */
enum OutputId : uint8_t {
  kOutLightsSiren = 0x00,
  // 0x01..0x07 reserved for future outputs
  kOutCount = 1,
};

/** Output drive modes. */
enum OutputMode : uint8_t {
  kOutModeOff   = 0,
  kOutModeOn    = 1,
  kOutModeFlash = 2,
  // kOutModePwm = 3 reserved: args would carry an 8-bit duty
};

/** Command ack status codes. */
enum AckStatus : uint8_t {
  kAckOk          = 0,
  kAckUnknownCmd  = 1,
  kAckBadArgs     = 2,
  kAckRejected    = 3,
};

// ---------------------------------------------------------------------------
// System state & faults
// ---------------------------------------------------------------------------

/** Firmware state machine states, as reported in telemetry/heartbeat. */
enum SystemState : uint8_t {
  kStateBoot         = 0,
  kStateInitializing = 1,
  kStateReady        = 2,
  kStateDiagnostic   = 3,
  kStateTestMode     = 4,
  kStateFault        = 5,
};

/** Global fault bitfield (uint16). The Nano only reports; the ESP32 reacts. */
enum FaultFlag : uint16_t {
  kFaultRcSignalLost     = 1u << 0,  ///< all RC channels invalid
  kFaultUartCrc          = 1u << 1,  ///< CRC error(s) on the command link
  kFaultHallLeft         = 1u << 2,  ///< left wheel quadrature fault latched
  kFaultHallRight        = 1u << 3,  ///< right wheel quadrature fault latched
  kFaultAdc              = 1u << 4,  ///< ADC acquisition overrun/failure
  kFaultWatchdogReset    = 1u << 5,  ///< last reset was caused by the WDT
  kFaultOutput           = 1u << 6,  ///< reserved (no output feedback fitted)
  kFaultInternal         = 1u << 7,  ///< initialization/internal error
  kFaultSchedulerOverrun = 1u << 8,  ///< a task missed its deadline
  kFaultSamplerOverrun   = 1u << 9,  ///< fast sampler ISR overran its slot
  // bits 10..15 reserved
};

/** Per-wheel fault bits (WheelData::fault). */
enum WheelFault : uint8_t {
  kWheelFaultIllegalTransition = 1u << 0,  ///< both quadrature bits flipped at once
  kWheelFaultTimeout           = 1u << 1,  ///< informational: no edges (stopped)
  // bits 2..7 reserved
};

/** Subsystem health levels (2 bits each in DiagnosticPayload::subsystems). */
enum HealthLevel : uint8_t {
  kHealthOk      = 0,
  kHealthWarning = 1,
  kHealthFault   = 2,
};

/** Bit offsets of each subsystem inside DiagnosticPayload::subsystems. */
enum SubsystemShift : uint8_t {
  kSubRc        = 0,
  kSubHallLeft  = 2,
  kSubHallRight = 4,
  kSubAdc       = 6,
  kSubUart      = 8,
  kSubScheduler = 10,
  kSubOutputs   = 12,
  kSubSystem    = 14,
};

// ---------------------------------------------------------------------------
// Payload structures (packed, little-endian on the wire)
// ---------------------------------------------------------------------------

#pragma pack(push, 1)

/**
 * @brief Raw quadrature measurement for one wheel.
 *
 * The Nano never converts these to speed: pulse counts and edge periods are
 * pure hardware observations.  The ESP32 owns wheel geometry.
 */
struct WheelData {
  int32_t  pulseCount;   ///< signed quadrature edge count since boot/reset
  uint16_t freqHzX10;    ///< quadrature edge frequency, 0.1 Hz units (0 = stopped)
  int8_t   direction;    ///< +1 = A leads B, -1 = B leads A, 0 = stopped/unknown
  uint32_t periodUs;     ///< microseconds between the last two edges (0 = stopped)
  uint8_t  fault;        ///< WheelFault bits
};
static_assert(sizeof(WheelData) == 12, "WheelData wire size");

/**
 * @brief Fixed-length telemetry snapshot, transmitted every 10 ms (100 Hz).
 *
 * One packet carries the complete, time-consistent state of every input and
 * output so the ESP32 always parses a single synchronized snapshot.
 */
struct TelemetryPayload {
  uint16_t rcPulseUs[6];   ///< raw pulse widths CH1..CH6 (0 = never seen)
  uint8_t  rcValidMask;    ///< bit n = CH(n+1) currently valid
  uint8_t  rcAgeMs[6];     ///< ms since last valid pulse, clamped to 255

  WheelData left;          ///< left wheel quadrature state
  WheelData right;         ///< right wheel quadrature state

  uint16_t motorSenseARaw;  ///< A0 oversampled average, raw ADC counts
  uint16_t motorSenseAFilt; ///< A0 EMA-filtered ADC counts
  uint16_t motorSenseBRaw;  ///< A1 oversampled average, raw ADC counts
  uint16_t motorSenseBFilt; ///< A1 EMA-filtered ADC counts
  uint16_t batteryRaw;      ///< A3 oversampled average, raw ADC counts
  uint16_t batteryFilt;     ///< A3 EMA-filtered ADC counts
  uint16_t batteryMin;      ///< minimum filtered value since boot/reset
  uint16_t batteryMax;      ///< maximum filtered value since boot/reset

  uint8_t  outputState;    ///< bits0-1: commanded OutputMode, bit7: physical level
  uint8_t  systemState;    ///< SystemState
  uint16_t faultFlags;     ///< FaultFlag bitfield
};
static_assert(sizeof(TelemetryPayload) == 63, "TelemetryPayload wire size");

/** @brief 1 Hz liveness beacon. */
struct HeartbeatPayload {
  uint32_t uptimeMs;        ///< milliseconds since boot
  uint8_t  systemState;     ///< SystemState
  uint16_t faultFlags;      ///< FaultFlag bitfield
  uint8_t  protocolVersion; ///< kProtocolVersion (redundant sanity check)
};
static_assert(sizeof(HeartbeatPayload) == 8, "HeartbeatPayload wire size");

/** @brief Internal health counters (kPktDiagnostic). */
struct DiagnosticPayload {
  uint16_t loopTimeAvgUs;     ///< main loop iteration time, EMA
  uint16_t loopTimeMaxUs;     ///< main loop iteration time, max since reset
  uint8_t  cpuLoadPct;        ///< scheduler busy time over the last second
  uint16_t subsystems;        ///< 2-bit HealthLevel per SubsystemShift
  uint16_t schedulerOverruns; ///< tasks that missed a deadline
  uint16_t samplerOverruns;   ///< fast-sampler ISR overruns
  uint16_t uartCrcErrors;     ///< frames dropped for bad CRC
  uint16_t uartFrameErrors;   ///< malformed headers / resyncs
  uint16_t uartTxDrops;       ///< packets dropped because TX buffer was full
  uint16_t commandsReceived;  ///< valid commands executed
  uint16_t rcGlitches;        ///< out-of-range RC pulses discarded
  uint16_t freeRamBytes;      ///< bytes between heap break and stack pointer
};
static_assert(sizeof(DiagnosticPayload) == 23, "DiagnosticPayload wire size");

/** @brief Command acknowledge (kPktCommandAck). */
struct CommandAckPayload {
  uint8_t  commandId;    ///< CommandId being acknowledged
  uint8_t  status;       ///< AckStatus
  uint16_t commandSeq;   ///< sequence number of the acknowledged command frame
};
static_assert(sizeof(CommandAckPayload) == 4, "CommandAckPayload wire size");

/** @brief Fault change notification (kPktFault). */
struct FaultPayload {
  uint16_t faultFlags;   ///< current FaultFlag bitfield
  uint16_t changedMask;  ///< bits that changed relative to the previous state
};
static_assert(sizeof(FaultPayload) == 4, "FaultPayload wire size");

/** @brief Boot announcement / version report (kPktBoot and kPktVersion). */
struct VersionPayload {
  uint8_t  fwMajor;
  uint8_t  fwMinor;
  uint8_t  fwPatch;
  uint8_t  protocolVersion;
  uint32_t gitHash;        ///< short commit hash as 32-bit value, 0 if unknown
  char     buildDate[12];  ///< __DATE__, NUL padded
  char     buildTime[9];   ///< __TIME__, NUL padded
  uint8_t  boardId;
  uint8_t  bootReason;     ///< 0 unknown, 1 power-on/external, 2 watchdog
};
static_assert(sizeof(VersionPayload) == 31, "VersionPayload wire size");

/** @brief ESP32 -> Nano command (kPktCommand). Fixed 5-byte payload. */
struct CommandPayload {
  uint8_t commandId;  ///< CommandId
  uint8_t args[4];    ///< command-specific arguments, zero-filled
};
static_assert(sizeof(CommandPayload) == 5, "CommandPayload wire size");

#pragma pack(pop)

// ---------------------------------------------------------------------------
// CRC16 and frame encode/decode
// ---------------------------------------------------------------------------

/**
 * @brief CRC16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection).
 * @param data  bytes to checksum
 * @param len   number of bytes
 * @return 16-bit CRC ("123456789" -> 0x29B1)
 */
uint16_t crc16(const uint8_t* data, size_t len);

/**
 * @brief Serialize a complete frame into @p out.
 *
 * @param out          destination buffer, at least kHeaderSize + payloadLen
 *                     + kCrcSize bytes
 * @param type         packet type
 * @param sequence     sender sequence number
 * @param timestampUs  sender clock in microseconds
 * @param payload      payload bytes (may be nullptr when payloadLen == 0)
 * @param payloadLen   payload length, <= kMaxPayload
 * @return total frame length in bytes, or 0 if payloadLen is out of range
 */
size_t encodeFrame(uint8_t* out, PacketType type, uint16_t sequence,
                   uint32_t timestampUs, const void* payload,
                   size_t payloadLen);

/** @brief Decoded frame handed out by FrameParser. */
struct Frame {
  uint8_t  version;
  uint8_t  type;
  uint8_t  payloadLen;
  uint16_t sequence;
  uint32_t timestampUs;
  uint8_t  payload[kMaxPayload];
};

/**
 * @brief Incremental, resynchronizing frame parser.
 *
 * Feed received bytes one at a time; feed() returns true exactly when a
 * complete frame with a valid CRC has been assembled in frame().  The parser
 * automatically resynchronizes on the 0x55 0xAA preamble after garbage,
 * truncated frames or CRC failures, and keeps error counters for the
 * diagnostics module.
 */
/** @return exact payload size for a known type, or -1 if the type is unused. */
int expectedPayloadSize(uint8_t type);

class FrameParser {
 public:
  FrameParser() { reset(); }

  /**
   * @brief Consume one received byte.
   * @return true when a complete, CRC-valid frame is available via frame()
   */
  bool feed(uint8_t byte);

  /** @return last successfully decoded frame (valid after feed() == true) */
  const Frame& frame() const { return frame_; }

  /** @return frames discarded due to CRC mismatch */
  uint16_t crcErrors() const { return crcErrors_; }
  /** @return header-level errors (bad version/length or lost sync) */
  uint16_t frameErrors() const { return frameErrors_; }

  bool partial() const { return state_ != St::kSync0; }

  /// Drop a half-built frame (inter-byte gap) without clearing counters.
  void abandonPartial();

  /** @brief Reset parser state and error counters. */
  void reset();

 private:
  enum class St : uint8_t { kSync0, kSync1, kHeader, kBody };

  bool replayFrom(size_t start);
  bool failAndReplay(bool crc);

  St       state_;
  size_t   idx_;
  size_t   total_;
  uint8_t  raw_[kMaxFrameSize];
  Frame    frame_;
  uint16_t crcErrors_;
  uint16_t frameErrors_;
  bool     resyncing_;
};

}  // namespace proto
}  // namespace veio

#endif  // VEIO_PROTOCOL_H
