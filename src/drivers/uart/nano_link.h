/**
 * @file nano_link.h
 * @brief UART link driver to the Nano R4 extended-I/O processor.
 *
 * Thin transport wrapper around the SHARED protocol module
 * (src/proto/protocol.{h,cpp} — copied verbatim from vcm_extended_io).
 * All wire-format knowledge lives there.
 */
#pragma once

#include <Arduino.h>

#include "proto/protocol.h"

namespace vcm {

class NanoLink {
 public:
  static constexpr uint32_t DEFAULT_BAUD = 460800;

  void begin(uint32_t baud = DEFAULT_BAUD);

  /// Drain UART RX into the shared FrameParser. Call at >= 100 Hz.
  void poll();

  /// Send a framed command (keeps Nano output failsafe alive).
  void sendCommand(veio::proto::CommandId cmd, const uint8_t args[4] = nullptr);

  /// Ping the Nano (preferred keepalive — Nano answers with COMMAND_ACK).
  void sendPing();
  void sendHeartbeat() { sendPing(); }  ///< alias used by the dynamics task

  bool online(uint32_t timeoutMs) const {
    return lastPacketMs_ != 0 && (millis() - lastPacketMs_) < timeoutMs;
  }

  const veio::proto::TelemetryPayload& latest() const { return latest_; }
  const veio::proto::HeartbeatPayload& heartbeat() const { return heartbeat_; }
  const veio::proto::DiagnosticPayload& diagnostic() const { return diagnostic_; }
  const veio::proto::VersionPayload& version() const { return version_; }
  bool haveVersion() const { return haveVersion_; }
  bool haveDiagnostic() const { return haveDiagnostic_; }

  uint32_t lastPacketMs() const { return lastPacketMs_; }
  uint32_t lastTimestampUs() const { return lastTimestampUs_; }
  uint32_t lastHeartbeatMs() const { return lastHeartbeatMs_; }

  // Link statistics --------------------------------------------------------
  uint32_t packetsReceived() const { return packetsReceived_; }
  uint32_t telemetryReceived() const { return telemetryReceived_; }
  uint32_t crcErrors() const { return parser_.crcErrors(); }
  uint32_t frameErrors() const { return parser_.frameErrors(); }
  uint32_t seqErrors() const { return seqErrors_; }
  uint32_t packetsLost() const { return packetsLost_; }
  uint32_t acksReceived() const { return acksReceived_; }
  uint32_t faultsReceived() const { return faultsReceived_; }
  uint32_t bytesReceived() const { return bytesReceived_; }
  uint32_t rxHighWater() const { return rxHighWater_; }
  uint32_t rxBackpressure() const { return rxBackpressure_; }
  uint32_t txDrops() const { return txDrops_; }
  int rxPinLevel() const { return rxPinLevel_; }
  float packetRateHz() const { return packetRateHz_; }
  float jitterUs() const { return jitterUs_; }
  bool protocolMismatch() const { return protocolMismatch_; }

 private:
  void handleFrame(const veio::proto::Frame& frame);
  void handleTelemetry(const veio::proto::Frame& frame,
                       const veio::proto::TelemetryPayload& t);
  void noteSequence(uint16_t seq);

  veio::proto::FrameParser parser_;
  veio::proto::TelemetryPayload latest_ = {};
  veio::proto::HeartbeatPayload heartbeat_ = {};
  veio::proto::DiagnosticPayload diagnostic_ = {};
  veio::proto::VersionPayload version_ = {};
  bool haveVersion_ = false;
  bool haveDiagnostic_ = false;
  bool protocolMismatch_ = false;

  volatile uint32_t lastPacketMs_ = 0;
  uint32_t lastHeartbeatMs_ = 0;
  uint32_t lastTimestampUs_ = 0;
  uint32_t lastArrivalUs_ = 0;
  uint16_t lastSeq_ = 0;
  bool haveSeq_ = false;
  uint32_t packetsReceived_ = 0, telemetryReceived_ = 0;
  uint32_t seqErrors_ = 0, packetsLost_ = 0;
  uint32_t acksReceived_ = 0, faultsReceived_ = 0;
  uint32_t bytesReceived_ = 0;
  uint32_t rxHighWater_ = 0;
  uint32_t rxBackpressure_ = 0;
  uint32_t lastRxMs_ = 0;
  uint32_t txDrops_ = 0;
  int rxPinLevel_ = -1;
  float packetRateHz_ = 0.0f, jitterUs_ = 0.0f;
  uint32_t rateWindowStart_ = 0, rateWindowCount_ = 0;
  uint16_t txSeq_ = 0;
  uint32_t baud_ = DEFAULT_BAUD;
};

extern NanoLink nano;

}  // namespace vcm
