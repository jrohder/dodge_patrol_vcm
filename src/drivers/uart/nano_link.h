/**
 * @file nano_link.h
 * @brief UART link driver to the Nano R4 extended-I/O processor.
 *
 * Thin transport wrapper around the SHARED protocol module
 * (src/proto/protocol.{h,cpp}) - all wire format knowledge lives there.
 * This driver owns the ESP32 UART, feeds received bytes to the shared
 * parser, tracks link statistics (loss, sequence gaps, resyncs, latency
 * jitter), collects Nano event-log packets and sends framed commands.
 */
#pragma once

#include <Arduino.h>

#include "proto/protocol.h"

namespace vcm {

class NanoLink {
 public:
  static constexpr size_t EVENT_RING = 8;

  void begin();

  /// Poll the UART, feed the shared parser. Call at >= 100 Hz.
  void poll();

  /// Send the heartbeat command. Call at ~10-100 Hz.
  void sendHeartbeat();
  void sendCommand(vcmproto::Command cmd, const uint8_t args[4] = nullptr);

  /// True if a valid telemetry frame arrived within timeoutMs.
  bool online(uint32_t timeoutMs) const {
    return lastPacketMs_ != 0 && (millis() - lastPacketMs_) < timeoutMs;
  }

  const vcmproto::TelemetryPayload& latest() const { return latest_; }
  uint32_t lastPacketMs() const { return lastPacketMs_; }
  uint32_t lastTimestampUs() const { return lastTimestampUs_; }

  // Link statistics --------------------------------------------------------
  uint32_t packetsReceived() const { return parser_.stats().frames; }
  uint32_t crcErrors() const { return parser_.stats().crcErrors; }
  uint32_t versionErrors() const { return parser_.stats().versionErrors; }
  uint32_t lengthErrors() const { return parser_.stats().lengthErrors; }
  uint32_t resyncs() const { return parser_.stats().resyncs; }
  uint32_t seqErrors() const { return seqErrors_; }
  uint32_t packetsLost() const { return packetsLost_; }
  float packetRateHz() const { return packetRateHz_; }
  bool protocolMismatch() const { return protocolMismatch_; }

  /// EMA of |local inter-arrival - Nano inter-timestamp| in us. A rough
  /// transport latency-jitter figure (clocks are not synchronized, so
  /// absolute latency is not observable).
  float jitterUs() const { return jitterUs_; }

  // Nano event log ---------------------------------------------------------
  uint32_t eventsReceived() const { return eventsReceived_; }
  uint32_t acksReceived() const { return acksReceived_; }
  /// Copy up to EVENT_RING most recent events (oldest first). Returns count.
  size_t copyEvents(vcmproto::EventPayload* out, size_t max) const;

 private:
  void handlePacket(const vcmproto::Packet& pkt);
  void handleTelemetry(const vcmproto::Packet& pkt,
                       const vcmproto::TelemetryPayload& t);

  vcmproto::Parser parser_;
  vcmproto::TelemetryPayload latest_ = {};
  volatile uint32_t lastPacketMs_ = 0;
  uint32_t lastTimestampUs_ = 0;
  uint32_t lastArrivalUs_ = 0;
  uint16_t lastSeq_ = 0;
  bool haveSeq_ = false;
  bool protocolMismatch_ = false;
  uint32_t lastVersionErrors_ = 0;
  uint32_t seqErrors_ = 0, packetsLost_ = 0;
  float packetRateHz_ = 0.0f, jitterUs_ = 0.0f;
  uint32_t rateWindowStart_ = 0, rateWindowCount_ = 0;
  uint16_t txSeq_ = 0;
  uint32_t eventsReceived_ = 0, acksReceived_ = 0;
  vcmproto::EventPayload events_[EVENT_RING] = {};
  size_t eventHead_ = 0, eventCount_ = 0;
};

extern NanoLink nano;

}  // namespace vcm
