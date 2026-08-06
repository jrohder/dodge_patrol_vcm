/**
 * @file nano_link.h
 * @brief UART link driver to the Nano R4 IO processor.
 *
 * Handles byte-stream framing, CRC validation, sequence tracking, link
 * statistics and heartbeat transmission. Publishes decoded values into
 * the telemetry hub and exposes the latest packet for the control tasks.
 */
#pragma once

#include <Arduino.h>

#include "drivers/uart/nano_protocol.h"

namespace vcm {

class NanoLink {
 public:
  void begin();

  /// Poll the UART, parse frames. Call at >= 100 Hz.
  void poll();

  /// Send the heartbeat/command packet. Call at 100 Hz.
  void sendHeartbeat();
  void sendCommand(NanoCommand type, const uint8_t payload[4] = nullptr);

  /// True if a valid packet arrived within timeoutMs.
  bool online(uint32_t timeoutMs) const {
    return lastPacketMs_ != 0 && (millis() - lastPacketMs_) < timeoutMs;
  }

  const NanoTelemetryPacket& latest() const { return latest_; }
  uint32_t lastPacketMs() const { return lastPacketMs_; }

  // link statistics
  uint32_t packetsReceived() const { return packetsReceived_; }
  uint32_t crcErrors() const { return crcErrors_; }
  uint32_t seqErrors() const { return seqErrors_; }
  uint32_t packetsLost() const { return packetsLost_; }
  float packetRateHz() const { return packetRateHz_; }
  bool protocolMismatch() const { return protocolMismatch_; }

 private:
  void handlePacket(const NanoTelemetryPacket& pkt);

  uint8_t buf_[sizeof(NanoTelemetryPacket)];
  size_t bufLen_ = 0;
  NanoTelemetryPacket latest_ = {};
  volatile uint32_t lastPacketMs_ = 0;
  uint16_t lastSeq_ = 0;
  bool haveSeq_ = false;
  uint32_t packetsReceived_ = 0, crcErrors_ = 0, seqErrors_ = 0,
           packetsLost_ = 0;
  float packetRateHz_ = 0.0f;
  uint32_t rateWindowStart_ = 0, rateWindowCount_ = 0;
  bool protocolMismatch_ = false;
};

extern NanoLink nano;

}  // namespace vcm
