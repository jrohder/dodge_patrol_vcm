#include "drivers/uart/nano_link.h"

#include <cstring>

#include "core/pins.h"
#include "driver/gpio.h"
#include "services/logger.h"

namespace vcm {

using namespace veio::proto;

NanoLink nano;

static HardwareSerial& kUart = Serial1;

void NanoLink::begin(uint32_t baud) {
  baud_ = baud;
  // Buffer sizes must be set before begin(); calling setRxBufferSize()
  // afterwards is a no-op and logs "RX Buffer can't be resized when Serial
  // is already running" — the default 256-byte RX ring then overruns at
  // 100 Hz / 460800.
  kUart.setRxBufferSize(1024);
  kUart.setTxBufferSize(512);
  kUart.begin(baud_, SERIAL_8N1, pins::NANO_RX, pins::NANO_TX);
  kUart.setTimeout(0);
  LOGI("NANO", "UART link @ %lu baud, protocol v%u (RX=%d TX=%d)",
       (unsigned long)baud_, kProtocolVersion, pins::NANO_RX, pins::NANO_TX);
  // Ask the Nano to identify itself once the link is up.
  sendCommand(kCmdRequestVersion);
}

void NanoLink::poll() {
  rxPinLevel_ = gpio_get_level((gpio_num_t)pins::NANO_RX);
  while (kUart.available() > 0) {
    bytesReceived_++;
    if (parser_.feed((uint8_t)kUart.read())) {
      handleFrame(parser_.frame());
    }
  }
}

void NanoLink::noteSequence(uint16_t seq) {
  if (haveSeq_) {
    const uint16_t gap = (uint16_t)(seq - lastSeq_);
    if (gap == 0) {
      seqErrors_++;
    } else if (gap > 1) {
      packetsLost_ += gap - 1;
    }
  }
  lastSeq_ = seq;
  haveSeq_ = true;
}

void NanoLink::handleFrame(const Frame& frame) {
  if (frame.version != kProtocolVersion) {
    if (!protocolMismatch_)
      LOGE("NANO", "Protocol mismatch: ESP32 v%u, Nano frame v%u",
           kProtocolVersion, frame.version);
    protocolMismatch_ = true;
    return;
  }
  protocolMismatch_ = false;
  noteSequence(frame.sequence);
  packetsReceived_++;
  lastPacketMs_ = millis();

  switch (frame.type) {
    case kPktTelemetry:
      if (frame.payloadLen == sizeof(TelemetryPayload)) {
        TelemetryPayload t;
        memcpy(&t, frame.payload, sizeof(t));
        handleTelemetry(frame, t);
      }
      break;

    case kPktHeartbeat:
      if (frame.payloadLen == sizeof(HeartbeatPayload)) {
        memcpy(&heartbeat_, frame.payload, sizeof(heartbeat_));
        lastHeartbeatMs_ = millis();
        if (heartbeat_.protocolVersion != kProtocolVersion) {
          protocolMismatch_ = true;
        }
      }
      break;

    case kPktDiagnostic:
      if (frame.payloadLen == sizeof(DiagnosticPayload)) {
        memcpy(&diagnostic_, frame.payload, sizeof(diagnostic_));
        haveDiagnostic_ = true;
      }
      break;

    case kPktCommandAck:
      if (frame.payloadLen == sizeof(CommandAckPayload)) {
        CommandAckPayload ack;
        memcpy(&ack, frame.payload, sizeof(ack));
        acksReceived_++;
        if (ack.status != kAckOk)
          LOGW("NANO", "Nano NACK cmd 0x%02X status %u (seq %u)", ack.commandId,
               ack.status, ack.commandSeq);
      }
      break;

    case kPktFault:
      if (frame.payloadLen == sizeof(FaultPayload)) {
        FaultPayload f;
        memcpy(&f, frame.payload, sizeof(f));
        faultsReceived_++;
        latest_.faultFlags = f.faultFlags;
        LOGW("NANO", "Nano fault change: flags=0x%04X changed=0x%04X",
             f.faultFlags, f.changedMask);
      }
      break;

    case kPktBoot:
    case kPktVersion:
      if (frame.payloadLen == sizeof(VersionPayload)) {
        memcpy(&version_, frame.payload, sizeof(version_));
        haveVersion_ = true;
        LOGI("NANO", "Nano FW %u.%u.%u proto v%u bootReason=%u",
             version_.fwMajor, version_.fwMinor, version_.fwPatch,
             version_.protocolVersion, version_.bootReason);
        if (version_.protocolVersion != kProtocolVersion)
          protocolMismatch_ = true;
      }
      break;

    default:
      break;
  }

  // Packet rate over a 1 s window (any valid frame)
  rateWindowCount_++;
  const uint32_t now = millis();
  if (now - rateWindowStart_ >= 1000) {
    packetRateHz_ =
        rateWindowCount_ * 1000.0f / (float)(now - rateWindowStart_);
    rateWindowStart_ = now;
    rateWindowCount_ = 0;
  }
}

void NanoLink::handleTelemetry(const Frame& frame,
                               const TelemetryPayload& t) {
  const uint32_t nowUs = micros();
  if (telemetryReceived_ > 0) {
    const uint32_t localDt = nowUs - lastArrivalUs_;
    const uint32_t nanoDt = frame.timestampUs - lastTimestampUs_;
    const float dev = fabsf((float)localDt - (float)nanoDt);
    if (dev < 500000.0f) jitterUs_ += 0.05f * (dev - jitterUs_);
  }
  lastArrivalUs_ = nowUs;
  lastTimestampUs_ = frame.timestampUs;
  latest_ = t;
  telemetryReceived_++;
}

void NanoLink::sendCommand(CommandId cmd, const uint8_t args[4]) {
  CommandPayload p = {};
  p.commandId = (uint8_t)cmd;
  if (args) memcpy(p.args, args, sizeof(p.args));
  uint8_t frame[kMaxFrameSize];
  const size_t len =
      encodeFrame(frame, kPktCommand, txSeq_++, micros(), &p, sizeof(p));
  if (len) kUart.write(frame, len);
}

void NanoLink::sendPing() { sendCommand(kCmdPing); }

}  // namespace vcm
