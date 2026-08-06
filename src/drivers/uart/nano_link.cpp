#include "drivers/uart/nano_link.h"

#include <cstring>

#include "core/pins.h"
#include "services/logger.h"

namespace vcm {

using namespace vcmproto;

NanoLink nano;

static HardwareSerial& kUart = Serial1;

void NanoLink::begin() {
  kUart.begin(DEFAULT_BAUD, SERIAL_8N1, pins::NANO_RX, pins::NANO_TX);
  kUart.setRxBufferSize(1024);
  LOGI("NANO", "UART link @ %lu baud, protocol v%u (RX=%d TX=%d)",
       (unsigned long)DEFAULT_BAUD, PROTOCOL_VERSION, pins::NANO_RX,
       pins::NANO_TX);
}

void NanoLink::poll() {
  Packet pkt;
  while (kUart.available() > 0) {
    if (parser_.feed((uint8_t)kUart.read(), pkt)) handlePacket(pkt);
  }
  // Protocol mismatch is latched while wrong-version frames keep arriving
  if (parser_.stats().versionErrors != lastVersionErrors_) {
    if (!protocolMismatch_)
      LOGE("NANO", "Protocol mismatch: ESP32 speaks v%u, Nano sends other",
           PROTOCOL_VERSION);
    protocolMismatch_ = true;
    lastVersionErrors_ = parser_.stats().versionErrors;
  }
}

void NanoLink::handlePacket(const Packet& pkt) {
  // Any valid same-version frame clears a previous mismatch latch
  protocolMismatch_ = false;

  if (haveSeq_) {
    const uint16_t gap = (uint16_t)(pkt.header.seq - lastSeq_);
    if (gap == 0) {
      seqErrors_++;
    } else if (gap > 1) {
      packetsLost_ += gap - 1;
    }
  }
  lastSeq_ = pkt.header.seq;
  haveSeq_ = true;

  if (const TelemetryPayload* t = pkt.asTelemetry()) {
    handleTelemetry(pkt, *t);
  } else if (const EventPayload* e = pkt.asEvent()) {
    eventsReceived_++;
    events_[eventHead_] = *e;
    eventHead_ = (eventHead_ + 1) % EVENT_RING;
    if (eventCount_ < EVENT_RING) eventCount_++;
    LOGW("NANO", "Nano event 0x%02X data %02X %02X %02X @ %lu us", e->code,
         e->data[0], e->data[1], e->data[2], (unsigned long)e->timestampUs);
  } else if (const AckPayload* a = pkt.asAck()) {
    acksReceived_++;
    if (a->status != 0)
      LOGW("NANO", "Nano NACK cmd 0x%02X status %u (seq %u)", a->command,
           a->status, a->ackSeq);
  }
}

void NanoLink::handleTelemetry(const Packet& pkt, const TelemetryPayload& t) {
  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  // Latency jitter: compare local inter-arrival vs Nano inter-timestamp
  if (lastPacketMs_ != 0) {
    const uint32_t localDt = nowUs - lastArrivalUs_;
    const uint32_t nanoDt = pkt.header.timestampUs - lastTimestampUs_;
    const float dev = fabsf((float)localDt - (float)nanoDt);
    if (dev < 500000.0f)  // ignore wrap/startup outliers
      jitterUs_ += 0.05f * (dev - jitterUs_);
  }
  lastArrivalUs_ = nowUs;
  lastTimestampUs_ = pkt.header.timestampUs;

  latest_ = t;
  lastPacketMs_ = nowMs;

  // Packet rate over a 1 s window
  rateWindowCount_++;
  if (nowMs - rateWindowStart_ >= 1000) {
    packetRateHz_ =
        rateWindowCount_ * 1000.0f / (float)(nowMs - rateWindowStart_);
    rateWindowStart_ = nowMs;
    rateWindowCount_ = 0;
  }
}

size_t NanoLink::copyEvents(EventPayload* out, size_t max) const {
  const size_t n = eventCount_ < max ? eventCount_ : max;
  for (size_t i = 0; i < n; ++i) {
    // oldest first
    const size_t idx = (eventHead_ + EVENT_RING - eventCount_ + i) % EVENT_RING;
    out[i] = events_[idx];
  }
  return n;
}

void NanoLink::sendCommand(Command cmd, const uint8_t args[4]) {
  CommandPayload p = {};
  p.command = (uint8_t)cmd;
  if (args) memcpy(p.args, args, sizeof(p.args));
  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_COMMAND, txSeq_++, micros(), &p, sizeof(p));
  kUart.write(frame, len);
}

void NanoLink::sendHeartbeat() { sendCommand(CMD_HEARTBEAT); }

}  // namespace vcm
