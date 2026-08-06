#include "drivers/uart/nano_link.h"

#include "core/pins.h"
#include "drivers/uart/crc16.h"
#include "services/logger.h"

namespace vcm {

NanoLink nano;

static HardwareSerial& kUart = Serial1;

void NanoLink::begin() {
  kUart.begin(NANO_BAUD, SERIAL_8N1, pins::NANO_RX, pins::NANO_TX);
  kUart.setRxBufferSize(1024);
  LOGI("NANO", "UART link @ %lu baud (RX=%d TX=%d)",
       (unsigned long)NANO_BAUD, pins::NANO_RX, pins::NANO_TX);
}

void NanoLink::poll() {
  while (kUart.available() > 0) {
    const uint8_t byte = (uint8_t)kUart.read();

    // Resynchronize on the magic word (0x55 0xAA little-endian)
    if (bufLen_ == 0) {
      if (byte != 0x55) continue;
    } else if (bufLen_ == 1) {
      if (byte != 0xAA) {
        bufLen_ = 0;
        if (byte == 0x55) bufLen_ = 1;  // could be start of next frame
        continue;
      }
    }
    buf_[bufLen_++] = byte;

    if (bufLen_ == sizeof(NanoTelemetryPacket)) {
      bufLen_ = 0;
      NanoTelemetryPacket pkt;
      memcpy(&pkt, buf_, sizeof(pkt));
      const uint16_t expected =
          crc16(buf_, sizeof(NanoTelemetryPacket) - sizeof(uint16_t));
      if (pkt.crc != expected) {
        crcErrors_++;
        continue;
      }
      handlePacket(pkt);
    }
  }
}

void NanoLink::handlePacket(const NanoTelemetryPacket& pkt) {
  if (pkt.version != NANO_PROTOCOL_VERSION) {
    if (!protocolMismatch_)
      LOGE("NANO", "Protocol mismatch: Nano v%u, ESP32 v%u", pkt.version,
           NANO_PROTOCOL_VERSION);
    protocolMismatch_ = true;
    return;
  }
  protocolMismatch_ = false;

  if (haveSeq_) {
    const uint16_t gap = (uint16_t)(pkt.seq - lastSeq_);
    if (gap == 0) {
      seqErrors_++;
    } else if (gap > 1) {
      packetsLost_ += gap - 1;
    }
  }
  lastSeq_ = pkt.seq;
  haveSeq_ = true;

  latest_ = pkt;
  lastPacketMs_ = millis();
  packetsReceived_++;

  // Packet rate over a 1 s window
  rateWindowCount_++;
  const uint32_t now = millis();
  if (now - rateWindowStart_ >= 1000) {
    packetRateHz_ =
        rateWindowCount_ * 1000.0f / (float)(now - rateWindowStart_);
    rateWindowStart_ = now;
    rateWindowCount_ = 0;
  }
}

void NanoLink::sendCommand(NanoCommand type, const uint8_t payload[4]) {
  NanoCommandPacket cmd = {};
  cmd.magic = NANO_MAGIC;
  cmd.version = NANO_PROTOCOL_VERSION;
  cmd.type = (uint8_t)type;
  if (payload) memcpy(cmd.payload, payload, sizeof(cmd.payload));
  cmd.crc = crc16((const uint8_t*)&cmd,
                  sizeof(NanoCommandPacket) - sizeof(uint16_t));
  kUart.write((const uint8_t*)&cmd, sizeof(cmd));
}

void NanoLink::sendHeartbeat() { sendCommand(NanoCommand::HEARTBEAT); }

}  // namespace vcm
