/**
 * @file protocol.cpp
 * @brief Shared VCM extended-I/O UART protocol implementation (version 2).
 *
 * Portable C++ - shared verbatim between the Nano extended-I/O firmware
 * and the ESP32 VCM. See protocol.h.
 */
#include "proto/protocol.h"

#include <cstring>

namespace vcmproto {

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                           : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

void Parser::reset() {
  pos_ = 0;
  frameLen_ = 0;
}

bool Parser::feed(uint8_t byte, Packet& out) {
  // Hunt for the sync sequence
  if (pos_ == 0) {
    if (byte != SYNC1) {
      stats_.resyncs++;
      return false;
    }
  } else if (pos_ == 1) {
    if (byte != SYNC2) {
      stats_.resyncs++;
      pos_ = (byte == SYNC1) ? 1 : 0;  // byte could start the next frame
      return false;
    }
  }

  buf_[pos_++] = byte;

  // Once the header is complete, validate it and compute the frame length
  if (pos_ == HEADER_SIZE) {
    const uint8_t version = buf_[2];
    const uint8_t payloadLen = buf_[4];
    if (version != PROTOCOL_VERSION) {
      stats_.versionErrors++;
      reset();
      return false;
    }
    if (payloadLen > MAX_PAYLOAD) {
      stats_.lengthErrors++;
      reset();
      return false;
    }
    frameLen_ = HEADER_SIZE + payloadLen + CRC_SIZE;
  }

  if (frameLen_ == 0 || pos_ < frameLen_) return false;

  // Full frame buffered: verify CRC over header+payload
  const size_t crcOffset = frameLen_ - CRC_SIZE;
  uint16_t rxCrc;
  memcpy(&rxCrc, buf_ + crcOffset, sizeof(rxCrc));
  const uint16_t calc = crc16(buf_, crcOffset);
  if (rxCrc != calc) {
    stats_.crcErrors++;
    reset();
    return false;
  }

  memcpy(&out.header, buf_, HEADER_SIZE);
  memcpy(out.payload, buf_ + HEADER_SIZE, out.header.payloadLen);
  stats_.frames++;
  reset();
  return true;
}

size_t encode(uint8_t* out, PacketType type, uint16_t seq,
              uint32_t timestampUs, const void* payload, uint8_t payloadLen) {
  if (payloadLen > MAX_PAYLOAD) return 0;
  Header h;
  h.sync1 = SYNC1;
  h.sync2 = SYNC2;
  h.version = PROTOCOL_VERSION;
  h.packetType = (uint8_t)type;
  h.payloadLen = payloadLen;
  h.seq = seq;
  h.timestampUs = timestampUs;
  memcpy(out, &h, HEADER_SIZE);
  if (payloadLen) memcpy(out + HEADER_SIZE, payload, payloadLen);
  const uint16_t crc = crc16(out, HEADER_SIZE + payloadLen);
  memcpy(out + HEADER_SIZE + payloadLen, &crc, sizeof(crc));
  return HEADER_SIZE + payloadLen + CRC_SIZE;
}

}  // namespace vcmproto
