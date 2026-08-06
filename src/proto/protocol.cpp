/**
 * @file protocol.cpp
 * @brief Frame encode / incremental decode for the Nano<->ESP32 UART link.
 *
 * Host-portable: no Arduino dependencies, unit-tested natively.
 */
#include "protocol.h"

#include <string.h>

namespace veio {
namespace proto {

uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

size_t encodeFrame(uint8_t* out, PacketType type, uint16_t sequence,
                   uint32_t timestampUs, const void* payload,
                   size_t payloadLen) {
  if (payloadLen > kMaxPayload) {
    return 0;
  }
  out[0] = kSync0;
  out[1] = kSync1;
  out[2] = kProtocolVersion;
  out[3] = static_cast<uint8_t>(type);
  out[4] = static_cast<uint8_t>(payloadLen);
  out[5] = static_cast<uint8_t>(sequence & 0xFF);
  out[6] = static_cast<uint8_t>(sequence >> 8);
  out[7] = static_cast<uint8_t>(timestampUs & 0xFF);
  out[8] = static_cast<uint8_t>((timestampUs >> 8) & 0xFF);
  out[9] = static_cast<uint8_t>((timestampUs >> 16) & 0xFF);
  out[10] = static_cast<uint8_t>((timestampUs >> 24) & 0xFF);
  if (payloadLen != 0) {
    memcpy(&out[kHeaderSize], payload, payloadLen);
  }
  // CRC covers everything after the sync bytes: version .. end of payload.
  const uint16_t crc = crc16(&out[2], kHeaderSize - 2 + payloadLen);
  out[kHeaderSize + payloadLen] = static_cast<uint8_t>(crc & 0xFF);
  out[kHeaderSize + payloadLen + 1] = static_cast<uint8_t>(crc >> 8);
  return kHeaderSize + payloadLen + kCrcSize;
}

void FrameParser::reset() {
  state_ = St::kSync0;
  idx_ = 0;
  total_ = 0;
  crcErrors_ = 0;
  frameErrors_ = 0;
}

bool FrameParser::feed(uint8_t byte) {
  switch (state_) {
    case St::kSync0:
      if (byte == kSync0) {
        raw_[0] = byte;
        state_ = St::kSync1;
      }
      return false;

    case St::kSync1:
      if (byte == kSync1) {
        raw_[1] = byte;
        idx_ = 2;
        state_ = St::kHeader;
      } else {
        // A lone 0x55 that is not followed by 0xAA might itself be the
        // first byte of a real preamble.
        state_ = (byte == kSync0) ? St::kSync1 : St::kSync0;
      }
      return false;

    case St::kHeader:
      raw_[idx_++] = byte;
      if (idx_ == kHeaderSize) {
        const uint8_t version = raw_[2];
        const uint8_t payloadLen = raw_[4];
        if (version != kProtocolVersion || payloadLen > kMaxPayload) {
          ++frameErrors_;
          state_ = St::kSync0;
          return false;
        }
        total_ = kHeaderSize + payloadLen + kCrcSize;
        state_ = St::kBody;
      }
      return false;

    case St::kBody:
      raw_[idx_++] = byte;
      if (idx_ < total_) {
        return false;
      }
      state_ = St::kSync0;
      {
        const size_t payloadLen = total_ - kHeaderSize - kCrcSize;
        const uint16_t expected = crc16(&raw_[2], kHeaderSize - 2 + payloadLen);
        const uint16_t received =
            static_cast<uint16_t>(raw_[total_ - 2]) |
            (static_cast<uint16_t>(raw_[total_ - 1]) << 8);
        if (expected != received) {
          ++crcErrors_;
          return false;
        }
        frame_.version = raw_[2];
        frame_.type = raw_[3];
        frame_.payloadLen = static_cast<uint8_t>(payloadLen);
        frame_.sequence =
            static_cast<uint16_t>(raw_[5]) | (static_cast<uint16_t>(raw_[6]) << 8);
        frame_.timestampUs = static_cast<uint32_t>(raw_[7]) |
                             (static_cast<uint32_t>(raw_[8]) << 8) |
                             (static_cast<uint32_t>(raw_[9]) << 16) |
                             (static_cast<uint32_t>(raw_[10]) << 24);
        memcpy(frame_.payload, &raw_[kHeaderSize], payloadLen);
      }
      return true;
  }
  return false;
}

}  // namespace proto
}  // namespace veio
