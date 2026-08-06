/**
 * @file crc16.h
 * @brief CRC16-CCITT (0x1021, init 0xFFFF) used by the Nano UART protocol.
 *
 * Pure logic - shared definition with the Nano firmware (see PROTOCOL.md).
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace vcm {

uint16_t crc16(const uint8_t* data, size_t len);

}  // namespace vcm
