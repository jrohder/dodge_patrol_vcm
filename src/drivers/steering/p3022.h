/**
 * @file p3022.h
 * @brief P3022 absolute hall angle encoder (SPI, 4096 counts/rev).
 *
 * IMPORTANT ROLE DISTINCTION (see HARDWARE.md):
 * This sensor measures the DRIVER'S STEERING WHEEL INPUT - what steering
 * the child is requesting in manual mode. It is NOT the actuator feedback;
 * that is the Firgelli potentiometer (firgelli_feedback.h).
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class P3022 {
 public:
  void begin();

  /// Read the raw absolute angle (0..4095). Call at up to 200 Hz.
  /// Returns false on SPI failure (all-zero/all-one frames).
  bool read(uint16_t& counts);

  uint16_t lastCounts() const { return lastCounts_; }
  SensorHealth health() const { return health_; }

  /**
   * @brief Normalize raw counts into a steering wheel input.
   * @param counts raw encoder counts
   * @param minC,centerC,maxC calibrated end/center points (config)
   * @param deadbandPct inputs closer to center than this are zero
   * @param invert flip direction
   * @return -1..+1 steering request (negative = left)
   */
  static float normalize(uint16_t counts, int minC, int centerC, int maxC,
                         float deadbandPct, bool invert);

 private:
  uint16_t lastCounts_ = 2048;
  SensorHealth health_ = SensorHealth::NOT_PRESENT;
  uint8_t badReads_ = 0;
};

}  // namespace vcm
