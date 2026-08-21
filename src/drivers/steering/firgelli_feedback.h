/**
 * @file firgelli_feedback.h
 * @brief Firgelli actuator built-in potentiometer feedback (ADC).
 *
 * This is the PRIMARY closed-loop feedback for the steering actuator:
 * it measures the ACTUAL road-wheel steering position. The steering
 * controller compares requested vs actual position from this sensor.
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class FirgelliFeedback {
 public:
  void begin();

  /// Sample the ADC and update the filtered value. Call at 200 Hz.
  /// @param filterHz low-pass cutoff (config: steering.feedback_filter)
  /// @param validMin/validMax raw range outside which wiring is broken
  void sample(float filterHz, int validMin, int validMax);

  uint16_t raw() const { return raw_; }
  float filtered() const { return filtered_; }
  SensorHealth health() const { return health_; }

  /// Convert a raw/filtered ADC value to position % using calibration.
  static float toPct(float adc, float leftAdc, float rightAdc);

 private:
  uint16_t raw_ = 0;
  float stage1_ = 0.0f;
  float filtered_ = 0.0f;
  bool primed_ = false;
  uint8_t invalidCount_ = 0;
  SensorHealth health_ = SensorHealth::NOT_PRESENT;
};

}  // namespace vcm
