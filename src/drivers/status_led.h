/**
 * @file status_led.h
 * @brief WS2812 RGB status LED reflecting the safety state machine.
 *
 * Blue=boot, green=ready, yellow=calibrating, purple=OTA, red=fault.
 * Blink patterns signal Nano/RC timeouts and OTA progress.
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class StatusLed {
 public:
  void begin();

  /// Update the LED for the current state. Call at ~10 Hz.
  /// @param nanoOnline blink when the Nano link is down
  void update(VehicleState state, bool nanoOnline);

 private:
  void show(uint8_t r, uint8_t g, uint8_t b);
  bool blinkOn_ = false;
  uint32_t lastToggle_ = 0;
};

extern StatusLed statusLed;

}  // namespace vcm
