#include "drivers/status_led.h"

#include "core/pins.h"

namespace vcm {

StatusLed statusLed;

void StatusLed::begin() { show(0, 0, 32); }  // blue: booting

void StatusLed::show(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(pins::STATUS_LED, r, g, b);
}

void StatusLed::update(VehicleState state, bool nanoOnline) {
  const uint32_t now = millis();
  const uint32_t period = nanoOnline ? 500 : 150;  // fast blink = link down
  if (now - lastToggle_ >= period) {
    lastToggle_ = now;
    blinkOn_ = !blinkOn_;
  }

  uint8_t r = 0, g = 0, b = 0;
  bool blink = !nanoOnline;
  switch (state) {
    case VehicleState::BOOT:
    case VehicleState::INITIALIZING: r = 0; g = 0; b = 40; break;   // blue
    case VehicleState::NOT_CALIBRATED: r = 40; g = 20; b = 0; blink = true; break;  // orange blink
    case VehicleState::READY: r = 0; g = 40; b = 0; break;          // green
    case VehicleState::DRIVING: r = 0; g = 40; b = 10; break;       // green/cyan
    case VehicleState::CALIBRATION:
    case VehicleState::DIAGNOSTIC: r = 40; g = 40; b = 0; break;    // yellow
    case VehicleState::OTA_UPDATE: r = 30; g = 0; b = 40; blink = true; break;  // purple blink
    case VehicleState::FAULT:
    case VehicleState::ESTOP: r = 60; g = 0; b = 0; blink = true; break;  // red blink
  }
  if (blink && !blinkOn_) { r = g = b = 0; }
  show(r, g, b);
}

}  // namespace vcm
