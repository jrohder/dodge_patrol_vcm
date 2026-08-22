/**
 * @file ina3221.h
 * @brief INA3221 triple-channel current/voltage monitor driver.
 *
 * All three channels are maintained independently; their semantic meaning
 * (left drive / right drive / steering) is mapped in configuration
 * (current.chN_role), not hardwired.
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class Ina3221 {
 public:
  bool begin();  ///< probes 0x40–0x43 (A0 strap)
  uint8_t address() const { return addr_; }
  uint16_t manufId() const { return manufId_; }

  /// Read all channels. Call at 100 Hz.
  /// @param shuntMilliOhm shunt resistor value (config)
  /// @param offsetA,scale calibration applied to every channel
  bool sample(float shuntMilliOhm, float offsetA, float scale);

  float currentA(int channel) const { return currentA_[channel]; }  ///< 0..2
  float busVoltage() const { return busVoltage_; }
  SensorHealth health() const { return health_; }

 private:
  bool readReg(uint8_t addr, uint8_t reg, uint16_t& value);
  bool probe(uint8_t addr);

  uint8_t addr_ = 0x40;
  uint16_t manufId_ = 0;
  float currentA_[3] = {0, 0, 0};
  float busVoltage_ = 0.0f;
  SensorHealth health_ = SensorHealth::NOT_PRESENT;
  uint8_t failCount_ = 0;
  bool loggedMissing_ = false;
};

extern Ina3221 ina;

}  // namespace vcm
