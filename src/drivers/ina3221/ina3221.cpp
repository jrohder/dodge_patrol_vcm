#include "drivers/ina3221/ina3221.h"

#include <Wire.h>

#include "services/logger.h"

namespace vcm {

static constexpr uint8_t ADDR = 0x40;
static constexpr uint8_t REG_CONFIG = 0x00;
static constexpr uint8_t REG_SHUNT_V_1 = 0x01;  // shunt/bus pairs: 1,3,5 / 2,4,6
static constexpr uint8_t REG_MANUF_ID = 0xFE;

bool Ina3221::readReg(uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)ADDR, 2) != 2) return false;
  value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

bool Ina3221::begin() {
  uint16_t id = 0;
  if (!readReg(REG_MANUF_ID, id) || id != 0x5449) {  // 'TI'
    health_ = SensorHealth::NOT_PRESENT;
    LOGW("INA", "INA3221 not detected at 0x%02X", ADDR);
    return false;
  }
  // All channels enabled, 16 sample average, 1.1 ms conversions, continuous
  Wire.beginTransmission(ADDR);
  Wire.write(REG_CONFIG);
  Wire.write(0x75);
  Wire.write(0x27);
  Wire.endTransmission();
  health_ = SensorHealth::OK;
  LOGI("INA", "INA3221 online");
  return true;
}

bool Ina3221::sample(float shuntMilliOhm, float offsetA, float scale) {
  if (health_ == SensorHealth::NOT_PRESENT) return false;

  bool ok = true;
  for (int ch = 0; ch < 3; ++ch) {
    uint16_t shuntRaw = 0, busRaw = 0;
    ok &= readReg(REG_SHUNT_V_1 + ch * 2, shuntRaw);
    ok &= readReg(REG_SHUNT_V_1 + ch * 2 + 1, busRaw);
    if (!ok) break;
    // Shunt voltage: 40 uV/LSB, register is value<<3
    const float shuntUv = (float)((int16_t)shuntRaw >> 3) * 40.0f;
    const float amps = (shuntMilliOhm > 0.01f)
                           ? (shuntUv / 1000.0f) / shuntMilliOhm
                           : 0.0f;
    currentA_[ch] = amps * scale + offsetA;
    if (ch == 0) {
      // Bus voltage: 8 mV/LSB, register is value<<3
      busVoltage_ = (float)((int16_t)busRaw >> 3) * 0.008f;
    }
  }
  if (!ok) {
    if (failCount_ < 255) failCount_++;
    if (failCount_ > 10) health_ = SensorHealth::FAULT;
    return false;
  }
  failCount_ = 0;
  health_ = SensorHealth::OK;
  return true;
}

}  // namespace vcm
