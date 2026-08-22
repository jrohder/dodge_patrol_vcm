#include "drivers/ina3221/ina3221.h"

#include <Wire.h>

#include "services/logger.h"

namespace vcm {

Ina3221 ina;

static constexpr uint8_t REG_CONFIG = 0x00;
static constexpr uint8_t REG_SHUNT_V_1 = 0x01;
static constexpr uint8_t REG_MANUF_ID = 0xFE;

bool Ina3221::readReg(uint8_t addr, uint8_t reg, uint16_t& value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 2) != 2) return false;
  value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

bool Ina3221::probe(uint8_t addr) {
  uint16_t id = 0;
  if (!readReg(addr, REG_MANUF_ID, id)) return false;
  manufId_ = id;
  return id == 0x5449;  // 'TI'
}

bool Ina3221::begin() {
  for (uint8_t a = 0x40; a <= 0x43; ++a) {
    if (!probe(a)) continue;
    addr_ = a;
    Wire.beginTransmission(addr_);
    Wire.write(REG_CONFIG);
    Wire.write(0x75);
    Wire.write(0x27);
    Wire.endTransmission();
    health_ = SensorHealth::OK;
    failCount_ = 0;
    LOGI("INA", "INA3221 online at 0x%02X", addr_);
    return true;
  }
    health_ = SensorHealth::NOT_PRESENT;
    if (!loggedMissing_) {
      loggedMissing_ = true;
      LOGW("INA", "INA3221 not detected at 0x40-0x43 (retrying in background)");
    }
    return false;
}

bool Ina3221::sample(float shuntMilliOhm, float offsetA, float scale) {
  if (health_ == SensorHealth::NOT_PRESENT) return false;

  bool ok = true;
  for (int ch = 0; ch < 3; ++ch) {
    uint16_t shuntRaw = 0, busRaw = 0;
    ok &= readReg(addr_, REG_SHUNT_V_1 + ch * 2, shuntRaw);
    ok &= readReg(addr_, REG_SHUNT_V_1 + ch * 2 + 1, busRaw);
    if (!ok) break;
    const float shuntUv = (float)((int16_t)shuntRaw >> 3) * 40.0f;
    const float amps = (shuntMilliOhm > 0.01f)
                           ? (shuntUv / 1000.0f) / shuntMilliOhm
                           : 0.0f;
    currentA_[ch] = amps * scale + offsetA;
    if (ch == 0) {
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
