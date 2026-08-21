#include "drivers/imu/mpu6050.h"

#include <Wire.h>

#include "services/logger.h"

namespace vcm {

static constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
static constexpr uint8_t REG_CONFIG = 0x1A;
static constexpr uint8_t REG_WHO_AM_I = 0x75;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr float ACCEL_LSB_PER_G = 16384.0f;  ///< +/-2g range
static constexpr float GYRO_LSB_PER_DPS = 131.0f;   ///< +/-250 dps range

bool Mpu6050::probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(REG_WHO_AM_I);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  const uint8_t id = Wire.read();
  // MPU6050=0x68, MPU6500=0x70, MPU9250=0x71, some clones echo the address
  const bool ok = (id == 0x68 || id == 0x70 || id == 0x71 || id == 0x73 ||
                   id == addr);
  if (ok) LOGI("IMU", "WHO_AM_I=0x%02X at 0x%02X", id, addr);
  return ok;
}

bool Mpu6050::begin() {
  const uint8_t candidates[] = {0x68, 0x69};
  bool found = false;
  for (uint8_t a : candidates) {
    if (probe(a)) {
      addr_ = a;
      found = true;
      break;
    }
  }
  if (!found) {
    // Some modules NACK WHO_AM_I but still wake on PWR_MGMT_1.
    for (uint8_t a : candidates) {
      Wire.beginTransmission(a);
      Wire.write(REG_PWR_MGMT_1);
      Wire.write(0x00);
      if (Wire.endTransmission() == 0) {
        addr_ = a;
        found = true;
        LOGI("IMU", "woke device at 0x%02X (no WHO_AM_I)", a);
        break;
      }
    }
  }
  if (!found) {
    health_ = SensorHealth::NOT_PRESENT;
    LOGW("IMU", "MPU6050 not detected at 0x68/0x69");
    return false;
  }
  Wire.beginTransmission(addr_);
  Wire.write(REG_PWR_MGMT_1);
  Wire.write(0x00);  // wake, internal oscillator
  Wire.endTransmission();
  Wire.beginTransmission(addr_);
  Wire.write(REG_CONFIG);
  Wire.write(0x03);  // DLPF ~44 Hz
  Wire.endTransmission();
  health_ = SensorHealth::OK;
  failCount_ = 0;
  LOGI("IMU", "IMU online at 0x%02X", addr_);
  return true;
}

bool Mpu6050::readRegisters(int16_t out[7]) {
  Wire.beginTransmission(addr_);
  Wire.write(REG_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr_, 14) != 14) return false;
  for (int i = 0; i < 7; ++i) {
    out[i] = (int16_t)((Wire.read() << 8) | Wire.read());
  }
  return true;
}

bool Mpu6050::sample(const Orientation& o, float filterHz) {
  if (health_ == SensorHealth::NOT_PRESENT) return false;

  int16_t r[7];
  if (!readRegisters(r)) {
    if (failCount_ < 255) failCount_++;
    if (failCount_ > 10) health_ = SensorHealth::FAULT;
    return false;
  }
  failCount_ = 0;
  health_ = SensorHealth::OK;

  rawAccel_[0] = r[0] / ACCEL_LSB_PER_G;
  rawAccel_[1] = r[1] / ACCEL_LSB_PER_G;
  rawAccel_[2] = r[2] / ACCEL_LSB_PER_G;
  tempC_ = r[3] / 340.0f + 36.53f;
  rawGyro_[0] = r[4] / GYRO_LSB_PER_DPS;
  rawGyro_[1] = r[5] / GYRO_LSB_PER_DPS;
  rawGyro_[2] = r[6] / GYRO_LSB_PER_DPS;

  // Remap sensor axes to vehicle axes per configured mounting orientation
  const uint8_t src[3] = {(uint8_t)min((int)o.srcX, 2),
                          (uint8_t)min((int)o.srcY, 2),
                          (uint8_t)min((int)o.srcZ, 2)};
  const float sign[3] = {o.invX ? -1.0f : 1.0f, o.invY ? -1.0f : 1.0f,
                         o.invZ ? -1.0f : 1.0f};
  for (int i = 0; i < 3; ++i) {
    accel_[i] = rawAccel_[src[i]] * sign[i] - accelZero_[i];
    gyro_[i] = rawGyro_[src[i]] * sign[i] - gyroZero_[i];
  }

  // Accelerometer-derived pitch/roll with low-pass filtering (100 Hz rate)
  const float dt = 1.0f / 100.0f;
  const float alpha = dt * filterHz * 6.2831853f /
                      (1.0f + dt * filterHz * 6.2831853f);
  const float pitchRaw =
      atan2f(-accel_[0], sqrtf(accel_[1] * accel_[1] + accel_[2] * accel_[2])) *
      57.2958f;
  const float rollRaw = atan2f(accel_[1], accel_[2]) * 57.2958f;
  pitch_ += alpha * (pitchRaw - pitch_);
  roll_ += alpha * (rollRaw - roll_);
  return true;
}

void Mpu6050::zero() {
  // Capture the current corrected values (plus existing offsets) as zero.
  for (int i = 0; i < 3; ++i) {
    accelZero_[i] += accel_[i];
    gyroZero_[i] += gyro_[i];
  }
  // Keep gravity on Z so pitch/roll remain meaningful
  accelZero_[2] -= 1.0f;
  LOGI("IMU", "Zero-offset calibration captured");
}

void Mpu6050::clearZero() {
  memset(accelZero_, 0, sizeof(accelZero_));
  memset(gyroZero_, 0, sizeof(gyroZero_));
}

}  // namespace vcm
