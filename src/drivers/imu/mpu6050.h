/**
 * @file mpu6050.h
 * @brief MPU6050 IMU driver with fully configurable mounting orientation.
 *
 * Raw sensor-frame values are preserved; a configurable axis remap
 * (source axis + inversion per vehicle axis) and zero-offset calibration
 * produce vehicle-frame values. Both raw and corrected values are
 * published so the calibration page can display them side by side.
 */
#pragma once

#include <Arduino.h>

#include "core/types.h"

namespace vcm {

class Mpu6050 {
 public:
  struct Orientation {
    uint8_t srcX = 0, srcY = 1, srcZ = 2;  ///< sensor axis feeding each vehicle axis
    bool invX = false, invY = false, invZ = false;
  };

  bool begin();  ///< probes 0x68 and 0x69 (AD0 strap)
  uint8_t address() const { return addr_; }
  uint8_t whoAmI() const { return whoAmI_; }

  /// Read and process one sample. Call at 100 Hz.
  /// @param orient mounting orientation from configuration
  /// @param filterHz low-pass for pitch/roll estimation
  bool sample(const Orientation& orient, float filterHz);

  /// Capture current readings as zero offsets (vehicle stationary).
  void zero();
  void clearZero();

  // raw sensor frame
  const float* rawAccel() const { return rawAccel_; }  ///< g
  const float* rawGyro() const { return rawGyro_; }    ///< deg/s
  // corrected vehicle frame
  const float* accel() const { return accel_; }
  const float* gyro() const { return gyro_; }
  float pitchDeg() const { return pitch_; }
  float rollDeg() const { return roll_; }
  float yawRateDps() const { return gyro_[2]; }
  float tempC() const { return tempC_; }
  SensorHealth health() const { return health_; }

 private:
  bool readRegisters(int16_t out[7]);
  bool probe(uint8_t addr);

  uint8_t addr_ = 0x68;
  uint8_t whoAmI_ = 0;
  float rawAccel_[3] = {0, 0, 0}, rawGyro_[3] = {0, 0, 0};
  float accel_[3] = {0, 0, 0}, gyro_[3] = {0, 0, 0};
  float accelZero_[3] = {0, 0, 0}, gyroZero_[3] = {0, 0, 0};
  float pitch_ = 0, roll_ = 0, tempC_ = 0;
  SensorHealth health_ = SensorHealth::NOT_PRESENT;
  uint8_t failCount_ = 0;
  bool loggedMissing_ = false;
};

extern Mpu6050 imu;

}  // namespace vcm
