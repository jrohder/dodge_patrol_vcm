/**
 * @file speed_calculator.h
 * @brief Converts raw wheel sensor data into per-wheel and vehicle speeds.
 *
 * Left and right wheels are ALWAYS maintained independently; vehicle speed
 * is a derived measurement. Includes slip/anomaly detection against the
 * geometric prediction from the vehicle model.
 *
 * Pure logic - no hardware dependencies; unit-tested on the host.
 */
#pragma once

#include <cstdint>

#include "control/vehicle_model.h"

namespace vcm {

struct WheelInput {
  float freqHz = 0.0f;   ///< hall pulse frequency
  int8_t direction = 0;  ///< -1 reverse, 0 stopped, +1 forward
  uint32_t count = 0;    ///< cumulative pulse counter
};

struct SpeedOutput {
  float leftSpeed = 0.0f;   ///< m/s, signed
  float rightSpeed = 0.0f;  ///< m/s, signed
  float leftRpm = 0.0f, rightRpm = 0.0f;
  float vehicleSpeed = 0.0f;   ///< m/s, signed (mean of wheels)
  float acceleration = 0.0f;   ///< m/s^2 (from speed derivative)
  float odometerM = 0.0f;      ///< total distance
  float tripM = 0.0f;          ///< resettable distance
  bool slipDetected = false;   ///< unexplained left/right difference
};

class SpeedCalculator {
 public:
  /// Per-wheel calibration factors (from speed calibration wizard).
  void setCalibration(float leftFactor, float rightFactor) {
    leftFactor_ = leftFactor;
    rightFactor_ = rightFactor;
  }

  /// Slip detection threshold: allowed extra L/R difference (m/s) beyond
  /// what steering geometry predicts. <=0 disables detection.
  void setSlipThreshold(float mps) { slipThreshold_ = mps; }

  void resetTrip() { trip_ = 0.0f; }

  /**
   * @brief Process one sensor sample.
   * @param model vehicle model (wheel size, counts/rev)
   * @param left/right raw wheel inputs from the Nano
   * @param steeringAngleDeg current road-wheel angle (for slip prediction)
   * @param dt seconds since last update
   */
  SpeedOutput update(const VehicleModel& model, const WheelInput& left,
                     const WheelInput& right, float steeringAngleDeg,
                     float dt);

 private:
  float leftFactor_ = 1.0f, rightFactor_ = 1.0f;
  float slipThreshold_ = 0.0f;
  float odometer_ = 0.0f, trip_ = 0.0f;
  float lastVehicleSpeed_ = 0.0f;
  float accelFiltered_ = 0.0f;
  uint32_t lastLeftCount_ = 0, lastRightCount_ = 0;
  bool haveCounts_ = false;
};

}  // namespace vcm
