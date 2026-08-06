/**
 * @file vehicle_dynamics.h
 * @brief Top-level vehicle behavior orchestrator (100 Hz).
 *
 * Pipeline per cycle:
 *   Nano data -> input mapping -> control arbitration -> throttle shaping
 *   -> speed target + accel/decel limits -> differential steering split
 *   -> wheel speed targets (handed to DriveController at 200 Hz)
 *
 * Also owns safety monitoring of the Nano link, RC signal, battery and
 * the safety state machine tick. Drive motors NEVER follow throttle
 * directly - they follow wheel speed targets produced here.
 */
#pragma once

#include <Arduino.h>

#include "control/differential_steering.h"
#include "control/speed_calculator.h"
#include "control/vehicle_model.h"
#include "core/types.h"

namespace vcm {

class VehicleDynamics {
 public:
  void begin();

  /// One 100 Hz step. Produces wheel targets consumed by the 200 Hz
  /// motor task via targets().
  void step();

  void targets(float& left, float& right, float& steeringCmd) const {
    left = leftTarget_;
    right = rightTarget_;
    steeringCmd = steeringCmd_;
  }

  const VehicleModel& model() const { return model_; }
  void resetTrip() { speedCalc_.resetTrip(); }

 private:
  void refreshConfigIfChanged();
  float mapRcChannel(uint16_t us, bool invert) const;
  float shapeThrottle(float raw) const;
  void monitorSafety(bool rcValid, float battV);

  VehicleModel model_;
  SpeedCalculator speedCalc_;
  DifferentialSteering diff_;
  uint32_t cfgRevision_ = 0;

  float limitedSpeed_ = 0.0f;  ///< slew-limited speed target
  volatile float leftTarget_ = 0.0f, rightTarget_ = 0.0f;
  volatile float steeringCmd_ = 0.0f;
  uint32_t lastStepMs_ = 0;
};

extern VehicleDynamics dynamics;

}  // namespace vcm
