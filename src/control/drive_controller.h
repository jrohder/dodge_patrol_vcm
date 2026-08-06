/**
 * @file drive_controller.h
 * @brief Converts per-wheel target speeds into BTS7960 PWM commands.
 *
 * Higher-level control (VehicleDynamics) never touches GPIO; it commands
 * wheel speeds here, and this layer handles feedforward conversion,
 * stiction compensation, current limiting (warn / limit / trip levels)
 * and brake mode. Wheel-speed feedback is compared against requests to
 * enable future closed-loop wheel control and slip handling.
 */
#pragma once

#include "drivers/motor/bts7960.h"

namespace vcm {

class DriveController {
 public:
  void begin();

  /**
   * @brief One 200 Hz output step.
   * @param leftTarget,rightTarget wheel speed targets (m/s, signed)
   * @param leftActual,rightActual measured wheel speeds (m/s, signed)
   * @param leftCurrentA,rightCurrentA measured motor currents
   * @param outputEnabled false = outputs forced safe (stop mode)
   */
  void step(float leftTarget, float rightTarget, float leftActual,
            float rightActual, float leftCurrentA, float rightCurrentA,
            bool outputEnabled);

  void refreshConfig();

  float leftPwm() const { return leftPwm_; }
  float rightPwm() const { return rightPwm_; }

  Bts7960& leftMotor() { return left_; }
  Bts7960& rightMotor() { return right_; }

 private:
  float speedToPwm(float speedTarget) const;
  float applyCurrentLimit(float pwm, float currentA, float& derate) const;

  Bts7960 left_;
  Bts7960 right_;
  float leftPwm_ = 0.0f, rightPwm_ = 0.0f;
  float leftDerate_ = 0.0f, rightDerate_ = 0.0f;
  uint32_t lastWarnMs_ = 0;
};

extern DriveController drive;

}  // namespace vcm
