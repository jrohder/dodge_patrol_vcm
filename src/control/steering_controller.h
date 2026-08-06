/**
 * @file steering_controller.h
 * @brief Closed-loop steering actuator controller (200 Hz).
 *
 * Requested position (from the active control source or calibration)
 * minus actual position (Firgelli feedback potentiometer) = steering
 * error, driven through a PID to the steering BTS7960. Enforces soft
 * limits, rate limiting, deadband, overcurrent trip and feedback-loss
 * detection. All tuning comes from configuration and can be changed
 * live from the web UI.
 */
#pragma once

#include "control/pid.h"
#include "drivers/motor/bts7960.h"
#include "drivers/steering/firgelli_feedback.h"
#include "drivers/steering/p3022.h"

namespace vcm {

class SteeringController {
 public:
  void begin();

  /**
   * @brief One 200 Hz control step.
   * @param steeringCmd -1..+1 steering request from the arbiter
   * @param steeringCurrentA measured actuator current
   * @param outputEnabled false = force actuator off (safety)
   */
  void step(float steeringCmd, float steeringCurrentA, bool outputEnabled);

  /// Refresh cached configuration (called when config revision changes).
  void refreshConfig();

  float estimatedAngleDeg() const { return actualAngleDeg_; }
  float actualPct() const { return actualPct_; }

 private:
  float commandToTargetPct(float cmd) const;

  Bts7960 motor_;
  FirgelliFeedback feedback_;
  P3022 wheelEncoder_;
  Pid pid_;
  float targetPct_ = 50.0f;    ///< rate-limited target
  float actualPct_ = 50.0f;
  float actualAngleDeg_ = 0.0f;
  uint32_t lastStepUs_ = 0;

 public:
  FirgelliFeedback& feedbackSensor() { return feedback_; }
  P3022& wheelSensor() { return wheelEncoder_; }
  Bts7960& actuator() { return motor_; }
};

extern SteeringController steering;

}  // namespace vcm
