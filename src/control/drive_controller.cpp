#include "control/drive_controller.h"

#include "config/config_registry.h"
#include "core/pins.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"

namespace vcm {

DriveController drive;

void DriveController::begin() {
  left_ = Bts7960("LEFT", pins::LEFT_RPWM, pins::LEFT_LPWM, pins::CH_LEFT_R,
                  pins::CH_LEFT_L);
  right_ = Bts7960("RIGHT", pins::RIGHT_RPWM, pins::RIGHT_LPWM,
                   pins::CH_RIGHT_R, pins::CH_RIGHT_L);
  left_.begin();
  right_.begin();
  refreshConfig();
  LOGI("DRIVE", "Drive controller ready (200 Hz)");
}

void DriveController::refreshConfig() {
  const float maxPwm = config.f(DRV_MAX_PWM);
  const auto mode = config.i(DRV_BRAKE_MODE) == 0 ? Bts7960::StopMode::COAST
                                                  : Bts7960::StopMode::BRAKE;
  left_.setMaxPct(maxPwm);
  right_.setMaxPct(maxPwm);
  left_.setStopMode(mode);
  right_.setStopMode(mode);
}

float DriveController::speedToPwm(float speedTarget) const {
  // Open-loop feedforward: linear speed->PWM with stiction compensation.
  // (Wheel-speed feedback is captured in telemetry for future closed loop.)
  const float maxSpeed = config.f(DRV_MAX_SPEED);
  if (maxSpeed <= 0.01f || fabsf(speedTarget) < 0.005f) return 0.0f;
  const float minPwm = config.f(DRV_MIN_PWM);
  const float maxPwm = config.f(DRV_MAX_PWM);
  const float mag = fabsf(speedTarget) / maxSpeed;
  const float pwm = minPwm + mag * (maxPwm - minPwm);
  return (speedTarget > 0 ? pwm : -pwm);
}

float DriveController::applyCurrentLimit(float pwm, float currentA,
                                         float& derate) const {
  const float limit = config.f(CUR_DRIVE_LIMIT);
  // Progressive derating above the limit level; 50%/s recovery below it
  if (currentA > limit && limit > 0.1f) {
    derate = min(derate + 0.02f, 0.8f);  // up to 80% reduction
  } else {
    derate = max(derate - 0.0025f, 0.0f);
  }
  return pwm * (1.0f - derate);
}

void DriveController::step(float leftTarget, float rightTarget,
                           float leftActual, float rightActual,
                           float leftCurrentA, float rightCurrentA,
                           bool outputEnabled) {
  (void)leftActual;
  (void)rightActual;

  // Trip level: stop the affected output and raise a fault
  if (leftCurrentA > config.f(CUR_DRIVE_TRIP)) safety.raiseFault(FLT_LEFT_OVERCURRENT);
  if (rightCurrentA > config.f(CUR_DRIVE_TRIP)) safety.raiseFault(FLT_RIGHT_OVERCURRENT);

  // Warning level: log (rate limited)
  const float warn = config.f(CUR_DRIVE_WARN);
  if ((leftCurrentA > warn || rightCurrentA > warn) &&
      millis() - lastWarnMs_ > 2000) {
    lastWarnMs_ = millis();
    LOGW("DRIVE", "High motor current L=%.1fA R=%.1fA", leftCurrentA,
         rightCurrentA);
  }

  // Motor test (commissioning) overrides normal targets, with limited power
  const MotorTest& test = calibration.motorTest();
  bool driveAllowed = outputEnabled && (safety.motionAllowed() ||
                                        (test.active && safety.testMotionAllowed()));

  float lPwm, rPwm;
  if (test.active && safety.testMotionAllowed()) {
    lPwm = test.left ? test.pwmPct : 0.0f;
    rPwm = test.right ? test.pwmPct : 0.0f;
  } else {
    lPwm = speedToPwm(leftTarget);
    rPwm = speedToPwm(rightTarget);
  }

  if (!driveAllowed) {
    lPwm = rPwm = 0.0f;
  }
  if (safety.faultActive(FLT_LEFT_OVERCURRENT)) lPwm = 0.0f;
  if (safety.faultActive(FLT_RIGHT_OVERCURRENT)) rPwm = 0.0f;

  lPwm = applyCurrentLimit(lPwm, leftCurrentA, leftDerate_);
  rPwm = applyCurrentLimit(rPwm, rightCurrentA, rightDerate_);

  left_.drive(lPwm);
  right_.drive(rPwm);
  leftPwm_ = lPwm;
  rightPwm_ = rPwm;
}

}  // namespace vcm
