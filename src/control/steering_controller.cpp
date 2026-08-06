#include "control/steering_controller.h"

#include "config/config_registry.h"
#include "core/pins.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/telemetry.h"

namespace vcm {

SteeringController steering;

void SteeringController::begin() {
  motor_ = Bts7960("STEER", pins::STEER_RPWM, pins::STEER_LPWM,
                   pins::CH_STEER_R, pins::CH_STEER_L);
  motor_.begin();
  feedback_.begin();
  wheelEncoder_.begin();
  refreshConfig();
  LOGI("STEER", "Steering controller ready (200 Hz)");
}

void SteeringController::refreshConfig() {
  Pid::Gains g;
  g.kp = config.f(STR_PID_KP);
  g.ki = config.f(STR_PID_KI);
  g.kd = config.f(STR_PID_KD);
  g.iMax = config.f(STR_PID_IMAX);
  g.outMax = config.f(STR_MAX_PWM);
  pid_.setGains(g);
  motor_.setMaxPct(config.f(STR_MAX_PWM));
  motor_.setInverted(config.b(STR_INVERT));
}

float SteeringController::commandToTargetPct(float cmd) const {
  // Map -1..+1 into calibrated soft-limit range around the trimmed center
  const SteeringCalData& cal = calibration.steeringCal();
  if (!cal.valid) return 50.0f;
  float leftAdc, rightAdc;
  calibration.softLimits(leftAdc, rightAdc);
  const float centerAdc = calibration.centerAdc();
  const float targetAdc = (cmd >= 0.0f)
                              ? centerAdc + cmd * (rightAdc - centerAdc)
                              : centerAdc + (-cmd) * (leftAdc - centerAdc);
  return FirgelliFeedback::toPct(targetAdc, cal.leftAdc, cal.rightAdc);
}

void SteeringController::step(float steeringCmd, float steeringCurrentA,
                              bool outputEnabled) {
  const uint32_t nowUs = micros();
  float dt = (lastStepUs_ == 0) ? 0.005f : (nowUs - lastStepUs_) * 1e-6f;
  lastStepUs_ = nowUs;
  dt = constrain(dt, 0.001f, 0.05f);

  // --- sensors ------------------------------------------------------------
  feedback_.sample(config.f(STR_FEEDBACK_FILTER), config.i(STR_FEEDBACK_MIN),
                   config.i(STR_FEEDBACK_MAX));
  uint16_t wheelRaw = wheelEncoder_.lastCounts();
  wheelEncoder_.read(wheelRaw);

  const SteeringCalData& cal = calibration.steeringCal();
  actualPct_ = cal.valid ? FirgelliFeedback::toPct(feedback_.filtered(),
                                                   cal.leftAdc, cal.rightAdc)
                         : 50.0f;
  const float maxAngle = config.f(STR_MAX_ANGLE);
  actualAngleDeg_ = (actualPct_ - 50.0f) / 50.0f * maxAngle;

  // --- safety checks --------------------------------------------------------
  if (feedback_.health() == SensorHealth::FAULT) {
    safety.raiseFault(FLT_STEER_FEEDBACK);
  } else if (feedback_.health() == SensorHealth::OK) {
    safety.clearFault(FLT_STEER_FEEDBACK);
  }
  if (steeringCurrentA > config.f(CUR_STEER_TRIP)) {
    safety.raiseFault(FLT_STEER_OVERCURRENT);
  }
  if (!cal.valid) {
    safety.raiseFault(FLT_STEER_NOT_CAL);
  }

  // --- calibration wizard owns the actuator --------------------------------
  if (calibration.steeringCalActive()) {
    const float calPwm =
        calibration.steeringCalStep(feedback_.filtered(), steeringCurrentA);
    motor_.drive(calPwm);
    telemetry.update([&](VehicleTelemetry& t) {
      t.steering.feedbackRaw = feedback_.raw();
      t.steering.feedbackFiltered = feedback_.filtered();
      t.steering.actualPct = actualPct_;
      t.steering.pwmPct = calPwm;
      t.steering.currentA = steeringCurrentA;
    });
    return;
  }

  // --- normal closed loop ----------------------------------------------------
  float requestPct;
  if (calibration.manualSteeringActive() && safety.testMotionAllowed()) {
    requestPct = calibration.manualSteeringTarget();
  } else {
    requestPct = commandToTargetPct(constrain(steeringCmd, -1.0f, 1.0f));
  }

  // Rate-limit the target so steering moves smoothly
  const float maxStep = config.f(STR_RATE_LIMIT) * dt;
  targetPct_ += constrain(requestPct - targetPct_, -maxStep, maxStep);

  const bool driveAllowed = outputEnabled && cal.valid &&
                            feedback_.health() == SensorHealth::OK &&
                            !safety.faultActive(FLT_STEER_OVERCURRENT) &&
                            (safety.motionAllowed() ||
                             safety.testMotionAllowed());

  float out = 0.0f;
  if (driveAllowed) {
    out = pid_.update(targetPct_, actualPct_, dt);
    if (fabsf(targetPct_ - actualPct_) < config.f(STR_DEADBAND)) out = 0.0f;
    // Enforce soft limits: never drive further toward a limit already reached
    float leftAdc, rightAdc;
    calibration.softLimits(leftAdc, rightAdc);
    const float leftPct =
        FirgelliFeedback::toPct(leftAdc, cal.leftAdc, cal.rightAdc);
    const float rightPct =
        FirgelliFeedback::toPct(rightAdc, cal.leftAdc, cal.rightAdc);
    if ((actualPct_ <= leftPct && out < 0) ||
        (actualPct_ >= rightPct && out > 0)) {
      out = 0.0f;
    }
  } else {
    pid_.reset();
  }
  motor_.drive(out);

  // --- telemetry -------------------------------------------------------------
  telemetry.update([&](VehicleTelemetry& t) {
    t.steering.requestedPct = targetPct_;
    t.steering.actualPct = actualPct_;
    t.steering.requestedAngleDeg = (targetPct_ - 50.0f) / 50.0f * maxAngle;
    t.steering.actualAngleDeg = actualAngleDeg_;
    t.steering.error = pid_.error();
    t.steering.pTerm = pid_.pTerm();
    t.steering.iTerm = pid_.iTerm();
    t.steering.dTerm = pid_.dTerm();
    t.steering.output = pid_.output();
    t.steering.pwmPct = motor_.currentPct();
    t.steering.feedbackRaw = feedback_.raw();
    t.steering.feedbackFiltered = feedback_.filtered();
    t.steering.wheelInputRaw = wheelRaw;
    t.steering.currentA = steeringCurrentA;
    t.steering.calibrated = cal.valid;
  });
}

}  // namespace vcm
