#include "control/steering_controller.h"

#include <cmath>

#include "config/config_registry.h"
#include "control/steering_compensation.h"
#include "core/pins.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/steering_characterization.h"
#include "services/steering_recorder.h"
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
  g.dFilterHz = config.f(STR_D_FILT);
  pid_.setGains(g);
  motor_.setMaxPct(config.f(STR_MAX_PWM));
  motor_.setInverted(config.b(STR_INVERT));
  compCfg_.startLeft = config.f(STR_MIN_START_L);
  compCfg_.startRight = config.f(STR_MIN_START_R);
  compCfg_.holdLeft = config.f(STR_MIN_HOLD_L);
  compCfg_.holdRight = config.f(STR_MIN_HOLD_R);
  compCfg_.hysteresis = config.f(STR_PWM_HYST);
  compCfg_.holdError = config.f(STR_HOLD_TH);
  compCfg_.reengageError = config.f(STR_REENGAGE);
  cfgRev_ = config.revision();
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

void SteeringController::recordSample(float setpoint, float pwm,
                                      bool outputEnabled, float currentA,
                                      bool currentValid) {
  SteeringDiagnosticSample s;
  s.timestamp_us = micros();
  s.setpoint = setpoint;
  s.raw_feedback = (float)feedback_.raw();
  s.filtered_feedback = actualPct_;
  s.error = setpoint - actualPct_;
  s.pwm = pwm;
  s.direction = (pwm > 0.5f) ? 1 : (pwm < -0.5f) ? -1 : 0;
  s.p_term = pid_.pTerm();
  s.i_term = pid_.iTerm();
  s.d_term = pid_.dTerm();
  s.feedforward = feedforward_;
  s.actuator_velocity = filtVel_;
  s.control_state = controlState_;
  s.output_enabled = outputEnabled;
  s.current_a = currentA;
  s.current_valid = currentValid;
  steerDiag.push(s);
}

void SteeringController::emitEdgeEvents() {
  if (controlState_ != lastState_) {
    steerDiag.pushEvent(SteerDiagEvent::MODE_CHANGE,
                        (int16_t)controlState_);
    lastState_ = controlState_;
  }
  const int8_t dir = (appliedPwm_ > 0.5f) ? 1 : (appliedPwm_ < -0.5f) ? -1 : 0;
  if (dir != 0 && lastDir_ != 0 && dir != lastDir_) {
    steerDiag.pushEvent(SteerDiagEvent::DIR_CHANGE, dir);
  }
  if (dir != 0) lastDir_ = dir;
}

void SteeringController::step(float steeringCmd, float steeringCurrentA,
                              bool outputEnabled, SensorHealth currentHealth,
                              uint32_t loopMisses) {
  loopMisses_ = loopMisses;
  if (config.revision() != cfgRev_) refreshConfig();

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

  if (!velPrimed_) {
    lastActualPct_ = actualPct_;
    lastTargetPct_ = targetPct_;
    velPrimed_ = true;
  }
  rawVel_ = (actualPct_ - lastActualPct_) / dt;
  lastActualPct_ = actualPct_;
  const float velHz = config.f(STR_D_FILT);
  const float velAlpha =
      (velHz > 0.0f)
          ? constrain(dt * velHz * 6.2831853f / (1.0f + dt * velHz * 6.2831853f),
                      0.0f, 1.0f)
          : 1.0f;
  filtVel_ += velAlpha * (rawVel_ - filtVel_);

  const bool currentValid = currentHealth == SensorHealth::OK;
  const bool calActive = calibration.steeringCalActive();
  const bool charActive = steerChar.active();

  // --- safety checks --------------------------------------------------------
  if (feedback_.health() == SensorHealth::FAULT) {
    safety.raiseFault(FLT_STEER_FEEDBACK);
  } else if (feedback_.health() == SensorHealth::OK) {
    safety.clearFault(FLT_STEER_FEEDBACK);
  }
  if (currentValid && steeringCurrentA > config.f(CUR_STEER_TRIP)) {
    safety.raiseFault(FLT_STEER_OVERCURRENT);
  }
  if (!cal.valid) {
    safety.raiseFault(FLT_STEER_NOT_CAL);
  }

  if (calActive && !lastCalActive_)
    steerDiag.pushEvent(SteerDiagEvent::CAL_START, 0);
  if (!calActive && lastCalActive_)
    steerDiag.pushEvent(SteerDiagEvent::CAL_FINISH, 0);
  lastCalActive_ = calActive;

  const bool faultNow = safety.faultActive(FLT_STEER_FEEDBACK) ||
                        safety.faultActive(FLT_STEER_OVERCURRENT) ||
                        safety.state() == VehicleState::FAULT ||
                        safety.state() == VehicleState::ESTOP;
  if (faultNow && !lastFault_) steerDiag.pushEvent(SteerDiagEvent::FAULT, 0);
  lastFault_ = faultNow;

  auto publish = [&](float setpoint, float pwm) {
    telemetry.update([&](VehicleTelemetry& t) {
      t.steering.requestedPct = setpoint;
      t.steering.actualPct = actualPct_;
      t.steering.requestedAngleDeg = (setpoint - 50.0f) / 50.0f * maxAngle;
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
      t.steering.currentValid = currentValid;
      t.steering.calibrated = cal.valid;
      t.steering.velocityPctS = filtVel_;
      t.steering.feedforward = feedforward_;
      t.steering.controlState = (uint8_t)controlState_;
      t.steering.outputEnabled = outputEnabled;
    });
    recordSample(setpoint, pwm, outputEnabled, steeringCurrentA, currentValid);
    emitEdgeEvents();
  };

  // --- calibration wizard owns the actuator --------------------------------
  if (calActive) {
    const float calPwm =
        calibration.steeringCalStep(feedback_.filtered(), steeringCurrentA);
    motor_.drive(calPwm);
    appliedPwm_ = calPwm;
    feedforward_ = 0.0f;
    controlState_ = SteerControlState::CALIBRATION;
    publish(actualPct_, calPwm);
    return;
  }

  // --- smart characterization (same actuator, existing PID when closed-loop)
  if (charActive) {
    if (!outputEnabled) {
      steerChar.abort("outputs disabled");
      motor_.drive(0);
      appliedPwm_ = 0;
      feedforward_ = 0.0f;
      controlState_ = faultNow ? SteerControlState::FAULT
                               : SteerControlState::IDLE;
      publish(actualPct_, 0.0f);
      return;
    }
    steerChar.step(actualPct_, feedback_.filtered(), (float)feedback_.raw(), dt,
                   feedback_.health() == SensorHealth::OK, faultNow,
                   loopMisses_);
    if (steerChar.live().openLoop) {
      const float charPwm = steerChar.live().pwmCmd;
      pid_.reset();
      motor_.drive(charPwm);
      appliedPwm_ = charPwm;
      feedforward_ = 0.0f;
      controlState_ = SteerControlState::CALIBRATION;
      targetPct_ = actualPct_;
      publish(targetPct_, charPwm);
      return;
    }
    // Closed-loop follow of the wizard target (center / settling step).
    targetPct_ = steerChar.live().closedLoopTarget;
  }

  // --- normal closed loop (also used for characterization closed-loop) ------
  float requestPct;
  if (charActive) {
    requestPct = steerChar.live().closedLoopTarget;
  } else if (calibration.manualSteeringActive() && safety.testMotionAllowed()) {
    requestPct = calibration.manualSteeringTarget();
  } else {
    requestPct = commandToTargetPct(constrain(steeringCmd, -1.0f, 1.0f));
  }

  const float maxStep = config.f(STR_RATE_LIMIT) * dt;
  targetPct_ += constrain(requestPct - targetPct_, -maxStep, maxStep);
  const float desiredVel = (targetPct_ - lastTargetPct_) / dt;
  lastTargetPct_ = targetPct_;

  const bool driveAllowed = outputEnabled && cal.valid &&
                            feedback_.health() == SensorHealth::OK &&
                            !safety.faultActive(FLT_STEER_OVERCURRENT) &&
                            (safety.motionAllowed() ||
                             safety.testMotionAllowed() || charActive);

  float leftAdc = 0, rightAdc = 0;
  calibration.softLimits(leftAdc, rightAdc);
  const float leftPct =
      cal.valid ? FirgelliFeedback::toPct(leftAdc, cal.leftAdc, cal.rightAdc)
                : 0.0f;
  const float rightPct =
      cal.valid ? FirgelliFeedback::toPct(rightAdc, cal.leftAdc, cal.rightAdc)
                : 100.0f;
  const bool atLimit = (actualPct_ <= leftPct + 0.2f) ||
                       (actualPct_ >= rightPct - 0.2f);
  if (atLimit && !lastAtLimit_)
    steerDiag.pushEvent(SteerDiagEvent::LIMIT_DETECTED, 0);
  lastAtLimit_ = atLimit;

  const float err = targetPct_ - actualPct_;
  const float absErr = fabsf(err);
  const float deadband = config.f(STR_DEADBAND);

  float out = 0.0f;
  feedforward_ = 0.0f;
  if (driveAllowed) {
    Pid::Gains g = pid_.gains();
    g.kp = scheduledKp(absErr, config.f(STR_FAR_P), config.f(STR_NEAR_P),
                       config.f(STR_HOLD_P), config.f(STR_APPROACH),
                       config.f(STR_SETTLE_TH), config.f(STR_PID_KP));
    pid_.setGains(g);

    const bool inHold = absErr <= config.f(STR_HOLD_TH);
    const bool inDead = absErr < deadband;
    const bool freezeI =
        inDead || inHold || atLimit || !compSt_.moving || pid_.saturated();
    Pid::StepOpts opt;
    opt.allowIntegral = !freezeI;
    opt.useMeasDerivative = true;
    opt.measDerivative = filtVel_;
    out = pid_.update(targetPct_, actualPct_, dt, opt);

    if (config.b(STR_FF_EN) &&
        steerChar.result().status == SteerCharStatus::OK) {
      const SteeringCharacterization& ch = steerChar.result();
      const float* velTab =
          (desiredVel < 0.0f) ? ch.velLeft : ch.velRight;
      feedforward_ =
          feedforwardPwm(desiredVel, ch.pwm, velTab, ch.nPoints,
                         config.f(STR_FF_GAIN));
      out += feedforward_;
    }

    if (inDead && !compSt_.moving) out = 0.0f;

    out = applyPwmHysteresis(out, err, compCfg_, compSt_);
    out = slewPwm(appliedPwm_, out, dt, config.f(STR_PWM_RAMP_UP),
                  config.f(STR_PWM_RAMP_DN));

    if ((actualPct_ <= leftPct && out < 0) ||
        (actualPct_ >= rightPct && out > 0)) {
      out = 0.0f;
      pid_.reset();
      compSt_.moving = false;
      compSt_.dir = 0;
    }

    const float maxPwm = config.f(STR_MAX_PWM);
    const bool atPwmLim = fabsf(out) >= maxPwm - 0.05f;
    if (atPwmLim && !lastPwmLimit_)
      steerDiag.pushEvent(SteerDiagEvent::PWM_LIMIT, (int16_t)lrintf(out));
    lastPwmLimit_ = atPwmLim;
  } else {
    pid_.reset();
    compSt_ = SteerCompState{};
    feedforward_ = 0.0f;
    lastPwmLimit_ = false;
  }
  motor_.drive(out);
  appliedPwm_ = motor_.currentPct();

  controlState_ = classifySteerState(
      absErr, appliedPwm_, atLimit && ((appliedPwm_ < 0 && actualPct_ <= leftPct) ||
                                       (appliedPwm_ > 0 && actualPct_ >= rightPct) ||
                                       (fabsf(appliedPwm_) < 0.5f && atLimit)),
      faultNow, calActive || charActive, driveAllowed, config.f(STR_APPROACH),
      config.f(STR_SETTLE_TH), config.f(STR_HOLD_TH), deadband);

  publish(targetPct_, appliedPwm_);
}

}  // namespace vcm
