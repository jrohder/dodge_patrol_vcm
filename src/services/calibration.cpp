#include "services/calibration.h"

#include <Preferences.h>

#include "config/config_registry.h"
#include "config/vehicle_defaults.h"
#include "core/version.h"
#include "services/logger.h"
#include "services/safety.h"

namespace vcm {

CalibrationService calibration;

static const char* kNamespace = "vcm_cal";
static constexpr uint32_t kCalTimeoutMs = 60000;  ///< abort runaway wizard

void CalibrationService::applyFactoryDefaults() {
  steerData_.valid = factory::kSteeringCalValid;
  steerData_.leftAdc = factory::kSteerLeftAdc;
  steerData_.rightAdc = factory::kSteerRightAdc;
  steerData_.centerAdc = factory::kSteerCenterAdc;
  steerData_.timestamp = 0;
  strncpy(steerData_.fwVersion, "factory", sizeof(steerData_.fwVersion) - 1);
  commissioned_ = factory::kCommissioned;
}

void CalibrationService::begin() {
  Preferences p;
  bool loaded = false;
  if (p.begin(kNamespace, true)) {
    steerData_.valid = p.getBool("str_valid", false);
    steerData_.leftAdc = p.getFloat("str_left", 0);
    steerData_.rightAdc = p.getFloat("str_right", 0);
    steerData_.centerAdc = p.getFloat("str_center", 0);
    steerData_.timestamp = p.getULong("str_time", 0);
    String fw = p.getString("str_fw", "");
    strncpy(steerData_.fwVersion, fw.c_str(), sizeof(steerData_.fwVersion) - 1);
    commissioned_ = p.getBool("commissioned", false);
    p.end();
    loaded = steerData_.valid;
  }
  if (!loaded) {
    applyFactoryDefaults();
    saveSteering();
    LOGI("CAL", "Applied factory steering limits L=%.0f C=%.0f R=%.0f",
         steerData_.leftAdc, steerData_.centerAdc, steerData_.rightAdc);
  }
  LOGI("CAL", "Steering calibration: %s%s",
       steerData_.valid ? "VALID" : "NOT CALIBRATED",
       commissioned_ ? ", vehicle commissioned" : ", NOT COMMISSIONED");
}

bool CalibrationService::saveSteering() {
  Preferences p;
  if (!p.begin(kNamespace, false)) {
    LOGE("CAL", "NVS open failed for steering cal");
    return false;
  }
  bool ok = p.putBool("str_valid", steerData_.valid);
  ok = p.putFloat("str_left", steerData_.leftAdc) && ok;
  ok = p.putFloat("str_right", steerData_.rightAdc) && ok;
  ok = p.putFloat("str_center", steerData_.centerAdc) && ok;
  ok = p.putULong("str_time", steerData_.timestamp) && ok;
  ok = p.putString("str_fw", steerData_.fwVersion) && ok;
  ok = p.putBool("commissioned", commissioned_) && ok;
  p.end();
  if (!ok) LOGE("CAL", "NVS write failed for steering cal");
  return ok;
}

void CalibrationService::dumpToLog() const {
  LOGI("CAL", "valid=%d commissioned=%d L=%.1f C=%.1f R=%.1f fw=%s",
       steerData_.valid ? 1 : 0, commissioned_ ? 1 : 0, steerData_.leftAdc,
       steerData_.centerAdc, steerData_.rightAdc, steerData_.fwVersion);
}

// ---------------------------------------------------------------- steering

bool CalibrationService::startSteeringCal(const SteeringCalParams& params) {
  if (steeringCalActive()) return false;
  if (!safety.requestState(VehicleState::CALIBRATION, "steering calibration"))
    return false;
  params_ = params;
  params_.pwmPct = constrain(params_.pwmPct, 5.0f, 40.0f);  // always low power
  steerState_ = SteeringCalState::MOVE_LEFT;
  calStartMs_ = millis();
  lastMotionMs_ = millis();
  lastAdc_ = 0;
  confirmStartMs_ = 0;
  openLoopActive_ = false;
  manualSteerActive_ = false;
  LOGI("CAL", "Steering auto-calibration started (pwm=%.0f%%, i>%.1fA, %lums)",
       params_.pwmPct, params_.currentThresholdA,
       (unsigned long)params_.noMotionMs);
  return true;
}

void CalibrationService::abortSteeringCal() {
  if (!steeringCalActive()) return;
  steerState_ = SteeringCalState::ABORTED;
  safety.requestState(commissioned_ ? VehicleState::READY
                                    : VehicleState::NOT_CALIBRATED,
                      "steering calibration aborted");
  LOGW("CAL", "Steering calibration ABORTED");
}

float CalibrationService::steeringCalStep(float feedbackAdc, float currentA,
                                           bool currentValid) {
  const uint32_t now = millis();

  // Global timeout guard: never let the wizard run away
  if (steeringCalActive() && now - calStartMs_ > kCalTimeoutMs) {
    LOGE("CAL", "Steering calibration timeout");
    abortSteeringCal();
    return 0.0f;
  }

  // Track motion: position considered changing if ADC moved > epsilon
  if (fabsf(feedbackAdc - lastAdc_) > params_.motionEpsilon) {
    lastAdc_ = feedbackAdc;
    lastMotionMs_ = now;
  }
  const uint32_t stallMs =
      currentValid ? params_.noMotionMs : max(params_.noMotionMs, (uint32_t)600);
  const bool noMotion = (now - lastMotionMs_) > stallMs;
  const bool currentHigh = currentValid && currentA > params_.currentThresholdA;
  // With INA: stall = elevated current AND no motion (never current alone).
  // Without INA: stall = no motion only (otherwise the wizard never finishes).
  const bool stall = currentValid ? (currentHigh && noMotion) : noMotion;
  const bool invert = config.b(STR_INVERT);
  const float drivePwm = invert ? -params_.pwmPct : params_.pwmPct;

  switch (steerState_) {
    case SteeringCalState::MOVE_LEFT:
    case SteeringCalState::CONFIRM_LEFT: {
      if (stall) {
        if (steerState_ == SteeringCalState::MOVE_LEFT) {
          steerState_ = SteeringCalState::CONFIRM_LEFT;
          confirmStartMs_ = now;
        } else if (now - confirmStartMs_ > stallMs) {
          calLeft_ = feedbackAdc;
          steerState_ = SteeringCalState::SETTLE_LEFT;
          settleStartMs_ = now;
          LOGI("CAL", "Left hard stop at ADC %.0f (I=%.2fA ina=%d)", calLeft_,
               currentA, currentValid ? 1 : 0);
          return 0.0f;  // remove power immediately
        }
      } else if (steerState_ == SteeringCalState::CONFIRM_LEFT) {
        steerState_ = SteeringCalState::MOVE_LEFT;  // condition released
      }
      return -drivePwm;  // drive toward left
    }
    case SteeringCalState::SETTLE_LEFT:
      if (now - settleStartMs_ > 500) {
        steerState_ = SteeringCalState::MOVE_RIGHT;
        lastMotionMs_ = now;
      }
      return 0.0f;
    case SteeringCalState::MOVE_RIGHT:
    case SteeringCalState::CONFIRM_RIGHT: {
      if (stall) {
        if (steerState_ == SteeringCalState::MOVE_RIGHT) {
          steerState_ = SteeringCalState::CONFIRM_RIGHT;
          confirmStartMs_ = now;
        } else if (now - confirmStartMs_ > stallMs) {
          calRight_ = feedbackAdc;
          steerState_ = SteeringCalState::SETTLE_RIGHT;
          settleStartMs_ = now;
          LOGI("CAL", "Right hard stop at ADC %.0f (I=%.2fA ina=%d)", calRight_,
               currentA, currentValid ? 1 : 0);
          return 0.0f;
        }
      } else if (steerState_ == SteeringCalState::CONFIRM_RIGHT) {
        steerState_ = SteeringCalState::MOVE_RIGHT;
      }
      return drivePwm;
    }
    case SteeringCalState::SETTLE_RIGHT:
      if (now - settleStartMs_ > 500) {
        setSteeringLimits(calLeft_, calRight_, (calLeft_ + calRight_) * 0.5f);
        steerState_ = SteeringCalState::DONE;
        safety.requestState(commissioned_ ? VehicleState::READY
                                          : VehicleState::NOT_CALIBRATED,
                            "steering calibration complete");
      }
      return 0.0f;
    default:
      return 0.0f;
  }
}

void CalibrationService::setSteeringLimits(float leftAdc, float rightAdc,
                                           float centerAdc) {
  if (fabsf(rightAdc - leftAdc) < 50.0f) {
    LOGE("CAL", "Steering calibration rejected: travel too small (%.0f)",
         fabsf(rightAdc - leftAdc));
    return;
  }
  steerData_.leftAdc = leftAdc;
  steerData_.rightAdc = rightAdc;
  steerData_.centerAdc = centerAdc;
  steerData_.valid = true;
  steerData_.timestamp = millis() / 1000;
  strncpy(steerData_.fwVersion, VCM_FW_VERSION,
          sizeof(steerData_.fwVersion) - 1);
  saveSteering();
  safety.clearFault(FLT_STEER_NOT_CAL);
  LOGI("CAL", "Steering limits saved: L=%.0f C=%.0f R=%.0f", leftAdc,
       centerAdc, rightAdc);
}

void CalibrationService::softLimits(float& leftAdc, float& rightAdc) const {
  const float span = steerData_.rightAdc - steerData_.leftAdc;
  const float margin = span * config.f(STR_SOFT_MARGIN) / 100.0f;
  leftAdc = steerData_.leftAdc + margin;
  rightAdc = steerData_.rightAdc - margin;
}

float CalibrationService::centerAdc() const {
  const float span = steerData_.rightAdc - steerData_.leftAdc;
  return steerData_.centerAdc + span * config.f(STR_CENTER_TRIM) / 100.0f;
}

// ---------------------------------------------------------------- motors

bool CalibrationService::startMotorTest(bool left, bool right, float pwmPct,
                                        uint32_t durationMs) {
  if (!safety.testMotionAllowed() &&
      !safety.requestState(VehicleState::DIAGNOSTIC, "motor test")) {
    return false;
  }
  motorTest_.active = true;
  motorTest_.left = left;
  motorTest_.right = right;
  motorTest_.pwmPct = constrain(pwmPct, 0.0f, 40.0f);  // commissioning limit
  motorTest_.endMs = millis() + min(durationMs, (uint32_t)15000);
  LOGI("CAL", "Motor test: %s%s at %.0f%% for %lums", left ? "LEFT " : "",
       right ? "RIGHT" : "", motorTest_.pwmPct,
       (unsigned long)min(durationMs, (uint32_t)15000));
  return true;
}

void CalibrationService::stopMotorTest() {
  if (!motorTest_.active && !manualSteerActive_) return;
  motorTest_.active = false;
  if (!manualSteerActive_ && safety.state() == VehicleState::DIAGNOSTIC) {
    safety.requestState(commissioned_ ? VehicleState::READY
                                      : VehicleState::NOT_CALIBRATED,
                        "motor test stopped");
  }
  LOGI("CAL", "Motor test stopped");
}

void CalibrationService::tickMotorTest() {
  if (motorTest_.active && millis() > motorTest_.endMs) stopMotorTest();
}

// ---------------------------------------------------------------- misc

void CalibrationService::setManualSteeringTarget(float pct, bool enabled) {
  if (enabled && !safety.testMotionAllowed()) {
    if (!safety.requestState(VehicleState::DIAGNOSTIC, "manual steering test"))
      return;
  }
  openLoopActive_ = false;
  manualSteerTarget_ = constrain(pct, 0.0f, 100.0f);
  manualSteerActive_ = enabled;
  if (!enabled && !motorTest_.active && !openLoopActive_ &&
      safety.state() == VehicleState::DIAGNOSTIC) {
    safety.requestState(commissioned_ ? VehicleState::READY
                                      : VehicleState::NOT_CALIBRATED,
                        "manual steering test ended");
  }
}

void CalibrationService::setOpenLoopSteer(float pwmPct, bool enabled) {
  if (enabled) {
    if (steeringCalActive()) return;
    if (!safety.testMotionAllowed() &&
        !safety.requestState(VehicleState::DIAGNOSTIC, "usb open-loop steer")) {
      LOGW("CAL", "open-loop steer rejected");
      return;
    }
  }
  manualSteerActive_ = false;
  openLoopPwm_ = constrain(pwmPct, -40.0f, 40.0f);
  openLoopActive_ = enabled;
  if (!enabled && !motorTest_.active &&
      safety.state() == VehicleState::DIAGNOSTIC) {
    safety.requestState(commissioned_ ? VehicleState::READY
                                      : VehicleState::NOT_CALIBRATED,
                        "usb open-loop steer ended");
  }
  LOGI("CAL", "open-loop steer %s pwm=%.1f", enabled ? "ON" : "off",
       enabled ? openLoopPwm_ : 0.0f);
}

void CalibrationService::captureCurrentOffset(float measuredOffsetA) {
  config.applyByKey("current.offset", -measuredOffsetA);
  LOGI("CAL", "Current zero offset captured: %.3f A (apply+save to persist)",
       -measuredOffsetA);
}

bool CalibrationService::commissioned() const {
  return commissioned_ && steerData_.valid;
}

void CalibrationService::setCommissioned(bool done) {
  commissioned_ = done;
  saveSteering();
  LOGI("CAL", "Vehicle %s", done ? "COMMISSIONED" : "de-commissioned");
}

}  // namespace vcm
