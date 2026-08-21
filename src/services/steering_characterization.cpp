#include "services/steering_characterization.h"

#include <Preferences.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "config/config_registry.h"
#include "control/steering_compensation.h"
#include "core/version.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/steering_recorder.h"

namespace vcm {

SteeringCharacterizationService steerChar;

static constexpr uint32_t kCharTimeoutMs = 180000;
static const char* kNs = "vcm_char";

const char* SteeringCharacterizationService::phaseName(SteerCharPhase p) {
  switch (p) {
    case SteerCharPhase::IDLE: return "IDLE";
    case SteerCharPhase::CHECK: return "CHECK";
    case SteerCharPhase::MOVE_CENTER: return "MOVE_CENTER";
    case SteerCharPhase::VERIFY_STILL: return "VERIFY_STILL";
    case SteerCharPhase::SWEEP_LEFT: return "SWEEP_LEFT";
    case SteerCharPhase::RETURN_CENTER_1: return "RETURN_CENTER";
    case SteerCharPhase::SWEEP_RIGHT: return "SWEEP_RIGHT";
    case SteerCharPhase::RETURN_CENTER_2: return "RETURN_CENTER";
    case SteerCharPhase::SETTLING: return "SETTLING";
    case SteerCharPhase::BACKLASH: return "BACKLASH";
    case SteerCharPhase::ANALYZE: return "ANALYZE";
    case SteerCharPhase::DONE: return "DONE";
    case SteerCharPhase::FAILED: return "FAILED";
  }
  return "?";
}

void SteeringCharacterizationService::begin() { loadFromNvs(); }

bool SteeringCharacterizationService::active() const {
  return phase_ != SteerCharPhase::IDLE && phase_ != SteerCharPhase::DONE &&
         phase_ != SteerCharPhase::FAILED;
}

bool SteeringCharacterizationService::start(bool confirmed) {
  if (!confirmed) return false;
  if (active()) return false;
  if (calibration.steeringCalActive()) return false;
  calibration.setOpenLoopSteer(0, false);
  if (!config.b(STR_CHAR_ENABLE)) {
    LOGW("SCHAR", "Smart characterization disabled in config");
    return false;
  }
  if (!safety.testMotionAllowed() &&
      !safety.requestState(VehicleState::CALIBRATION,
                           "smart characterization")) {
    return false;
  }

  current_ = SteeringCharacterization{};
  current_.magic = kSteerCharMagic;
  current_.version = 1;
  current_.status = SteerCharStatus::INVALID;
  strncpy(current_.fwVersion, VCM_FW_VERSION, sizeof(current_.fwVersion) - 1);
  recommended_ = SteerRecommendations{};

  pwmLevel_ = config.f(STR_CHAR_START_PWM);
  dwellPass_ = 0;
  lookingForStart_ = true;
  foundStart_ = false;
  holdSearch_ = false;
  minStartThisDir_ = 0;
  holdThisDir_ = 0;
  lastMisses_ = 0;
  settleLeg_ = 0;
  startMs_ = millis();
  steerDiag.setRecording(true);
  steerDiag.pushEvent(SteerDiagEvent::CHAR_STEP, 0);
  setPhase(SteerCharPhase::CHECK, "Verifying feedback and limits");
  LOGI("SCHAR", "Smart characterization started");
  return true;
}

void SteeringCharacterizationService::abort(const char* reason) {
  if (!active()) return;
  fail(reason ? reason : "aborted");
}

void SteeringCharacterizationService::setPhase(SteerCharPhase p,
                                              const char* msg) {
  phase_ = p;
  phaseStartMs_ = millis();
  live_.phase = p;
  live_.stepName = phaseName(p);
  live_.progressPct = 0;
  if (msg) {
    strncpy(live_.message, msg, sizeof(live_.message) - 1);
    live_.message[sizeof(live_.message) - 1] = 0;
  }
  steerDiag.pushEvent(SteerDiagEvent::CHAR_STEP, (int16_t)p);
}

void SteeringCharacterizationService::fail(const char* reason) {
  current_.status = SteerCharStatus::FAILED;
  current_.timestamp = millis() / 1000;
  strncpy(current_.failReason, reason ? reason : "failed",
          sizeof(current_.failReason) - 1);
  analyze();  // keep partial curve / recommendations if any points exist
  persistCurrent();
  saveHistory();
  live_.openLoop = true;
  live_.pwmCmd = 0;
  setPhase(SteerCharPhase::FAILED, current_.failReason);
  steerDiag.pushEvent(SteerDiagEvent::CHAR_FAIL, 0);
  safety.requestState(calibration.commissioned() ? VehicleState::READY
                                                 : VehicleState::NOT_CALIBRATED,
                      "characterization failed");
  LOGW("SCHAR", "FAILED: %s", current_.failReason);
}

void SteeringCharacterizationService::finishOk() {
  current_.status = SteerCharStatus::OK;
  current_.timestamp = millis() / 1000;
  current_.failReason[0] = 0;
  persistCurrent();
  saveHistory();
  live_.openLoop = false;
  live_.pwmCmd = 0;
  setPhase(SteerCharPhase::DONE, "Characterization complete");
  live_.progressPct = 100;
  steerDiag.pushEvent(SteerDiagEvent::CHAR_DONE, 0);
  safety.requestState(calibration.commissioned() ? VehicleState::READY
                                                 : VehicleState::NOT_CALIBRATED,
                      "characterization complete");
  LOGI("SCHAR",
       "OK start L/R %.1f/%.1f hold %.1f/%.1f vmax %.2f/%.2f backlash %.2f/%.2f",
       current_.minStartLeft, current_.minStartRight, current_.holdLeft,
       current_.holdRight, current_.maxVelLeft, current_.maxVelRight,
       current_.backlashLeft, current_.backlashRight);
}

float SteeringCharacterizationService::driveToCenter(float actualPct) {
  (void)actualPct;
  live_.openLoop = false;
  live_.closedLoopTarget = centerTarget_;
  live_.pwmCmd = 0;
  return 0.0f;
}

void SteeringCharacterizationService::recordSweepPoint(int dirSign, float pwm,
                                                       float vel) {
  if (current_.nPoints >= kSteerCharMaxPoints) return;
  // Share a common PWM axis: add a new PWM slot or fill the existing one.
  int idx = -1;
  for (uint8_t i = 0; i < current_.nPoints; ++i) {
    if (fabsf(current_.pwm[i] - pwm) < 0.05f) {
      idx = i;
      break;
    }
  }
  if (idx < 0) {
    idx = current_.nPoints++;
    current_.pwm[idx] = pwm;
  }
  if (dirSign < 0) {
    current_.velLeft[idx] = vel;
    current_.maxVelLeft = std::max(current_.maxVelLeft, fabsf(vel));
    current_.maxTestLeft = std::max(current_.maxTestLeft, pwm);
  } else {
    current_.velRight[idx] = vel;
    current_.maxVelRight = std::max(current_.maxVelRight, fabsf(vel));
    current_.maxTestRight = std::max(current_.maxTestRight, pwm);
  }
}

float SteeringCharacterizationService::sweepStep(float actualPct, float dt,
                                                int dirSign) {
  (void)dt;
  const float startP = config.f(STR_CHAR_START_PWM);
  const float step = config.f(STR_CHAR_PWM_STEP);
  const float maxP = config.f(STR_CHAR_MAX_PWM);
  const uint32_t dwellMs = (uint32_t)config.i(STR_CHAR_DWELL_MS);
  const uint32_t settleMs = (uint32_t)config.i(STR_CHAR_SETTLE_MS);
  const float motion = config.f(STR_CHAR_MOTION_PCT);
  const float margin = 8.0f;
  const uint32_t now = millis();
  const uint32_t elapsed = now - phaseStartMs_;

  live_.openLoop = true;
  live_.direction = dirSign;
  live_.pwmLevel = pwmLevel_;

  const bool nearLimit =
      (dirSign < 0 && actualPct <= margin) ||
      (dirSign > 0 && actualPct >= (100.0f - margin));

  auto storeHold = [&]() {
    if (dirSign < 0) current_.holdLeft = holdThisDir_;
    else current_.holdRight = holdThisDir_;
  };

  auto resumeVelocitySweep = [&]() -> bool {
    holdSearch_ = false;
    storeHold();
    pwmLevel_ = minStartThisDir_ + step;
    dwellPass_ = 0;
    lookingForStart_ = false;
    phaseStartMs_ = now;
    LOGI("SCHAR", "%s hold PWM %.1f%% (start %.1f%%)",
         dirSign < 0 ? "LEFT" : "RIGHT", holdThisDir_, minStartThisDir_);
    if (pwmLevel_ > maxP + 0.01f || nearLimit) return true;
    return false;
  };

  auto bumpPwm = [&]() {
    pwmLevel_ += step;
    dwellPass_ = 0;
    lookingForStart_ = !foundStart_;
    holdSearch_ = false;
    phaseStartMs_ = now;
    if (pwmLevel_ > maxP + 0.01f || nearLimit) return true;  // done
    return false;
  };

  // dwellPass_: 0=settle, 1=apply, 2=optional repeat settle, 3=repeat apply
  if (dwellPass_ == 0) {
    live_.pwmCmd = 0;
    if (elapsed >= settleMs) {
      dwellPos0_ = actualPct;
      dwellPass_ = 1;
      phaseStartMs_ = now;
      steerDiag.pushEvent(SteerDiagEvent::PWM_TEST,
                          (int16_t)lrintf(pwmLevel_ * 10.0f));
      snprintf(live_.message, sizeof(live_.message),
               holdSearch_ ? "Hold PWM %.1f%% %s" : "PWM %.1f%% %s",
               pwmLevel_, dirSign < 0 ? "LEFT" : "RIGHT");
    }
    return 0.0f;
  }

  if (dwellPass_ == 1 || dwellPass_ == 3) {
    live_.pwmCmd = (float)dirSign * pwmLevel_;
    if (elapsed >= dwellMs) {
      const float pos1 = actualPct;
      const float move = pos1 - dwellPos0_;
      const float vel = (dwellMs > 0) ? move / (dwellMs * 0.001f) : 0.0f;
      sweepVel_ = vel;

      if (holdSearch_ && dwellPass_ == 1) {
        recordSweepPoint(dirSign, pwmLevel_, vel);
        if (fabsf(move) >= motion) {
          holdThisDir_ = pwmLevel_;
          pwmLevel_ -= step;
          dwellPass_ = 0;
          phaseStartMs_ = now;
          if (pwmLevel_ < startP - 0.01f) {
            if (resumeVelocitySweep()) return 1e9f;
          }
          return 0.0f;
        }
        if (holdThisDir_ <= 0.0f) holdThisDir_ = minStartThisDir_;
        if (resumeVelocitySweep()) return 1e9f;
        return 0.0f;
      }

      if (dwellPass_ == 1) {
        dwellPos1_ = move;
        if (lookingForStart_ && fabsf(move) >= motion) {
          dwellPass_ = 2;  // repeat for repeatability
          phaseStartMs_ = now;
          return 0.0f;
        }
        if (!lookingForStart_) recordSweepPoint(dirSign, pwmLevel_, vel);
        if (lookingForStart_ && fabsf(move) < motion) {
          if (bumpPwm()) return 1e9f;  // signal sweep complete
          return 0.0f;
        }
        if (!lookingForStart_) {
          if (bumpPwm()) return 1e9f;
          return 0.0f;
        }
      } else {  // repeat apply — require the same direction twice
        if (movementRepeatable(dwellPos1_, move, motion)) {
          foundStart_ = true;
          lookingForStart_ = false;
          minStartThisDir_ = pwmLevel_;
          holdThisDir_ = pwmLevel_;
          if (dirSign < 0) current_.minStartLeft = pwmLevel_;
          else current_.minStartRight = pwmLevel_;
          recordSweepPoint(dirSign, pwmLevel_, vel);
          LOGI("SCHAR", "%s min start PWM %.1f%% (move %.2f / %.2f)",
               dirSign < 0 ? "LEFT" : "RIGHT", pwmLevel_, dwellPos1_, move);
          // After start is proven, walk PWM downward to find hold PWM.
          holdSearch_ = true;
          pwmLevel_ = minStartThisDir_ - step;
          dwellPass_ = 0;
          phaseStartMs_ = now;
          if (pwmLevel_ < startP - 0.01f) {
            holdThisDir_ = minStartThisDir_ * 0.75f;
            if (resumeVelocitySweep()) return 1e9f;
          }
          storeHold();
          return 0.0f;
        }
        // Noise: not repeatable, keep sweeping
        if (bumpPwm()) return 1e9f;
        return 0.0f;
      }
    }
    return live_.pwmCmd;
  }

  if (dwellPass_ == 2) {
    live_.pwmCmd = 0;
    if (elapsed >= settleMs) {
      dwellPos0_ = actualPct;
      dwellPass_ = 3;
      phaseStartMs_ = now;
    }
    return 0.0f;
  }
  return 0.0f;
}

float SteeringCharacterizationService::step(float actualPct, float filteredAdc,
                                            float rawAdc, float dt,
                                            bool feedbackOk, bool fault,
                                            uint32_t loopMisses) {
  (void)filteredAdc;
  (void)rawAdc;
  (void)dt;
  live_.phase = phase_;
  live_.stepName = phaseName(phase_);

  if (!active()) {
    live_.openLoop = false;
    live_.pwmCmd = 0;
    return 0.0f;
  }

  if (fault) {
    fail("safety fault");
    return 0.0f;
  }
  if (!feedbackOk) {
    fail("feedback invalid");
    return 0.0f;
  }
  if (millis() - startMs_ > kCharTimeoutMs) {
    fail("timeout");
    return 0.0f;
  }
  if (lastMisses_ != 0 && loopMisses > lastMisses_ + 20) {
    fail("control loop missed deadlines");
    return 0.0f;
  }
  lastMisses_ = loopMisses;

  const uint32_t now = millis();

  switch (phase_) {
    case SteerCharPhase::CHECK: {
      live_.openLoop = true;
      live_.pwmCmd = 0;
      if (!calibration.steeringCal().valid) {
        fail("steering limits missing — run position calibration first");
        return 0.0f;
      }
      setPhase(SteerCharPhase::MOVE_CENTER, "Moving to center");
      live_.progressPct = 5;
      return 0.0f;
    }
    case SteerCharPhase::MOVE_CENTER:
    case SteerCharPhase::RETURN_CENTER_1:
    case SteerCharPhase::RETURN_CENTER_2: {
      driveToCenter(actualPct);
      live_.progressPct =
          (phase_ == SteerCharPhase::MOVE_CENTER) ? 10
          : (phase_ == SteerCharPhase::RETURN_CENTER_1) ? 45
                                                        : 70;
      if (fabsf(actualPct - centerTarget_) < 2.5f &&
          now - phaseStartMs_ > 400) {
        if (phase_ == SteerCharPhase::MOVE_CENTER) {
          stillPos0_ = actualPct;
          setPhase(SteerCharPhase::VERIFY_STILL, "Verifying stationary");
        } else if (phase_ == SteerCharPhase::RETURN_CENTER_1) {
          pwmLevel_ = config.f(STR_CHAR_START_PWM);
          dwellPass_ = 0;
          lookingForStart_ = true;
          foundStart_ = false;
          holdSearch_ = false;
          setPhase(SteerCharPhase::SWEEP_RIGHT, "Characterizing RIGHT");
        } else {
          setPhase(SteerCharPhase::SETTLING, "Settling test");
          settleLeg_ = 0;
          settleTarget_ = 70.0f;
          settlePeak_ = actualPct;
          settleReversals_ = 0;
          settleLastDir_ = 0;
          settleStartUs_ = micros();
        }
      } else if (now - phaseStartMs_ > 30000) {
        fail("failed to reach center");
      }
      return 0.0f;
    }
    case SteerCharPhase::VERIFY_STILL: {
      live_.openLoop = true;
      live_.pwmCmd = 0;
      live_.progressPct = 12;
      const float motion = config.f(STR_CHAR_MOTION_PCT);
      const uint32_t stillMs = now - phaseStartMs_;
      if (stillMs < 300) return 0.0f;
      if (fabsf(actualPct - stillPos0_) > motion) {
        if (stillMs > 1500) {
          fail("actuator not stationary");
          return 0.0f;
        }
        return 0.0f;
      }
      pwmLevel_ = config.f(STR_CHAR_START_PWM);
      dwellPass_ = 0;
      lookingForStart_ = true;
      foundStart_ = false;
      holdSearch_ = false;
      setPhase(SteerCharPhase::SWEEP_LEFT, "Characterizing LEFT");
      return 0.0f;
    }
    case SteerCharPhase::SWEEP_LEFT:
    case SteerCharPhase::SWEEP_RIGHT: {
      const int dir = (phase_ == SteerCharPhase::SWEEP_LEFT) ? -1 : 1;
      live_.progressPct =
          (phase_ == SteerCharPhase::SWEEP_LEFT) ? 20 : 55;
      const float sig = sweepStep(actualPct, dt, dir);
      if (sig > 1e8f) {
        // sweep complete
        if (phase_ == SteerCharPhase::SWEEP_LEFT)
          setPhase(SteerCharPhase::RETURN_CENTER_1, "Returning to center");
        else
          setPhase(SteerCharPhase::RETURN_CENTER_2, "Returning to center");
        return 0.0f;
      }
      return live_.pwmCmd;
    }
    case SteerCharPhase::SETTLING: {
      live_.openLoop = false;
      live_.closedLoopTarget = settleTarget_;
      live_.progressPct = 80;
      const float err = settleTarget_ - actualPct;
      const int8_t d = (err > 0.4f) ? 1 : (err < -0.4f) ? -1 : 0;
      if (d && settleLastDir_ && d != settleLastDir_) settleReversals_++;
      if (d) settleLastDir_ = d;
      if (settleLeg_ == 0) {
        settlePeak_ = std::max(settlePeak_, actualPct);
        if (fabsf(err) < 1.5f && now - phaseStartMs_ > 200) {
          current_.overshootPct = std::max(0.0f, settlePeak_ - settleTarget_);
          current_.settlingTimeS =
              (micros() - settleStartUs_) * 1e-6f;
          current_.finalErrorPct = err;
          current_.dirReversals = (uint8_t)std::min(settleReversals_, 255);
          settleLeg_ = 1;
          settleTarget_ = 50.0f;
          phaseStartMs_ = now;
        } else if (now - phaseStartMs_ > 6000) {
          current_.overshootPct = std::max(0.0f, settlePeak_ - settleTarget_);
          current_.settlingTimeS = 6.0f;
          current_.finalErrorPct = err;
          setPhase(SteerCharPhase::BACKLASH, "Backlash test");
        }
      } else if (now - phaseStartMs_ > 1500) {
        setPhase(SteerCharPhase::BACKLASH, "Backlash test");
        backlashStartPos_ = actualPct;
        backlashMoved_ = false;
        backlashMoveUs_ = 0;
      }
      return 0.0f;
    }
    case SteerCharPhase::BACKLASH: {
      // Open-loop: drive left at start PWM until motion, then reverse.
      const float p = std::max(config.f(STR_CHAR_START_PWM),
                               std::max(current_.minStartLeft,
                                        current_.minStartRight));
      const float motion = config.f(STR_CHAR_MOTION_PCT);
      live_.openLoop = true;
      live_.progressPct = 90;
      const uint32_t el = now - phaseStartMs_;
      if (el < 400) {
        live_.pwmCmd = 0;
      } else if (el < 1400) {
        live_.pwmCmd = -p;
        if (!backlashMoved_ &&
            fabsf(actualPct - backlashStartPos_) >= motion) {
          backlashMoved_ = true;
          backlashMoveUs_ = micros();
          current_.backlashLeft = fabsf(actualPct - backlashStartPos_);
        }
      } else if (el < 1600) {
        live_.pwmCmd = 0;
        if (el >= 1400 && backlashStartPos_ == actualPct) {
          /* keep start for reverse */
        }
        if (el < 1410) backlashStartPos_ = actualPct;
      } else if (el < 2600) {
        live_.pwmCmd = p;
        if (backlashMoved_ &&
            fabsf(actualPct - backlashStartPos_) >= motion) {
          current_.backlashRight = fabsf(actualPct - backlashStartPos_);
          backlashMoved_ = false;
        }
      } else {
        live_.pwmCmd = 0;
        setPhase(SteerCharPhase::ANALYZE, "Analyzing");
      }
      return live_.pwmCmd;
    }
    case SteerCharPhase::ANALYZE:
      live_.openLoop = true;
      live_.pwmCmd = 0;
      analyze();
      finishOk();
      return 0.0f;
    default:
      live_.openLoop = true;
      live_.pwmCmd = 0;
      return 0.0f;
  }
}

void SteeringCharacterizationService::analyze() {
  auto preferred = [](const float* pwm, const float* vel, int n) {
    float bestPwm = 0, bestScore = 1e9f;
    float vmax = 0;
    for (int i = 0; i < n; ++i) vmax = std::max(vmax, fabsf(vel[i]));
    if (vmax < 1e-3f) return 0.0f;
    for (int i = 0; i < n; ++i) {
      if (fabsf(vel[i]) < 0.3f) continue;
      // Prefer reliable moderate speed, not the fastest point.
      const float v = fabsf(vel[i]);
      const float target = std::min(6.0f, vmax * 0.45f);
      const float score = fabsf(v - target) + pwm[i] * 0.02f;
      if (score < bestScore) {
        bestScore = score;
        bestPwm = pwm[i];
      }
    }
    return bestPwm;
  };
  current_.preferredLeft =
      preferred(current_.pwm, current_.velLeft, current_.nPoints);
  current_.preferredRight =
      preferred(current_.pwm, current_.velRight, current_.nPoints);
  if (current_.preferredLeft < 0.1f) current_.preferredLeft = current_.minStartLeft;
  if (current_.preferredRight < 0.1f)
    current_.preferredRight = current_.minStartRight;
  current_.oscCount = current_.dirReversals;

  recommended_ = recommendSteering(current_, config.f(STR_PID_KP),
                                   config.f(STR_PID_KD), config.f(STR_DEADBAND),
                                   config.f(STR_MAX_PWM));
}

void SteeringCharacterizationService::saveBlob(
    const char* key, const SteeringCharacterization& c) {
  Preferences p;
  if (!p.begin(kNs, false)) return;
  p.putBytes(key, &c, sizeof(c));
  p.end();
}

bool SteeringCharacterizationService::loadBlob(const char* key,
                                               SteeringCharacterization& c) {
  Preferences p;
  if (!p.begin(kNs, true)) return false;
  const size_t n = p.getBytesLength(key);
  bool ok = false;
  if (n >= sizeof(uint32_t)) {
    SteeringCharacterization tmp{};
    p.getBytes(key, &tmp, sizeof(tmp));
    if (tmp.magic == kSteerCharMagic) {
      c = tmp;
      ok = true;
    }
  }
  p.end();
  return ok;
}

void SteeringCharacterizationService::persistCurrent() {
  saveBlob("current", current_);
}

void SteeringCharacterizationService::saveHistory() {
  if (histCount_ < kSteerCharHistory) {
    history_[histCount_++] = current_;
  } else {
    for (size_t i = 1; i < kSteerCharHistory; ++i) history_[i - 1] = history_[i];
    history_[kSteerCharHistory - 1] = current_;
  }
  Preferences p;
  if (!p.begin(kNs, false)) return;
  p.putUChar("nhist", histCount_);
  for (uint8_t i = 0; i < histCount_; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "h%u", (unsigned)i);
    p.putBytes(key, &history_[i], sizeof(history_[i]));
  }
  p.end();
}

void SteeringCharacterizationService::loadFromNvs() {
  loadBlob("current", current_);
  loadBlob("baseline", baseline_);
  Preferences p;
  if (!p.begin(kNs, true)) return;
  histCount_ = p.getUChar("nhist", 0);
  if (histCount_ > kSteerCharHistory) histCount_ = kSteerCharHistory;
  for (uint8_t i = 0; i < histCount_; ++i) {
    char key[16];
    snprintf(key, sizeof(key), "h%u", (unsigned)i);
    p.getBytes(key, &history_[i], sizeof(history_[i]));
  }
  p.end();
  if (current_.status == SteerCharStatus::OK) {
    recommended_ = recommendSteering(current_, config.f(STR_PID_KP),
                                     config.f(STR_PID_KD),
                                     config.f(STR_DEADBAND),
                                     config.f(STR_MAX_PWM));
  }
  LOGI("SCHAR", "Loaded characterization %s, baseline %s, history %u",
       current_.status == SteerCharStatus::OK ? "OK"
       : current_.status == SteerCharStatus::FAILED ? "FAILED"
                                                    : "none",
       baseline_.status == SteerCharStatus::OK ? "OK" : "none",
       (unsigned)histCount_);
}

bool SteeringCharacterizationService::saveAsBaseline() {
  if (current_.status != SteerCharStatus::OK) return false;
  baseline_ = current_;
  saveBlob("baseline", baseline_);
  LOGI("SCHAR", "Saved characterization as baseline");
  return true;
}

bool SteeringCharacterizationService::loadHistorySlot(
    uint8_t index, SteeringCharacterization& out) const {
  if (index >= histCount_) return false;
  out = history_[index];
  return true;
}

}  // namespace vcm
