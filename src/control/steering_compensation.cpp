#include "control/steering_compensation.h"

#include <algorithm>
#include <cmath>

namespace vcm {

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

bool movementRepeatable(float delta1, float delta2, float threshold) {
  if (!(threshold > 0.0f)) threshold = 0.0f;
  return std::fabs(delta1) >= threshold && std::fabs(delta2) >= threshold &&
         (delta1 * delta2 > 0.0f);
}

float scheduledKp(float absError, float farP, float nearP, float holdP,
                  float approachThresh, float settleThresh, float fallbackKp) {
  auto nz = [fallbackKp](float v) { return v > 0.0f ? v : fallbackKp; };
  if (absError >= approachThresh) return nz(farP);
  if (absError >= settleThresh) return nz(nearP);
  return nz(holdP);
}

SteerControlState classifySteerState(float absError, float pwm, bool atLimit,
                                     bool fault, bool calibration,
                                     bool outputEnabled, float approachThresh,
                                     float settleThresh, float holdThresh,
                                     float deadband) {
  if (fault) return SteerControlState::FAULT;
  if (calibration) return SteerControlState::CALIBRATION;
  if (!outputEnabled) return SteerControlState::IDLE;
  if (atLimit) return SteerControlState::LIMIT;
  if (absError <= holdThresh) return SteerControlState::HOLDING;
  if (absError <= deadband && std::fabs(pwm) < 0.5f)
    return SteerControlState::DEADBAND;
  if (absError <= settleThresh) return SteerControlState::SETTLING;
  if (absError <= approachThresh) return SteerControlState::APPROACHING;
  if (pwm > 0.5f) return SteerControlState::MOVING_RIGHT;
  if (pwm < -0.5f) return SteerControlState::MOVING_LEFT;
  return SteerControlState::IDLE;
}

float applyPwmHysteresis(float pidOut, float error, const SteerCompConfig& cfg,
                         SteerCompState& st) {
  const float mag = std::fabs(pidOut);
  const int8_t want = (pidOut > 0.05f) ? 1 : (pidOut < -0.05f) ? -1 : 0;
  const float start =
      (want < 0 || (want == 0 && st.dir < 0)) ? cfg.startLeft : cfg.startRight;
  const float hold =
      (st.dir < 0 || want < 0) ? cfg.holdLeft : cfg.holdRight;
  const float startUse = std::max(0.0f, start);
  const float holdUse = std::max(0.0f, hold);
  const float absErr = std::fabs(error);

  if (!st.moving) {
    if (want == 0 || mag + 1e-3f < startUse) return 0.0f;
    if (absErr < cfg.reengageError && cfg.reengageError > 0.0f) return 0.0f;
    st.moving = true;
    st.dir = want;
    return pidOut;
  }

  // Direction reversal: require start PWM in the new direction.
  if (want != 0 && want != st.dir) {
    const float revStart = (want < 0) ? cfg.startLeft : cfg.startRight;
    if (mag + 1e-3f < std::max(0.0f, revStart) + cfg.hysteresis) {
      st.moving = false;
      st.dir = 0;
      return 0.0f;
    }
    st.dir = want;
    return pidOut;
  }

  if (want == 0 || mag < holdUse) {
    if (absErr <= cfg.holdError) {
      st.moving = false;
      st.dir = 0;
      return 0.0f;
    }
    if (holdUse > 0.0f && absErr > cfg.holdError) {
      const int8_t d = st.dir != 0 ? st.dir : (error > 0 ? 1 : -1);
      return (float)d * holdUse;
    }
    st.moving = false;
    st.dir = 0;
    return 0.0f;
  }

  if (st.dir == 0) st.dir = want;
  return pidOut;
}

float slewPwm(float previous, float commanded, float dt, float rampUp,
              float rampDown) {
  if (dt <= 0.0f) return previous;
  const float delta = commanded - previous;
  const float towardZero = (commanded * previous < 0.0f) ||
                           (std::fabs(commanded) < std::fabs(previous));
  float maxDelta;
  if (towardZero) {
    maxDelta = (rampDown > 0.0f) ? rampDown * dt : 1e9f;
  } else {
    maxDelta = (rampUp > 0.0f) ? rampUp * dt : 1e9f;
  }
  return previous + clampf(delta, -maxDelta, maxDelta);
}

float lerpTable(const float* x, const float* y, int n, float xq) {
  if (!x || !y || n <= 0) return 0.0f;
  if (n == 1 || xq <= x[0]) return y[0];
  if (xq >= x[n - 1]) return y[n - 1];
  for (int i = 1; i < n; ++i) {
    if (xq <= x[i]) {
      const float span = x[i] - x[i - 1];
      if (std::fabs(span) < 1e-6f) return y[i];
      const float a = (xq - x[i - 1]) / span;
      return y[i - 1] + a * (y[i] - y[i - 1]);
    }
  }
  return y[n - 1];
}

float velocityAtPwm(float signedPwm, const float* pwm, const float* vel,
                    int n) {
  const float ap = std::fabs(signedPwm);
  float mag = std::fabs(lerpTable(pwm, vel, n, ap));
  if (signedPwm < 0.0f) mag = -mag;
  else if (signedPwm == 0.0f) mag = 0.0f;
  return mag;
}

float feedforwardPwm(float desiredVel, const float* pwm, const float* vel,
                     int n, float gain) {
  if (n < 1 || std::fabs(desiredVel) < 1e-4f || gain <= 0.0f) return 0.0f;
  const float av = std::fabs(desiredVel);
  // Invert vel(|pwm|) by scanning; vel table is signed, compare magnitudes.
  float bestPwm = pwm[n - 1];
  float absVelPrev = 0.0f;
  float pwmPrev = 0.0f;
  bool havePrev = false;
  for (int i = 0; i < n; ++i) {
    const float avp = std::fabs(vel[i]);
    if (!havePrev) {
      absVelPrev = avp;
      pwmPrev = pwm[i];
      havePrev = true;
      if (av <= avp) {
        bestPwm = pwm[i];
        break;
      }
      continue;
    }
    if (av <= avp) {
      const float span = avp - absVelPrev;
      const float a = (span > 1e-6f) ? (av - absVelPrev) / span : 1.0f;
      bestPwm = pwmPrev + a * (pwm[i] - pwmPrev);
      havePrev = true;
      break;
    }
    absVelPrev = avp;
    pwmPrev = pwm[i];
    bestPwm = pwm[i];
  }
  const float signedPwm = (desiredVel < 0.0f) ? -bestPwm : bestPwm;
  return signedPwm * gain;
}

SteerRecommendations recommendSteering(const SteeringCharacterization& c,
                                       float currentKp, float currentKd,
                                       float currentDeadband, float maxPwm) {
  SteerRecommendations r;
  const bool haveData = c.nPoints > 0 || c.minStartLeft > 0.0f ||
                        c.minStartRight > 0.0f;
  if (c.status == SteerCharStatus::INVALID || !haveData) return r;
  if (c.status != SteerCharStatus::OK && c.status != SteerCharStatus::FAILED)
    return r;
  r.valid = true;
  r.startLeft = c.minStartLeft;
  r.startRight = c.minStartRight;
  r.holdLeft = c.holdLeft > 0.0f ? c.holdLeft : c.minStartLeft * 0.7f;
  r.holdRight = c.holdRight > 0.0f ? c.holdRight : c.minStartRight * 0.7f;

  const float start = std::max(c.minStartLeft, c.minStartRight);
  const float approach = 10.0f;
  // Conservative P: reach start PWM at ~approach-threshold error, then
  // back off. Never recommend more than 8 or the current Kp * 1.25.
  float far = (approach > 0.1f) ? (start / approach) : currentKp;
  far = clampf(far, 1.0f, std::min(8.0f, std::max(currentKp, 1.0f) * 1.25f));
  r.farP = far;
  r.nearP = far * 0.55f;
  r.holdP = far * 0.25f;
  r.kp = r.nearP;
  r.ki = 0.0f;  // keep I off until the model is trusted
  r.kd = clampf(currentKd, 0.0f, 0.2f);
  const float backlash =
      std::max(std::fabs(c.backlashLeft), std::fabs(c.backlashRight));
  r.deadband = std::max(currentDeadband, std::max(0.5f, backlash * 0.4f));
  r.settlingThreshold = std::max(1.5f, r.deadband * 2.0f);
  r.nearTargetGain = r.nearP;
  const float vmax = std::max(std::fabs(c.maxVelLeft), std::fabs(c.maxVelRight));
  r.maxVelocity = vmax > 0.1f ? vmax * 0.7f : 8.0f;
  r.pwmSlew = clampf(r.maxVelocity * 12.0f, 40.0f, 250.0f);
  if (maxPwm > 0.0f) {
    r.startLeft = std::min(r.startLeft, maxPwm);
    r.startRight = std::min(r.startRight, maxPwm);
  }
  return r;
}

float relativeChange(float baseline, float current) {
  if (std::fabs(baseline) < 1e-6f) return 0.0f;
  return (current - baseline) / std::fabs(baseline);
}

SteerDeviation compareCharacterization(const SteeringCharacterization& baseline,
                                       const SteeringCharacterization& current) {
  SteerDeviation d;
  d.comparable = baseline.status == SteerCharStatus::OK &&
                 (current.status == SteerCharStatus::OK ||
                  current.status == SteerCharStatus::FAILED) &&
                 baseline.magic == kSteerCharMagic;
  if (!d.comparable) return d;
  d.minPwmLeftPct = relativeChange(baseline.minStartLeft, current.minStartLeft);
  d.minPwmRightPct =
      relativeChange(baseline.minStartRight, current.minStartRight);
  d.maxVelLeftPct = relativeChange(baseline.maxVelLeft, current.maxVelLeft);
  d.maxVelRightPct = relativeChange(baseline.maxVelRight, current.maxVelRight);
  d.holdLeftPct = relativeChange(baseline.holdLeft, current.holdLeft);
  d.holdRightPct = relativeChange(baseline.holdRight, current.holdRight);
  d.backlashLeftPct =
      relativeChange(baseline.backlashLeft, current.backlashLeft);
  d.backlashRightPct =
      relativeChange(baseline.backlashRight, current.backlashRight);
  return d;
}

float steeringHealthScore(float rmsError, bool hunting, bool calValid,
                          float startPwm, float baselineStartPwm) {
  float score = 100.0f;
  if (!calValid) score -= 40.0f;
  score -= clampf(rmsError * 8.0f, 0.0f, 40.0f);
  if (hunting) score -= 20.0f;
  if (baselineStartPwm > 1.0f && startPwm > 0.0f) {
    const float dev = std::fabs(relativeChange(baselineStartPwm, startPwm));
    score -= clampf(dev * 40.0f, 0.0f, 20.0f);
  }
  return clampf(score, 0.0f, 100.0f);
}

int countSignChanges(const float* v, int n, float dead) {
  if (!v || n < 2) return 0;
  int changes = 0;
  int last = 0;
  for (int i = 0; i < n; ++i) {
    const int s = (v[i] > dead) ? 1 : (v[i] < -dead) ? -1 : 0;
    if (s == 0) continue;
    if (last != 0 && s != last) changes++;
    last = s;
  }
  return changes;
}

}  // namespace vcm
