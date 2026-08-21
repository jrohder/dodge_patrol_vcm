#include "control/pid.h"

#include <algorithm>
#include <cmath>

namespace vcm {

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

float Pid::update(float setpoint, float measured, float dt) {
  return update(setpoint, measured, dt, StepOpts{});
}

float Pid::update(float setpoint, float measured, float dt,
                  const StepOpts& opt) {
  if (dt <= 0.0f) return output_;
  const float error = setpoint - measured;

  pTerm_ = gains_.kp * error;

  if (opt.allowIntegral && gains_.ki > 0.0f) {
    integral_ += error * dt;
    const float lim = gains_.iMax / gains_.ki;
    integral_ = clampf(integral_, -lim, lim);
  } else if (gains_.ki <= 0.0f) {
    integral_ = 0.0f;
  }
  iTerm_ = gains_.ki * integral_;

  if (opt.useMeasDerivative) {
    dTerm_ = -gains_.kd * opt.measDerivative;
    dFiltered_ = opt.measDerivative;
  } else {
    float dRaw = 0.0f;
    if (!first_) dRaw = (error - lastError_) / dt;
    const float alpha =
        (gains_.dFilterHz > 0.0f)
            ? clampf(dt * gains_.dFilterHz * 6.2831853f /
                         (1.0f + dt * gains_.dFilterHz * 6.2831853f),
                     0.0f, 1.0f)
            : 1.0f;
    dFiltered_ += alpha * (dRaw - dFiltered_);
    dTerm_ = gains_.kd * dFiltered_;
  }

  lastError_ = error;
  first_ = false;

  const float unclamped = pTerm_ + iTerm_ + dTerm_;
  output_ = clampf(unclamped, -gains_.outMax, gains_.outMax);
  saturated_ = output_ != unclamped;

  // Back-calculate anti-windup: do not keep integrating into a stop.
  if (saturated_ && opt.allowIntegral && gains_.ki > 0.0f) {
    integral_ -= error * dt;
    const float lim = gains_.iMax / gains_.ki;
    integral_ = clampf(integral_, -lim, lim);
    iTerm_ = gains_.ki * integral_;
    output_ = clampf(pTerm_ + iTerm_ + dTerm_, -gains_.outMax, gains_.outMax);
  }
  return output_;
}

void Pid::reset() {
  integral_ = 0.0f;
  lastError_ = 0.0f;
  dFiltered_ = 0.0f;
  pTerm_ = iTerm_ = dTerm_ = output_ = 0.0f;
  first_ = true;
  saturated_ = false;
}

}  // namespace vcm
