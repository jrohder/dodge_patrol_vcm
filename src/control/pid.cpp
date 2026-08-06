#include "control/pid.h"

#include <algorithm>
#include <cmath>

namespace vcm {

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

float Pid::update(float setpoint, float measured, float dt) {
  if (dt <= 0.0f) return output_;
  const float error = setpoint - measured;

  pTerm_ = gains_.kp * error;

  integral_ += error * dt;
  // Anti-windup: clamp the integral so ki*integral stays within iMax
  if (gains_.ki > 0.0f) {
    const float lim = gains_.iMax / gains_.ki;
    integral_ = clampf(integral_, -lim, lim);
  } else {
    integral_ = 0.0f;
  }
  iTerm_ = gains_.ki * integral_;

  float dRaw = 0.0f;
  if (!first_) dRaw = (error - lastError_) / dt;
  // First-order low-pass on the derivative to reject sensor noise
  const float alpha =
      (gains_.dFilterHz > 0.0f)
          ? clampf(dt * gains_.dFilterHz * 6.2831853f /
                       (1.0f + dt * gains_.dFilterHz * 6.2831853f),
                   0.0f, 1.0f)
          : 1.0f;
  dFiltered_ += alpha * (dRaw - dFiltered_);
  dTerm_ = gains_.kd * dFiltered_;

  lastError_ = error;
  first_ = false;

  output_ = clampf(pTerm_ + iTerm_ + dTerm_, -gains_.outMax, gains_.outMax);
  return output_;
}

void Pid::reset() {
  integral_ = 0.0f;
  lastError_ = 0.0f;
  dFiltered_ = 0.0f;
  pTerm_ = iTerm_ = dTerm_ = output_ = 0.0f;
  first_ = true;
}

}  // namespace vcm
