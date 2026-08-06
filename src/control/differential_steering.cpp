#include "control/differential_steering.h"

#include <algorithm>
#include <cmath>

namespace vcm {

static float clampf(float v, float lo, float hi) {
  return std::max(lo, std::min(hi, v));
}

DiffOutput DifferentialSteering::update(const VehicleModel& model,
                                        float targetSpeed,
                                        float steeringAngleDeg,
                                        float steeringDemandPct, float dt) {
  DiffOutput out;
  out.leftSpeed = targetSpeed;
  out.rightSpeed = targetSpeed;

  const float speedAbs = std::fabs(targetSpeed);
  const float angleAbs = std::fabs(steeringAngleDeg);

  bool active = config_.enabled &&
                config_.algorithm != DiffAlgorithm::OFF &&
                angleAbs >= config_.activationAngleDeg &&
                speedAbs >= config_.minSpeedMps &&
                speedAbs <= config_.maxSpeedMps &&
                (targetSpeed >= 0.0f || config_.reverseEnabled);

  // Ramp the authority up/down so the effect never jumps suddenly
  const float target = active ? 1.0f : 0.0f;
  const float step = (config_.rampRatePctPerS / 100.0f) * dt;
  authority_ = clampf(authority_ + clampf(target - authority_, -step, step),
                      0.0f, 1.0f);
  if (authority_ <= 0.001f) return out;

  // Effect scale 0..1 between activation and full-effect angles
  float effect = 0.0f;
  if (config_.fullEffectAngleDeg > config_.activationAngleDeg) {
    effect = (angleAbs - config_.activationAngleDeg) /
             (config_.fullEffectAngleDeg - config_.activationAngleDeg);
  } else {
    effect = 1.0f;
  }
  effect = clampf(effect, 0.0f, 1.0f);

  // Aggressive turn assist: progressively raise authority near full lock
  if (config_.aggressiveAssist &&
      steeringDemandPct > config_.aggressiveStartPct) {
    const float t = clampf((steeringDemandPct - config_.aggressiveStartPct) /
                               (100.0f - config_.aggressiveStartPct),
                           0.0f, 1.0f);
    effect *= 1.0f + t * (config_.aggressiveGain - 1.0f);
  }
  effect *= authority_;

  float leftRatio = 1.0f, rightRatio = 1.0f;
  if (config_.algorithm == DiffAlgorithm::GEOMETRY) {
    model.ackermannRatios(steeringAngleDeg, leftRatio, rightRatio);
    // Blend between no-effect (1,1) and full geometric ratios
    leftRatio = 1.0f + (leftRatio - 1.0f) * effect;
    rightRatio = 1.0f + (rightRatio - 1.0f) * effect;
  } else {  // SIMPLE percentage strategy
    const float reduction = (config_.insideReductionPct / 100.0f) * effect;
    const float boost = (config_.outsideBoostPct / 100.0f) * effect;
    const float maxDiff = config_.maxDifferentialPct / 100.0f;
    const float inner = clampf(1.0f - reduction, 1.0f - maxDiff, 1.0f);
    const float outer = 1.0f + std::min(boost, maxDiff);
    if (steeringAngleDeg < 0.0f) {  // left turn: left = inner
      leftRatio = inner;
      rightRatio = outer;
    } else {
      leftRatio = outer;
      rightRatio = inner;
    }
  }

  out.leftSpeed = targetSpeed * leftRatio;
  out.rightSpeed = targetSpeed * rightRatio;

  // Optional inside wheel braking when its target collapses to near zero
  if (config_.allowInsideBrake) {
    const float thr = (config_.insideBrakeThresholdPct / 100.0f) * speedAbs;
    const bool leftInner = steeringAngleDeg < 0.0f;
    float& inside = leftInner ? out.leftSpeed : out.rightSpeed;
    if (std::fabs(inside) < thr) {
      inside = 0.0f;
      out.insideBraking = true;
    }
  }

  out.bias = (speedAbs > 0.01f)
                 ? clampf((out.rightSpeed - out.leftSpeed) / speedAbs, -1.0f,
                          1.0f)
                 : 0.0f;
  return out;
}

}  // namespace vcm
