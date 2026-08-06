#include "control/vehicle_model.h"

#include <cmath>

namespace vcm {

static constexpr float kPi = 3.14159265358979f;
static constexpr float kMinAngleDeg = 0.05f;

float VehicleModel::wheelCircumferenceM() const {
  return wheelDiameterM * kPi;
}

float VehicleModel::freqToRpm(float freqHz) const {
  if (countsPerRev <= 0.0f) return 0.0f;
  return freqHz * 60.0f / countsPerRev;
}

float VehicleModel::freqToSpeed(float freqHz) const {
  return freqToRpm(freqHz) / 60.0f * wheelCircumferenceM();
}

float VehicleModel::countsToDistance(float counts) const {
  if (countsPerRev <= 0.0f) return 0.0f;
  return counts / countsPerRev * wheelCircumferenceM();
}

float VehicleModel::turnRadiusM(float steeringAngleDeg) const {
  const float a = std::fabs(steeringAngleDeg);
  if (a < kMinAngleDeg) return 1e6f;
  return wheelbaseM / std::tan(a * kPi / 180.0f);
}

void VehicleModel::ackermannRatios(float steeringAngleDeg, float& leftRatio,
                                   float& rightRatio) const {
  const float r = turnRadiusM(steeringAngleDeg);
  if (r >= 1e5f) {
    leftRatio = rightRatio = 1.0f;
    return;
  }
  const float half = trackWidthM * 0.5f;
  float inner = (r - half) / r;
  float outer = (r + half) / r;
  if (inner < 0.0f) inner = 0.0f;  // radius smaller than half-track
  if (steeringAngleDeg < 0.0f) {   // left turn: left wheel is inner
    leftRatio = inner;
    rightRatio = outer;
  } else {
    leftRatio = outer;
    rightRatio = inner;
  }
}

}  // namespace vcm
