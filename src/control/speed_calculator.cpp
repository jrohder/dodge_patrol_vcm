#include "control/speed_calculator.h"

#include <cmath>

namespace vcm {

SpeedOutput SpeedCalculator::update(const VehicleModel& model,
                                    const WheelInput& left,
                                    const WheelInput& right,
                                    float steeringAngleDeg, float dt) {
  SpeedOutput out;

  const float lDir = (left.direction == 0) ? 0.0f : (float)left.direction;
  const float rDir = (right.direction == 0) ? 0.0f : (float)right.direction;

  out.leftSpeed = model.freqToSpeed(left.freqHz) * leftFactor_ * lDir;
  out.rightSpeed = model.freqToSpeed(right.freqHz) * rightFactor_ * rDir;
  out.leftRpm = model.freqToRpm(left.freqHz) * leftFactor_ * lDir;
  out.rightRpm = model.freqToRpm(right.freqHz) * rightFactor_ * rDir;

  out.vehicleSpeed = 0.5f * (out.leftSpeed + out.rightSpeed);

  // Distance from pulse counters (robust across sample jitter)
  if (haveCounts_) {
    const uint32_t dl = left.count - lastLeftCount_;
    const uint32_t dr = right.count - lastRightCount_;
    // Counters only increase; direction sign applied for odometer purposes
    const float dist = 0.5f * (model.countsToDistance((float)dl) * leftFactor_ +
                               model.countsToDistance((float)dr) * rightFactor_);
    odometer_ += dist;
    trip_ += dist;
  }
  lastLeftCount_ = left.count;
  lastRightCount_ = right.count;
  haveCounts_ = true;
  out.odometerM = odometer_;
  out.tripM = trip_;

  // Acceleration from filtered speed derivative
  if (dt > 0.0f) {
    const float rawAccel = (out.vehicleSpeed - lastVehicleSpeed_) / dt;
    accelFiltered_ += 0.2f * (rawAccel - accelFiltered_);
    out.acceleration = accelFiltered_;
  }
  lastVehicleSpeed_ = out.vehicleSpeed;

  // Slip: compare measured L/R difference against geometric prediction.
  // Large unexplained differences indicate slip, wheel lift or sensor failure.
  if (slipThreshold_ > 0.0f) {
    float lr, rr;
    model.ackermannRatios(steeringAngleDeg, lr, rr);
    const float predictedDiff = out.vehicleSpeed * (lr - rr);
    const float measuredDiff = out.leftSpeed - out.rightSpeed;
    out.slipDetected =
        std::fabs(measuredDiff - predictedDiff) > slipThreshold_;
  }

  return out;
}

}  // namespace vcm
