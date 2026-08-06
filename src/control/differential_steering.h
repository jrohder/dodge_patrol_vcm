/**
 * @file differential_steering.h
 * @brief Pluggable differential steering module.
 *
 * Splits a single target speed into left/right wheel targets based on
 * steering angle. Two strategies are provided:
 *   SIMPLE   - percentage based inside-reduction / outside-boost
 *   GEOMETRY - Ackermann geometry from the vehicle model
 * plus an optional progressive "aggressive turn assist" layer and
 * configurable inside-wheel braking.
 *
 * Pure logic - no hardware dependencies; unit-tested on the host.
 */
#pragma once

#include "control/vehicle_model.h"

namespace vcm {

enum class DiffAlgorithm : int { OFF = 0, SIMPLE = 1, GEOMETRY = 2 };

struct DiffConfig {
  bool enabled = false;
  DiffAlgorithm algorithm = DiffAlgorithm::SIMPLE;
  float activationAngleDeg = 3.0f;   ///< no effect below this steering angle
  float fullEffectAngleDeg = 25.0f;  ///< full authority at/above this angle
  float maxDifferentialPct = 50.0f;  ///< SIMPLE: max total split authority
  float insideReductionPct = 60.0f;  ///< SIMPLE: max inside wheel reduction
  float outsideBoostPct = 10.0f;     ///< SIMPLE: max outside wheel boost
  bool allowInsideBrake = false;     ///< permit inside wheel to reverse/brake
  float insideBrakeThresholdPct = 10.0f;  ///< inside target below this -> brake
  float minSpeedMps = 0.2f;   ///< below this vehicle speed: no effect
  float maxSpeedMps = 10.0f;  ///< above this: no effect (stability)
  bool reverseEnabled = false;
  float rampRatePctPerS = 200.0f;  ///< authority slew rate (prevents jumps)
  bool aggressiveAssist = false;
  float aggressiveStartPct = 80.0f;  ///< steering demand where assist begins
  float aggressiveGain = 1.5f;       ///< authority multiplier at full lock
};

struct DiffOutput {
  float leftSpeed = 0.0f;   ///< m/s target
  float rightSpeed = 0.0f;  ///< m/s target
  float bias = 0.0f;        ///< -1..1 applied bias (for visualization)
  bool insideBraking = false;
};

class DifferentialSteering {
 public:
  void setConfig(const DiffConfig& c) { config_ = c; }
  const DiffConfig& config() const { return config_; }

  /**
   * @brief Split target speed into wheel targets.
   * @param model vehicle geometry (used by GEOMETRY algorithm)
   * @param targetSpeed signed vehicle target speed (m/s)
   * @param steeringAngleDeg signed road-wheel angle; negative = left
   * @param steeringDemandPct 0..100, |steering request| for assist layer
   * @param dt seconds since last call (for authority ramping)
   */
  DiffOutput update(const VehicleModel& model, float targetSpeed,
                    float steeringAngleDeg, float steeringDemandPct, float dt);

  void reset() { authority_ = 0.0f; }

 private:
  DiffConfig config_;
  float authority_ = 0.0f;  ///< 0..1 ramped effect strength
};

}  // namespace vcm
