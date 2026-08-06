/**
 * @file vehicle_model.h
 * @brief Geometric/physical vehicle model. All values come from configuration.
 *
 * Pure logic - no hardware dependencies; unit-tested on the host.
 */
#pragma once

namespace vcm {

/// Complete geometric description of the vehicle, loaded from configuration.
struct VehicleModel {
  float wheelDiameterM = 0.25f;
  float wheelbaseM = 0.60f;
  float trackWidthM = 0.50f;
  float massKg = 30.0f;         ///< reserved for future dynamics
  float cogHeightM = 0.20f;     ///< reserved for future dynamics
  float gearRatio = 1.0f;       ///< motor revs per wheel rev (sensor on wheel = 1)
  float countsPerRev = 12.0f;   ///< hall counts per WHEEL revolution
  float maxSteeringAngleDeg = 30.0f;  ///< road-wheel angle at full lock

  float wheelCircumferenceM() const;

  /// Convert hall pulse frequency (Hz) to wheel RPM.
  float freqToRpm(float freqHz) const;

  /// Convert hall pulse frequency (Hz) to ground speed (m/s).
  float freqToSpeed(float freqHz) const;

  /// Convert pulse count to distance traveled (m).
  float countsToDistance(float counts) const;

  /// Turning radius (m) of vehicle centerline for a road-wheel angle (deg).
  /// Returns a very large value for near-zero angles (straight line).
  float turnRadiusM(float steeringAngleDeg) const;

  /**
   * @brief Ackermann rear-wheel speed ratios for a given steering angle.
   *
   * With turn radius R, the inner rear wheel travels (R - track/2)/R of the
   * centerline speed and the outer (R + track/2)/R.
   *
   * @param steeringAngleDeg signed road-wheel angle; negative = left turn
   * @param leftRatio  out: left wheel speed / vehicle speed
   * @param rightRatio out: right wheel speed / vehicle speed
   */
  void ackermannRatios(float steeringAngleDeg, float& leftRatio,
                       float& rightRatio) const;
};

}  // namespace vcm
