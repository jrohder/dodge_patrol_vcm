/**
 * @file steering_compensation.h
 * @brief PWM hysteresis, gain scheduling, slew limiting, feed-forward lookup.
 *
 * These helpers extend the existing PID; they are not a second controller.
 * All functions are host-testable (no hardware).
 */
#pragma once

#include "control/steering_types.h"

namespace vcm {

/// First PWM that produced repeatable motion in the same direction.
bool movementRepeatable(float delta1, float delta2, float threshold);

/// Select scheduled P gain. A configured gain of 0 inherits fallbackKp
/// so an OTA that adds scheduling does not change behaviour until tuned.
float scheduledKp(float absError, float farP, float nearP, float holdP,
                  float approachThresh, float settleThresh, float fallbackKp);

SteerControlState classifySteerState(float absError, float pwm, bool atLimit,
                                     bool fault, bool calibration,
                                     bool outputEnabled, float approachThresh,
                                     float settleThresh, float holdThresh,
                                     float deadband);

/**
 * @brief Start/hold PWM with hysteresis.
 *
 * Stopped: output stays 0 until |pidOut| reaches the directional start PWM
 * (we do NOT boost a small command up to start PWM).
 * Moving: output is floored at hold PWM while error remains outside the
 * hold band; dropping below hold PWM with small error stops the motor.
 */
float applyPwmHysteresis(float pidOut, float error, const SteerCompConfig& cfg,
                         SteerCompState& st);

/// Rate-limit PWM. rampUp/rampDown are % per second; 0 = unlimited.
float slewPwm(float previous, float commanded, float dt, float rampUp,
              float rampDown);

/// Linear interpolate PWM required for a signed actuator velocity (%/s).
/// Tables store |PWM| in pwm[] and signed velocity in vel[].
float feedforwardPwm(float desiredVel, const float* pwm, const float* vel,
                     int n, float gain);

/// Inverse: velocity expected at a given signed PWM.
float velocityAtPwm(float signedPwm, const float* pwm, const float* vel,
                    int n);

/// Linear interpolate y(x). x must be monotonic increasing. n >= 1.
float lerpTable(const float* x, const float* y, int n, float xq);

/// Conservative PID / PWM recommendations from a completed characterization.
/// Never overwrites live config by itself.
SteerRecommendations recommendSteering(const SteeringCharacterization& c,
                                       float currentKp, float currentKd,
                                       float currentDeadband, float maxPwm);

/// Relative change (current - baseline) / |baseline|. 0 if baseline is 0.
float relativeChange(float baseline, float current);

SteerDeviation compareCharacterization(const SteeringCharacterization& baseline,
                                       const SteeringCharacterization& current);

/// Simple 0..100 health indicator. Raw measurements remain authoritative.
float steeringHealthScore(float rmsError, bool hunting, bool calValid,
                          float startPwm, float baselineStartPwm);

/// Count sign changes in a sequence (error or PWM). Used for hunting.
int countSignChanges(const float* v, int n, float dead);

}  // namespace vcm
