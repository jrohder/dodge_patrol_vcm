/**
 * @file pid.h
 * @brief Generic PID controller with anti-windup and derivative filtering.
 *
 * Pure logic - no hardware dependencies; unit-tested on the host.
 */
#pragma once

namespace vcm {

class Pid {
 public:
  struct Gains {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float iMax = 100.0f;    ///< integral term clamp (absolute)
    float outMax = 100.0f;  ///< output clamp (absolute)
    float dFilterHz = 20.0f;  ///< derivative low-pass cutoff
  };

  void setGains(const Gains& g) { gains_ = g; }
  const Gains& gains() const { return gains_; }

  /**
   * @brief Run one PID step.
   * @param setpoint desired value
   * @param measured actual value
   * @param dt seconds since last update (must be > 0)
   * @return controller output, clamped to +/- outMax
   */
  float update(float setpoint, float measured, float dt);

  void reset();

  float pTerm() const { return pTerm_; }
  float iTerm() const { return iTerm_; }
  float dTerm() const { return dTerm_; }
  float output() const { return output_; }
  float error() const { return lastError_; }

 private:
  Gains gains_;
  float integral_ = 0.0f;
  float lastError_ = 0.0f;
  float dFiltered_ = 0.0f;
  float pTerm_ = 0.0f, iTerm_ = 0.0f, dTerm_ = 0.0f, output_ = 0.0f;
  bool first_ = true;
};

}  // namespace vcm
