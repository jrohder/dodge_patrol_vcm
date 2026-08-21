/**
 * @file steering_types.h
 * @brief Shared steering diagnostic / characterization types.
 *
 * Pure data (no Arduino, no heap). Packed samples keep the 200 Hz
 * 60-second recorder within ESP32 RAM/PSRAM limits. Host unit tests
 * cover packing, PWM hysteresis, gain scheduling and characterization
 * analysis.
 */
#pragma once

#include <cstdint>
#include <cstddef>
#include <cmath>

namespace vcm {

/// Closed-loop / wizard state stored in every diagnostic sample.
enum class SteerControlState : uint8_t {
  IDLE = 0,
  MOVING_LEFT,
  MOVING_RIGHT,
  APPROACHING,
  SETTLING,
  HOLDING,
  DEADBAND,
  LIMIT,
  FAULT,
  CALIBRATION,
};

const char* steerControlStateName(SteerControlState s);

/// Vertical markers on the diagnostic graph (not a second recorder).
enum class SteerDiagEvent : uint8_t {
  CAL_START = 0,
  CAL_FINISH,
  DIR_CHANGE,
  LIMIT_DETECTED,
  PWM_LIMIT,
  FAULT,
  MODE_CHANGE,
  CHAR_STEP,
  PWM_TEST,
  CHAR_FAIL,
  CHAR_DONE,
};

const char* steerDiagEventName(SteerDiagEvent e);

/// Unpacked engineering sample (used by the 200 Hz writer and tests).
struct SteeringDiagnosticSample {
  uint32_t timestamp_us = 0;
  float setpoint = 50.0f;
  float raw_feedback = 0.0f;       ///< ADC counts
  float filtered_feedback = 50.0f; ///< position %
  float error = 0.0f;
  float pwm = 0.0f;
  int8_t direction = 0;  ///< -1 left, 0 stopped, +1 right
  float p_term = 0.0f;
  float i_term = 0.0f;
  float d_term = 0.0f;
  float feedforward = 0.0f;
  float actuator_velocity = 0.0f;  ///< %/s
  SteerControlState control_state = SteerControlState::IDLE;
  bool output_enabled = false;
  float current_a = 0.0f;
  bool current_valid = false;
};

/// 28-byte packed sample: 12 000 × 28 = 336 kB (PSRAM when available).
struct SteeringDiagPacked {
  uint32_t t_us;
  int16_t sp_x100;
  uint16_t raw_adc;
  int16_t filt_x100;
  int16_t err_x100;
  int16_t pwm_x100;
  int16_t p_x100;
  int16_t i_x100;
  int16_t d_x100;
  int16_t ff_x100;
  int16_t vel_x100;
  int16_t cur_ma;  ///< INT16_MIN = current not available
  uint8_t flags;   ///< bit0-1 dir+1 (0,1,2), bit2 enabled, bit3 cur valid
  uint8_t state;
} __attribute__((packed));

static_assert(sizeof(SteeringDiagPacked) == 28,
              "SteeringDiagPacked must stay 28 bytes");

static constexpr int16_t kSteerCurrentInvalid = -32768;
static constexpr size_t kSteerCharMaxPoints = 24;
static constexpr size_t kSteerCharHistory = 8;
static constexpr uint32_t kSteerCharMagic = 0x53434831;  // 'SCH1'

inline int16_t steerQ100(float v) {
  if (v > 327.67f) return 32767;
  if (v < -327.68f) return -32768;
  return (int16_t)lrintf(v * 100.0f);
}

inline float steerFromQ100(int16_t v) { return (float)v * 0.01f; }

inline SteeringDiagPacked packSteerSample(const SteeringDiagnosticSample& s) {
  SteeringDiagPacked p{};
  p.t_us = s.timestamp_us;
  p.sp_x100 = steerQ100(s.setpoint);
  float raw = s.raw_feedback;
  if (raw < 0.0f) raw = 0.0f;
  if (raw > 65535.0f) raw = 65535.0f;
  p.raw_adc = (uint16_t)lrintf(raw);
  p.filt_x100 = steerQ100(s.filtered_feedback);
  p.err_x100 = steerQ100(s.error);
  p.pwm_x100 = steerQ100(s.pwm);
  p.p_x100 = steerQ100(s.p_term);
  p.i_x100 = steerQ100(s.i_term);
  p.d_x100 = steerQ100(s.d_term);
  p.ff_x100 = steerQ100(s.feedforward);
  p.vel_x100 = steerQ100(s.actuator_velocity);
  if (!s.current_valid) {
    p.cur_ma = kSteerCurrentInvalid;
  } else {
    const float ma = s.current_a * 1000.0f;
    if (ma > 32767.0f) p.cur_ma = 32767;
    else if (ma < -32767.0f) p.cur_ma = -32767;
    else p.cur_ma = (int16_t)lrintf(ma);
  }
  const uint8_t dirBits = (uint8_t)(s.direction + 1);  // 0,1,2
  p.flags = (uint8_t)((dirBits & 0x03) | (s.output_enabled ? 0x04 : 0) |
                      (s.current_valid ? 0x08 : 0));
  p.state = (uint8_t)s.control_state;
  return p;
}

inline SteeringDiagnosticSample unpackSteerSample(const SteeringDiagPacked& p) {
  SteeringDiagnosticSample s;
  s.timestamp_us = p.t_us;
  s.setpoint = steerFromQ100(p.sp_x100);
  s.raw_feedback = (float)p.raw_adc;
  s.filtered_feedback = steerFromQ100(p.filt_x100);
  s.error = steerFromQ100(p.err_x100);
  s.pwm = steerFromQ100(p.pwm_x100);
  s.p_term = steerFromQ100(p.p_x100);
  s.i_term = steerFromQ100(p.i_x100);
  s.d_term = steerFromQ100(p.d_x100);
  s.feedforward = steerFromQ100(p.ff_x100);
  s.actuator_velocity = steerFromQ100(p.vel_x100);
  s.current_valid = (p.flags & 0x08) != 0 && p.cur_ma != kSteerCurrentInvalid;
  s.current_a = s.current_valid ? (float)p.cur_ma * 0.001f : 0.0f;
  s.direction = (int8_t)((p.flags & 0x03) - 1);
  s.output_enabled = (p.flags & 0x04) != 0;
  s.control_state = (SteerControlState)p.state;
  return s;
}

enum class SteerCharStatus : uint8_t { INVALID = 0, OK = 1, FAILED = 2 };

/// Persistent actuator characterization (NVS). Independent of position cal.
struct SteeringCharacterization {
  uint32_t magic = kSteerCharMagic;
  uint8_t version = 1;
  SteerCharStatus status = SteerCharStatus::INVALID;
  uint8_t nPoints = 0;
  uint8_t reserved = 0;
  uint32_t timestamp = 0;  ///< millis()/1000 when completed
  char fwVersion[24] = {0};
  float minStartLeft = 0, minStartRight = 0;
  float holdLeft = 0, holdRight = 0;
  float preferredLeft = 0, preferredRight = 0;
  float maxTestLeft = 0, maxTestRight = 0;
  float maxVelLeft = 0, maxVelRight = 0;  ///< %/s, signed magnitude stored +
  float backlashLeft = 0, backlashRight = 0;
  float overshootPct = 0;
  float settlingTimeS = 0;
  float finalErrorPct = 0;
  uint8_t oscCount = 0;
  uint8_t dirReversals = 0;
  float pwm[kSteerCharMaxPoints] = {};
  float velLeft[kSteerCharMaxPoints] = {};   ///< %/s (negative = left)
  float velRight[kSteerCharMaxPoints] = {};  ///< %/s (positive = right)
  char failReason[48] = {0};
};

struct SteerRecommendations {
  float kp = 0, ki = 0, kd = 0;
  float farP = 0, nearP = 0, holdP = 0;
  float startLeft = 0, startRight = 0;
  float holdLeft = 0, holdRight = 0;
  float deadband = 0;
  float settlingThreshold = 0;
  float nearTargetGain = 0;
  float pwmSlew = 0;
  float maxVelocity = 0;
  bool valid = false;
};

struct SteerDeviation {
  float minPwmLeftPct = 0, minPwmRightPct = 0;
  float maxVelLeftPct = 0, maxVelRightPct = 0;
  float holdLeftPct = 0, holdRightPct = 0;
  float backlashLeftPct = 0, backlashRightPct = 0;
  bool comparable = false;
};

struct SteerCompConfig {
  float startLeft = 0, startRight = 0;
  float holdLeft = 0, holdRight = 0;
  float hysteresis = 0;
  float holdError = 1.0f;      ///< error below which hold PWM is dropped
  float reengageError = 2.0f;  ///< min |error| to start from rest
};

struct SteerCompState {
  bool moving = false;
  int8_t dir = 0;
};

struct SteerEventMarker {
  uint32_t t_us = 0;
  SteerDiagEvent type = SteerDiagEvent::MODE_CHANGE;
  int16_t value_x10 = 0;  ///< PWM*10 or generic payload
};

}  // namespace vcm
