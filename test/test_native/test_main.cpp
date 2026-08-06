/**
 * @file test_main.cpp
 * @brief Host-side unit tests for the pure control/math logic.
 *
 * Run with: pio test -e native
 */
#include <unity.h>

#include "control/differential_steering.h"
#include "control/pid.h"
#include "control/speed_calculator.h"
#include "control/vehicle_model.h"
#include "drivers/uart/crc16.h"

using namespace vcm;

// ------------------------------------------------------------------ CRC16

static void test_crc16_known_vector() {
  // CRC16-CCITT (0x1021, init 0xFFFF) of "123456789" = 0x29B1
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(data, sizeof(data)));
}

static void test_crc16_detects_corruption() {
  uint8_t data[16];
  for (int i = 0; i < 16; ++i) data[i] = (uint8_t)i;
  const uint16_t good = crc16(data, sizeof(data));
  data[7] ^= 0x01;
  TEST_ASSERT_NOT_EQUAL(good, crc16(data, sizeof(data)));
}

// ------------------------------------------------------------------ PID

static void test_pid_proportional() {
  Pid pid;
  Pid::Gains g;
  g.kp = 2.0f;
  g.outMax = 100.0f;
  pid.setGains(g);
  const float out = pid.update(10.0f, 0.0f, 0.005f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, out);
}

static void test_pid_output_clamped() {
  Pid pid;
  Pid::Gains g;
  g.kp = 100.0f;
  g.outMax = 50.0f;
  pid.setGains(g);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 50.0f, pid.update(100.0f, 0.0f, 0.005f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -50.0f, pid.update(-100.0f, 0.0f, 0.005f));
}

static void test_pid_integral_antiwindup() {
  Pid pid;
  Pid::Gains g;
  g.ki = 10.0f;
  g.iMax = 5.0f;
  g.outMax = 100.0f;
  pid.setGains(g);
  for (int i = 0; i < 10000; ++i) pid.update(100.0f, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(pid.iTerm() <= 5.01f);
}

static void test_pid_converges() {
  // Simple first-order plant: measured moves toward output
  Pid pid;
  Pid::Gains g;
  g.kp = 5.0f;
  g.ki = 2.0f;
  g.outMax = 100.0f;
  g.iMax = 50.0f;
  pid.setGains(g);
  float plant = 0.0f;
  for (int i = 0; i < 2000; ++i) {
    const float out = pid.update(50.0f, plant, 0.005f);
    plant += out * 0.005f;
  }
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 50.0f, plant);
}

// ------------------------------------------------------- vehicle model

static VehicleModel makeModel() {
  VehicleModel m;
  m.wheelDiameterM = 0.25f;
  m.wheelbaseM = 0.60f;
  m.trackWidthM = 0.50f;
  m.countsPerRev = 12.0f;
  return m;
}

static void test_model_freq_to_rpm() {
  const VehicleModel m = makeModel();
  // 12 counts/rev at 12 Hz = 1 rev/s = 60 RPM
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, m.freqToRpm(12.0f));
}

static void test_model_freq_to_speed() {
  const VehicleModel m = makeModel();
  // 1 rev/s * circumference (pi*0.25) = 0.7854 m/s
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7854f, m.freqToSpeed(12.0f));
}

static void test_model_counts_to_distance() {
  const VehicleModel m = makeModel();
  // 120 counts = 10 revs = 10 * pi * 0.25 m
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.854f, m.countsToDistance(120.0f));
}

static void test_model_turn_radius() {
  const VehicleModel m = makeModel();
  // R = wheelbase / tan(angle); at 30 deg: 0.6/0.5774 = 1.039 m
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.039f, m.turnRadiusM(30.0f));
  TEST_ASSERT_TRUE(m.turnRadiusM(0.0f) > 1e5f);  // straight
}

static void test_model_ackermann_ratios() {
  const VehicleModel m = makeModel();
  float l, r;
  m.ackermannRatios(0.0f, l, r);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, l);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, r);

  // Right turn (positive): right wheel is inner (slower)
  m.ackermannRatios(30.0f, l, r);
  TEST_ASSERT_TRUE(r < 1.0f);
  TEST_ASSERT_TRUE(l > 1.0f);
  // R=1.039, half-track 0.25 -> inner = (1.039-0.25)/1.039 = 0.7594
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.7594f, r);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.2406f, l);

  // Left turn mirrors
  float l2, r2;
  m.ackermannRatios(-30.0f, l2, r2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, r, l2);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, l, r2);
}

// ------------------------------------------------------- speed calculator

static void test_speed_calculator_basic() {
  SpeedCalculator sc;
  const VehicleModel m = makeModel();
  WheelInput lw, rw;
  lw.freqHz = 12.0f;
  lw.direction = 1;
  lw.count = 0;
  rw = lw;
  SpeedOutput out = sc.update(m, lw, rw, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7854f, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7854f, out.vehicleSpeed);

  // Reverse direction gives negative speed
  lw.direction = rw.direction = -1;
  out = sc.update(m, lw, rw, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(out.vehicleSpeed < 0.0f);
}

static void test_speed_calculator_odometer() {
  SpeedCalculator sc;
  const VehicleModel m = makeModel();
  WheelInput lw, rw;
  lw.direction = rw.direction = 1;
  lw.count = rw.count = 0;
  sc.update(m, lw, rw, 0.0f, 0.01f);  // prime counters
  lw.count = rw.count = 120;          // 10 revolutions
  const SpeedOutput out = sc.update(m, lw, rw, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.854f, out.odometerM);
}

static void test_speed_calculator_slip() {
  SpeedCalculator sc;
  sc.setSlipThreshold(0.3f);
  const VehicleModel m = makeModel();
  WheelInput lw, rw;
  lw.direction = rw.direction = 1;
  lw.freqHz = 24.0f;  // left much faster than right, going straight
  rw.freqHz = 6.0f;
  const SpeedOutput out = sc.update(m, lw, rw, 0.0f, 0.01f);
  TEST_ASSERT_TRUE(out.slipDetected);
}

// ------------------------------------------------------- differential

static DiffConfig diffBase() {
  DiffConfig c;
  c.enabled = true;
  c.algorithm = DiffAlgorithm::SIMPLE;
  c.activationAngleDeg = 3.0f;
  c.fullEffectAngleDeg = 25.0f;
  c.insideReductionPct = 60.0f;
  c.outsideBoostPct = 10.0f;
  c.maxDifferentialPct = 80.0f;
  c.minSpeedMps = 0.1f;
  c.maxSpeedMps = 10.0f;
  c.rampRatePctPerS = 10000.0f;  // effectively instant for tests
  return c;
}

static void test_diff_disabled_passthrough() {
  DifferentialSteering d;
  DiffConfig c = diffBase();
  c.enabled = false;
  d.setConfig(c);
  const VehicleModel m = makeModel();
  const DiffOutput out = d.update(m, 2.0f, 20.0f, 60.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.rightSpeed);
}

static void test_diff_simple_right_turn() {
  DifferentialSteering d;
  d.setConfig(diffBase());
  const VehicleModel m = makeModel();
  DiffOutput out;
  for (int i = 0; i < 50; ++i)  // let authority ramp settle
    out = d.update(m, 2.0f, 25.0f, 80.0f, 0.01f);
  // Right turn: right = inner (reduced), left = outer (boosted)
  TEST_ASSERT_TRUE(out.rightSpeed < 2.0f);
  TEST_ASSERT_TRUE(out.leftSpeed >= 2.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f * 0.4f, out.rightSpeed);  // 60% reduction
}

static void test_diff_below_activation_angle() {
  DifferentialSteering d;
  d.setConfig(diffBase());
  const VehicleModel m = makeModel();
  const DiffOutput out = d.update(m, 2.0f, 1.0f, 5.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.rightSpeed);
}

static void test_diff_geometry_matches_ackermann() {
  DifferentialSteering d;
  DiffConfig c = diffBase();
  c.algorithm = DiffAlgorithm::GEOMETRY;
  d.setConfig(c);
  const VehicleModel m = makeModel();
  DiffOutput out;
  for (int i = 0; i < 50; ++i) out = d.update(m, 2.0f, 25.0f, 80.0f, 0.01f);
  float lr, rr;
  m.ackermannRatios(25.0f, lr, rr);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f * lr, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f * rr, out.rightSpeed);
}

static void test_diff_reverse_disabled_by_default() {
  DifferentialSteering d;
  d.setConfig(diffBase());
  const VehicleModel m = makeModel();
  DiffOutput out;
  for (int i = 0; i < 50; ++i) out = d.update(m, -1.0f, 25.0f, 80.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, out.rightSpeed);
}

static void test_diff_inside_brake() {
  DifferentialSteering d;
  DiffConfig c = diffBase();
  c.insideReductionPct = 100.0f;
  c.maxDifferentialPct = 100.0f;
  c.allowInsideBrake = true;
  c.insideBrakeThresholdPct = 20.0f;
  d.setConfig(c);
  const VehicleModel m = makeModel();
  DiffOutput out;
  for (int i = 0; i < 50; ++i) out = d.update(m, 2.0f, 30.0f, 100.0f, 0.01f);
  TEST_ASSERT_TRUE(out.insideBraking);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.rightSpeed);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_known_vector);
  RUN_TEST(test_crc16_detects_corruption);
  RUN_TEST(test_pid_proportional);
  RUN_TEST(test_pid_output_clamped);
  RUN_TEST(test_pid_integral_antiwindup);
  RUN_TEST(test_pid_converges);
  RUN_TEST(test_model_freq_to_rpm);
  RUN_TEST(test_model_freq_to_speed);
  RUN_TEST(test_model_counts_to_distance);
  RUN_TEST(test_model_turn_radius);
  RUN_TEST(test_model_ackermann_ratios);
  RUN_TEST(test_speed_calculator_basic);
  RUN_TEST(test_speed_calculator_odometer);
  RUN_TEST(test_speed_calculator_slip);
  RUN_TEST(test_diff_disabled_passthrough);
  RUN_TEST(test_diff_simple_right_turn);
  RUN_TEST(test_diff_below_activation_angle);
  RUN_TEST(test_diff_geometry_matches_ackermann);
  RUN_TEST(test_diff_reverse_disabled_by_default);
  RUN_TEST(test_diff_inside_brake);
  return UNITY_END();
}
