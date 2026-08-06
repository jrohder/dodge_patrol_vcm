/**
 * @file test_main.cpp
 * @brief Host-side unit tests for pure control/math and the shared protocol.
 *
 * Run with: pio test -e native
 */
#include <unity.h>

#include <cstring>

#include "control/differential_steering.h"
#include "control/pid.h"
#include "control/speed_calculator.h"
#include "control/vehicle_model.h"
#include "proto/protocol.h"

using namespace vcm;
using namespace vcmproto;

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

// -------------------------------------------------------------- protocol

static void test_telemetry_payload_size() {
  TEST_ASSERT_EQUAL(63, (int)sizeof(TelemetryPayload));
  TEST_ASSERT_EQUAL(11, (int)HEADER_SIZE);
  TEST_ASSERT_EQUAL(2, (int)PROTOCOL_VERSION);
}

static void fillTelemetry(TelemetryPayload& t) {
  memset(&t, 0, sizeof(t));
  for (int i = 0; i < 6; ++i) t.rcUs[i] = (uint16_t)(1000 + i * 100);
  t.rcValidMask = 0x03;
  t.left.periodUs = 50000;  // 20 Hz
  t.left.count = 42;
  t.left.direction = 1;
  t.right.periodUs = 0;  // stopped
  t.right.count = 7;
  t.right.direction = 0;
  t.adc[0] = 2048;
  t.faultBits = FB_RC_LOST;
  t.statusBits = SB_TIMER_ISR_OK;
  t.fwMajor = 1;
  t.fwMinor = 2;
  t.uptimeMs = 123456;
}

static void test_encode_decode_roundtrip() {
  TelemetryPayload src;
  fillTelemetry(src);

  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_TELEMETRY, 7, 999999u, &src, sizeof(src));
  TEST_ASSERT_EQUAL(HEADER_SIZE + sizeof(TelemetryPayload) + CRC_SIZE, (int)len);

  Parser p;
  Packet out;
  bool got = false;
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(frame[i], out)) {
      got = true;
      break;
    }
  }
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL(PKT_TELEMETRY, out.header.packetType);
  TEST_ASSERT_EQUAL(7, out.header.seq);
  TEST_ASSERT_EQUAL(999999u, out.header.timestampUs);
  TEST_ASSERT_EQUAL(sizeof(TelemetryPayload), out.header.payloadLen);

  const TelemetryPayload* t = out.asTelemetry();
  TEST_ASSERT_NOT_NULL(t);
  TEST_ASSERT_EQUAL(src.left.periodUs, t->left.periodUs);
  TEST_ASSERT_EQUAL(src.left.count, t->left.count);
  TEST_ASSERT_EQUAL(src.left.direction, t->left.direction);
  TEST_ASSERT_EQUAL(src.right.periodUs, t->right.periodUs);
  TEST_ASSERT_EQUAL(src.rcUs[0], t->rcUs[0]);
  TEST_ASSERT_EQUAL(src.faultBits, t->faultBits);
  TEST_ASSERT_EQUAL(src.fwMajor, t->fwMajor);
  TEST_ASSERT_EQUAL(src.fwMinor, t->fwMinor);
  TEST_ASSERT_EQUAL(1, (int)p.stats().frames);
  TEST_ASSERT_EQUAL(0, (int)p.stats().crcErrors);
}

static void test_parser_rejects_bad_crc() {
  TelemetryPayload src;
  fillTelemetry(src);
  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_TELEMETRY, 1, 100u, &src, sizeof(src));
  frame[len - 1] ^= 0xFF;  // corrupt CRC

  Parser p;
  Packet out;
  bool got = false;
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(frame[i], out)) got = true;
  }
  TEST_ASSERT_FALSE(got);
  TEST_ASSERT_EQUAL(1, (int)p.stats().crcErrors);
  TEST_ASSERT_EQUAL(0, (int)p.stats().frames);
}

static void test_parser_rejects_wrong_version() {
  TelemetryPayload src;
  fillTelemetry(src);
  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_TELEMETRY, 1, 100u, &src, sizeof(src));
  frame[2] = 99;  // wrong version; CRC will also fail / version check first
  // Recompute CRC so we isolate the version check
  const uint16_t crc = crc16(frame, HEADER_SIZE + sizeof(src));
  memcpy(frame + HEADER_SIZE + sizeof(src), &crc, 2);

  Parser p;
  Packet out;
  bool got = false;
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(frame[i], out)) got = true;
  }
  TEST_ASSERT_FALSE(got);
  TEST_ASSERT_EQUAL(1, (int)p.stats().versionErrors);
}

static void test_parser_resync_after_garbage() {
  TelemetryPayload src;
  fillTelemetry(src);
  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_TELEMETRY, 3, 42u, &src, sizeof(src));

  Parser p;
  Packet out;
  // Feed garbage then a valid frame
  const uint8_t junk[] = {0x00, 0xFF, 0xAA, 0x00, 0x55, 0x12};
  for (uint8_t b : junk) p.feed(b, out);

  bool got = false;
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(frame[i], out)) got = true;
  }
  TEST_ASSERT_TRUE(got);
  TEST_ASSERT_EQUAL(3, out.header.seq);
  TEST_ASSERT_TRUE(p.stats().resyncs > 0);
  TEST_ASSERT_EQUAL(1, (int)p.stats().frames);
}

static void test_command_encode() {
  CommandPayload cmd = {};
  cmd.command = CMD_HEARTBEAT;
  uint8_t frame[MAX_FRAME];
  const size_t len =
      encode(frame, PKT_COMMAND, 1, 12345u, &cmd, sizeof(cmd));
  TEST_ASSERT_EQUAL(HEADER_SIZE + sizeof(CommandPayload) + CRC_SIZE, (int)len);
  TEST_ASSERT_EQUAL(SYNC1, frame[0]);
  TEST_ASSERT_EQUAL(SYNC2, frame[1]);
  TEST_ASSERT_EQUAL(PROTOCOL_VERSION, frame[2]);
  TEST_ASSERT_EQUAL(PKT_COMMAND, frame[3]);
  TEST_ASSERT_EQUAL(sizeof(CommandPayload), frame[4]);
}

static void test_period_to_frequency() {
  // Consumer-side conversion: freqHz = 1e6 / periodUs
  const uint32_t periodUs = 50000;  // 20 Hz
  const float freq = 1e6f / (float)periodUs;
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, freq);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f,
                           0.0f);  // periodUs==0 => stopped (handled by caller)
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
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, m.freqToRpm(12.0f));
}

static void test_model_freq_to_speed() {
  const VehicleModel m = makeModel();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7854f, m.freqToSpeed(12.0f));
}

static void test_model_counts_to_distance() {
  const VehicleModel m = makeModel();
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.854f, m.countsToDistance(120.0f));
}

static void test_model_turn_radius() {
  const VehicleModel m = makeModel();
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.039f, m.turnRadiusM(30.0f));
  TEST_ASSERT_TRUE(m.turnRadiusM(0.0f) > 1e5f);
}

static void test_model_ackermann_ratios() {
  const VehicleModel m = makeModel();
  float l, r;
  m.ackermannRatios(0.0f, l, r);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, l);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, r);

  m.ackermannRatios(30.0f, l, r);
  TEST_ASSERT_TRUE(r < 1.0f);
  TEST_ASSERT_TRUE(l > 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.7594f, r);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.2406f, l);

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
  sc.update(m, lw, rw, 0.0f, 0.01f);
  lw.count = rw.count = 120;
  const SpeedOutput out = sc.update(m, lw, rw, 0.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.854f, out.odometerM);
}

static void test_speed_calculator_slip() {
  SpeedCalculator sc;
  sc.setSlipThreshold(0.3f);
  const VehicleModel m = makeModel();
  WheelInput lw, rw;
  lw.direction = rw.direction = 1;
  lw.freqHz = 24.0f;
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
  c.rampRatePctPerS = 10000.0f;
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
  for (int i = 0; i < 50; ++i)
    out = d.update(m, 2.0f, 25.0f, 80.0f, 0.01f);
  TEST_ASSERT_TRUE(out.rightSpeed < 2.0f);
  TEST_ASSERT_TRUE(out.leftSpeed >= 2.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 2.0f * 0.4f, out.rightSpeed);
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
  RUN_TEST(test_telemetry_payload_size);
  RUN_TEST(test_encode_decode_roundtrip);
  RUN_TEST(test_parser_rejects_bad_crc);
  RUN_TEST(test_parser_rejects_wrong_version);
  RUN_TEST(test_parser_resync_after_garbage);
  RUN_TEST(test_command_encode);
  RUN_TEST(test_period_to_frequency);
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
