/**
 * @file test_main.cpp
 * @brief Host-side unit tests: shared veio::proto UART protocol + control math.
 *
 * Protocol tests mirror vcm_extended_io/test/test_protocol so both ends of
 * the link exercise the same wire format.
 *
 * Run with: pio test -e native
 */
#include <cstring>
#include <unity.h>

#include "control/differential_steering.h"
#include "control/pid.h"
#include "control/speed_calculator.h"
#include "control/vehicle_model.h"
#include "proto/protocol.h"

using namespace vcm;
using namespace veio::proto;

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE
// ---------------------------------------------------------------------------

static void test_crc16_known_vector() {
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16(data, sizeof(data)));
}

static void test_crc16_empty() {
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16(nullptr, 0));
}

static void test_payload_sizes() {
  TEST_ASSERT_EQUAL_UINT(12, sizeof(WheelData));
  TEST_ASSERT_EQUAL_UINT(63, sizeof(TelemetryPayload));
  TEST_ASSERT_EQUAL_UINT(8, sizeof(HeartbeatPayload));
  TEST_ASSERT_EQUAL_UINT(23, sizeof(DiagnosticPayload));
  TEST_ASSERT_EQUAL_UINT(4, sizeof(CommandAckPayload));
  TEST_ASSERT_EQUAL_UINT(4, sizeof(FaultPayload));
  TEST_ASSERT_EQUAL_UINT(31, sizeof(VersionPayload));
  TEST_ASSERT_EQUAL_UINT(5, sizeof(CommandPayload));
  TEST_ASSERT_EQUAL_UINT(1, kProtocolVersion);
  TEST_ASSERT_EQUAL_HEX8(0x55, kSync0);
  TEST_ASSERT_EQUAL_HEX8(0xAA, kSync1);
}

static void test_encode_layout() {
  uint8_t buf[kMaxFrameSize];
  const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
  const size_t len =
      encodeFrame(buf, kPktTelemetry, 0x1234, 0xAABBCCDD, payload, 4);

  TEST_ASSERT_EQUAL_UINT(kHeaderSize + 4 + kCrcSize, len);
  TEST_ASSERT_EQUAL_HEX8(0x55, buf[0]);
  TEST_ASSERT_EQUAL_HEX8(0xAA, buf[1]);
  TEST_ASSERT_EQUAL_HEX8(kProtocolVersion, buf[2]);
  TEST_ASSERT_EQUAL_HEX8(kPktTelemetry, buf[3]);
  TEST_ASSERT_EQUAL_HEX8(4, buf[4]);
  TEST_ASSERT_EQUAL_HEX8(0x34, buf[5]);
  TEST_ASSERT_EQUAL_HEX8(0x12, buf[6]);
  TEST_ASSERT_EQUAL_HEX8(0xDD, buf[7]);
  TEST_ASSERT_EQUAL_HEX8(0xCC, buf[8]);
  TEST_ASSERT_EQUAL_HEX8(0xBB, buf[9]);
  TEST_ASSERT_EQUAL_HEX8(0xAA, buf[10]);
  TEST_ASSERT_EQUAL_HEX8(0xDE, buf[11]);
}

static void test_encode_rejects_oversize() {
  uint8_t buf[kMaxFrameSize];
  uint8_t payload[kMaxPayload + 1] = {};
  TEST_ASSERT_EQUAL_UINT(
      0, encodeFrame(buf, kPktTelemetry, 0, 0, payload, kMaxPayload + 1));
}

static void feedAll(FrameParser& p, const uint8_t* data, size_t len,
                    int* frames) {
  for (size_t i = 0; i < len; ++i) {
    if (p.feed(data[i])) ++(*frames);
  }
}

static void test_roundtrip_telemetry() {
  TelemetryPayload t = {};
  t.rcPulseUs[0] = 1500;
  t.rcPulseUs[1] = 1548;
  t.rcValidMask = 0x03;
  t.left.pulseCount = -42;
  t.left.direction = -1;
  t.left.freqHzX10 = 123;
  t.left.periodUs = 81300;
  t.right.pulseCount = 99999;
  t.right.direction = 1;
  t.batteryFilt = 9876;
  t.systemState = kStateReady;
  t.faultFlags = kFaultRcSignalLost;

  uint8_t buf[kMaxFrameSize];
  const size_t len =
      encodeFrame(buf, kPktTelemetry, 7, 123456789UL, &t, sizeof(t));
  TEST_ASSERT_TRUE(len > 0);

  FrameParser parser;
  int frames = 0;
  feedAll(parser, buf, len, &frames);
  TEST_ASSERT_EQUAL_INT(1, frames);

  const Frame& f = parser.frame();
  TEST_ASSERT_EQUAL_HEX8(kPktTelemetry, f.type);
  TEST_ASSERT_EQUAL_UINT16(7, f.sequence);
  TEST_ASSERT_EQUAL_UINT32(123456789UL, f.timestampUs);
  TEST_ASSERT_EQUAL_UINT(sizeof(t), f.payloadLen);

  TelemetryPayload out;
  memcpy(&out, f.payload, sizeof(out));
  TEST_ASSERT_EQUAL_UINT16(1500, out.rcPulseUs[0]);
  TEST_ASSERT_EQUAL_INT32(-42, out.left.pulseCount);
  TEST_ASSERT_EQUAL_UINT32(81300, out.left.periodUs);
  TEST_ASSERT_EQUAL_HEX16(kFaultRcSignalLost, out.faultFlags);
}

static void test_roundtrip_empty_payload() {
  uint8_t buf[kMaxFrameSize];
  const size_t len = encodeFrame(buf, kPktHeartbeat, 1, 2, nullptr, 0);
  TEST_ASSERT_EQUAL_UINT(kHeaderSize + kCrcSize, len);
  FrameParser parser;
  int frames = 0;
  feedAll(parser, buf, len, &frames);
  TEST_ASSERT_EQUAL_INT(1, frames);
  TEST_ASSERT_EQUAL_UINT(0, parser.frame().payloadLen);
}

static void test_parser_resync_after_garbage() {
  CommandPayload cmd = {};
  cmd.commandId = kCmdPing;
  uint8_t frameBuf[kMaxFrameSize];
  const size_t len =
      encodeFrame(frameBuf, kPktCommand, 5, 1000, &cmd, sizeof(cmd));

  uint8_t stream[64];
  const uint8_t garbage[] = {0x00, 0x55, 0x01, 0xAA, 0x55, 0x55, 0xFF};
  memcpy(stream, garbage, sizeof(garbage));
  memcpy(stream + sizeof(garbage), frameBuf, len);

  FrameParser parser;
  int frames = 0;
  feedAll(parser, stream, sizeof(garbage) + len, &frames);
  TEST_ASSERT_EQUAL_INT(1, frames);
  TEST_ASSERT_EQUAL_HEX8(kPktCommand, parser.frame().type);
}

static void test_parser_rejects_bad_crc() {
  CommandPayload cmd = {};
  cmd.commandId = kCmdSetOutput;
  uint8_t buf[kMaxFrameSize];
  const size_t len =
      encodeFrame(buf, kPktCommand, 9, 42, &cmd, sizeof(cmd));
  buf[len - 1] ^= 0xFF;

  FrameParser parser;
  int frames = 0;
  feedAll(parser, buf, len, &frames);
  TEST_ASSERT_EQUAL_INT(0, frames);
  TEST_ASSERT_EQUAL_UINT16(1, parser.crcErrors());
}

static void test_parser_rejects_bad_version() {
  uint8_t buf[kMaxFrameSize];
  const size_t len = encodeFrame(buf, kPktHeartbeat, 0, 0, nullptr, 0);
  uint8_t bad[kMaxFrameSize];
  memcpy(bad, buf, len);
  bad[2] = 0x7F;

  FrameParser parser;
  int frames = 0;
  feedAll(parser, bad, len, &frames);
  TEST_ASSERT_EQUAL_INT(0, frames);
  TEST_ASSERT_EQUAL_UINT16(1, parser.frameErrors());
}

static void test_parser_back_to_back_frames() {
  uint8_t buf[3 * kMaxFrameSize];
  size_t total = 0;
  for (uint16_t seq = 0; seq < 3; ++seq) {
    HeartbeatPayload hb = {};
    hb.uptimeMs = seq * 1000;
    total += encodeFrame(buf + total, kPktHeartbeat, seq, seq * 10, &hb,
                         sizeof(hb));
  }
  FrameParser parser;
  int frames = 0;
  feedAll(parser, buf, total, &frames);
  TEST_ASSERT_EQUAL_INT(3, frames);
  TEST_ASSERT_EQUAL_UINT16(2, parser.frame().sequence);
}

static void test_period_and_freq() {
  // Consumer may use either Nano freqHzX10 or derive from periodUs.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 12.3f, 123 / 10.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 20.0f, 1e6f / 50000.0f);
}

// ------------------------------------------------------------------ PID

static void test_pid_proportional() {
  Pid pid;
  Pid::Gains g;
  g.kp = 2.0f;
  g.outMax = 100.0f;
  pid.setGains(g);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 20.0f, pid.update(10.0f, 0.0f, 0.005f));
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
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, makeModel().freqToRpm(12.0f));
}

static void test_model_freq_to_speed() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.7854f, makeModel().freqToSpeed(12.0f));
}

static void test_model_counts_to_distance() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.854f, makeModel().countsToDistance(120.0f));
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
  m.ackermannRatios(30.0f, l, r);
  TEST_ASSERT_TRUE(r < 1.0f);
  TEST_ASSERT_TRUE(l > 1.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.7594f, r);
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
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 7.854f,
                           sc.update(m, lw, rw, 0.0f, 0.01f).odometerM);
}

static void test_speed_calculator_slip() {
  SpeedCalculator sc;
  sc.setSlipThreshold(0.3f);
  const VehicleModel m = makeModel();
  WheelInput lw, rw;
  lw.direction = rw.direction = 1;
  lw.freqHz = 24.0f;
  rw.freqHz = 6.0f;
  TEST_ASSERT_TRUE(sc.update(m, lw, rw, 0.0f, 0.01f).slipDetected);
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
  const DiffOutput out = d.update(makeModel(), 2.0f, 20.0f, 60.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.leftSpeed);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.rightSpeed);
}

static void test_diff_simple_right_turn() {
  DifferentialSteering d;
  d.setConfig(diffBase());
  DiffOutput out;
  for (int i = 0; i < 50; ++i)
    out = d.update(makeModel(), 2.0f, 25.0f, 80.0f, 0.01f);
  TEST_ASSERT_TRUE(out.rightSpeed < 2.0f);
  TEST_ASSERT_TRUE(out.leftSpeed >= 2.0f);
}

static void test_diff_below_activation_angle() {
  DifferentialSteering d;
  d.setConfig(diffBase());
  const DiffOutput out = d.update(makeModel(), 2.0f, 1.0f, 5.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f, out.leftSpeed);
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
  DiffOutput out;
  for (int i = 0; i < 50; ++i)
    out = d.update(makeModel(), -1.0f, 25.0f, 80.0f, 0.01f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -1.0f, out.leftSpeed);
}

static void test_diff_inside_brake() {
  DifferentialSteering d;
  DiffConfig c = diffBase();
  c.insideReductionPct = 100.0f;
  c.maxDifferentialPct = 100.0f;
  c.allowInsideBrake = true;
  c.insideBrakeThresholdPct = 20.0f;
  d.setConfig(c);
  DiffOutput out;
  for (int i = 0; i < 50; ++i)
    out = d.update(makeModel(), 2.0f, 30.0f, 100.0f, 0.01f);
  TEST_ASSERT_TRUE(out.insideBraking);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.rightSpeed);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_known_vector);
  RUN_TEST(test_crc16_empty);
  RUN_TEST(test_payload_sizes);
  RUN_TEST(test_encode_layout);
  RUN_TEST(test_encode_rejects_oversize);
  RUN_TEST(test_roundtrip_telemetry);
  RUN_TEST(test_roundtrip_empty_payload);
  RUN_TEST(test_parser_resync_after_garbage);
  RUN_TEST(test_parser_rejects_bad_crc);
  RUN_TEST(test_parser_rejects_bad_version);
  RUN_TEST(test_parser_back_to_back_frames);
  RUN_TEST(test_period_and_freq);
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
