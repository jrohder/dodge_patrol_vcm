#include "control/vehicle_dynamics.h"

#include "config/config_registry.h"
#include "control/control_arbiter.h"
#include "control/steering_controller.h"
#include "drivers/steering/p3022.h"
#include "drivers/uart/nano_link.h"
#include "services/calibration.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/telemetry.h"

namespace vcm {

VehicleDynamics dynamics;

void VehicleDynamics::begin() {
  refreshConfigIfChanged();
  LOGI("DYN", "Vehicle dynamics ready (100 Hz)");
}

void VehicleDynamics::refreshConfigIfChanged() {
  if (cfgRevision_ == config.revision()) return;
  cfgRevision_ = config.revision();

  model_.wheelDiameterM = config.f(VEH_WHEEL_DIAMETER);
  model_.wheelbaseM = config.f(VEH_WHEELBASE);
  model_.trackWidthM = config.f(VEH_TRACK_WIDTH);
  model_.massKg = config.f(VEH_MASS);
  model_.cogHeightM = config.f(VEH_COG_HEIGHT);
  model_.gearRatio = config.f(DRV_GEAR_RATIO);
  model_.countsPerRev = config.f(DRV_COUNTS_PER_REV);
  model_.maxSteeringAngleDeg = config.f(STR_MAX_ANGLE);

  speedCalc_.setCalibration(config.f(DRV_LEFT_CAL), config.f(DRV_RIGHT_CAL));
  speedCalc_.setSlipThreshold(config.f(DRV_SLIP_THRESHOLD));

  DiffConfig dc;
  dc.enabled = config.b(DIF_ENABLED);
  dc.algorithm = (DiffAlgorithm)config.i(DIF_ALGORITHM);
  dc.activationAngleDeg = config.f(DIF_ACT_ANGLE);
  dc.fullEffectAngleDeg = config.f(DIF_FULL_ANGLE);
  dc.maxDifferentialPct = config.f(DIF_MAX_PCT);
  dc.insideReductionPct = config.f(DIF_INSIDE_RED);
  dc.outsideBoostPct = config.f(DIF_OUTSIDE_BOOST);
  dc.allowInsideBrake = config.b(DIF_INSIDE_BRAKE);
  dc.insideBrakeThresholdPct = config.f(DIF_BRAKE_THRESH);
  dc.minSpeedMps = config.f(DIF_MIN_SPEED);
  dc.maxSpeedMps = config.f(DIF_MAX_SPEED);
  dc.reverseEnabled = config.b(DIF_REVERSE);
  dc.rampRatePctPerS = config.f(DIF_RAMP);
  dc.aggressiveAssist = config.b(DIF_AGGR_ENABLE);
  dc.aggressiveStartPct = config.f(DIF_AGGR_START);
  dc.aggressiveGain = config.f(DIF_AGGR_GAIN);
  diff_.setConfig(dc);

  logger.setLevel((LogLevel)config.i(LOG_LEVEL));
  steering.refreshConfig();
}

float VehicleDynamics::mapRcChannel(uint16_t us, bool invert) const {
  if (us < 800 || us > 2300) return 0.0f;  // invalid pulse
  const int center = config.i(RC_CENTER_US);
  const int deadband = config.i(RC_DEADBAND_US);
  const int minUs = config.i(RC_MIN_US), maxUs = config.i(RC_MAX_US);
  float v = 0.0f;
  if ((int)us > center + deadband) {
    v = (float)((int)us - center - deadband) / (maxUs - center - deadband);
  } else if ((int)us < center - deadband) {
    v = (float)((int)us - center + deadband) / (center - deadband - minUs);
  }
  v = constrain(v, -1.0f, 1.0f);
  return invert ? -v : v;
}

float VehicleDynamics::shapeThrottle(float raw) const {
  const float db = config.f(DRV_THROTTLE_DEADBAND) / 100.0f;
  if (fabsf(raw) < db) return 0.0f;
  float v = (raw > 0 ? raw - db : raw + db) / (1.0f - db);
  // Expo curve: blend linear with cubic for finer center control
  const float expo = config.f(DRV_THROTTLE_EXPO);
  v = (1.0f - expo) * v + expo * v * v * v;
  return v;
}

void VehicleDynamics::monitorSafety(bool rcValid, float battV) {
  // Nano link watchdog
  if (nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT))) {
    safety.clearFault(FLT_NANO_TIMEOUT);
  } else {
    safety.raiseFault(FLT_NANO_TIMEOUT);
  }
  if (nano.protocolMismatch()) safety.raiseFault(FLT_NANO_PROTOCOL);

  // COM-002: burst of CRC errors (wiring/EMI), not the lifetime total.
  static uint32_t lastCrc = 0;
  static uint32_t lastCrcMs = 0;
  const uint32_t nowMs = millis();
  const uint32_t crc = nano.crcErrors();
  if (lastCrcMs == 0) {
    lastCrc = crc;
    lastCrcMs = nowMs;
  } else if (nowMs - lastCrcMs >= 1000) {
    const uint32_t delta = crc - lastCrc;
    lastCrc = crc;
    lastCrcMs = nowMs;
    if (delta >= 20)
      safety.raiseFault(FLT_NANO_CRC);
    else if (delta == 0)
      safety.clearFault(FLT_NANO_CRC);
  }

  // RC loss only matters while RC owns the vehicle
  if (arbiter.activeSource() == ControlSource::RC_REMOTE && !rcValid) {
    safety.raiseFault(FLT_RC_LOST);
  } else if (rcValid) {
    safety.clearFault(FLT_RC_LOST);
  }

  // Battery monitoring (from INA3221 bus voltage; PWR-001/PWR-002)
  if (battV > 4.0f) {  // plausible measurement present
    if (battV < config.f(SAF_BATT_CUTOFF)) {
      safety.raiseFault(FLT_BATTERY_CUTOFF);
    } else if (battV > config.f(SAF_BATT_CUTOFF) + 0.5f) {
      safety.clearFault(FLT_BATTERY_CUTOFF);  // hysteresis
    }
    if (battV < config.f(SAF_BATT_WARN)) {
      safety.raiseFault(FLT_BATTERY_LOW);
    } else if (battV > config.f(SAF_BATT_WARN) + 0.3f) {
      safety.clearFault(FLT_BATTERY_LOW);
    }
  }
}

void VehicleDynamics::step() {
  refreshConfigIfChanged();

  const uint32_t now = millis();
  float dt = (lastStepMs_ == 0) ? 0.01f : (now - lastStepMs_) * 1e-3f;
  lastStepMs_ = now;
  dt = constrain(dt, 0.001f, 0.1f);

  // --- inputs from the Nano (shared veio::proto telemetry) -------------------
  const veio::proto::TelemetryPayload& pkt = nano.latest();
  const bool nanoOnline = nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT));

  const int chS = constrain(config.i(RC_CH_STEER), 1, 6) - 1;
  const int chT = constrain(config.i(RC_CH_THROTTLE), 1, 6) - 1;
  const bool rcSignal =
      nanoOnline && !(pkt.faultFlags & veio::proto::kFaultRcSignalLost) &&
      (pkt.rcValidMask & (1u << chT)) && pkt.rcPulseUs[chT] > 800;
  const float rcThrottle =
      rcSignal ? mapRcChannel(pkt.rcPulseUs[chT], config.b(RC_INV_THROTTLE))
               : 0.0f;
  const float rcSteering =
      rcSignal ? mapRcChannel(pkt.rcPulseUs[chS], config.b(RC_INV_STEER))
               : 0.0f;

  // Manual inputs: P3022 steering wheel + pedal via motor-wire sense ADCs
  const int senseThresh = config.i(CTL_SENSE_THRESH);
  const bool pedalFwd = nanoOnline && pkt.motorSenseAFilt > senseThresh;
  const bool pedalRev = nanoOnline && pkt.motorSenseBFilt > senseThresh;
  const bool manualActive = pedalFwd || pedalRev;
  const float manualLevel = config.f(CTL_MANUAL_THROTTLE) / 100.0f;
  const float manualThrottle =
      pedalFwd ? manualLevel : (pedalRev ? -manualLevel : 0.0f);
  const float manualSteering = P3022::normalize(
      steering.wheelSensor().lastCounts(), config.i(SW_MIN),
      config.i(SW_CENTER), config.i(SW_MAX), config.f(SW_DEADBAND),
      config.b(SW_INVERT));

  // --- arbitration ------------------------------------------------------------
  ControlCommand cmd = arbiter.arbitrate(rcThrottle, rcSteering, rcSignal,
                                         manualThrottle, manualSteering,
                                         manualActive);

  // --- speed measurement --------------------------------------------------------
  // Prefer Nano-reported freqHzX10; fall back to 1e6/periodUs when needed.
  WheelInput lw, rw;
  lw.freqHz = pkt.left.freqHzX10 > 0
                  ? pkt.left.freqHzX10 / 10.0f
                  : (pkt.left.periodUs > 0 ? 1e6f / (float)pkt.left.periodUs
                                          : 0.0f);
  lw.direction = pkt.left.direction;
  lw.count = (uint32_t)(pkt.left.pulseCount < 0 ? -pkt.left.pulseCount
                                                : pkt.left.pulseCount);
  rw.freqHz = pkt.right.freqHzX10 > 0
                  ? pkt.right.freqHzX10 / 10.0f
                  : (pkt.right.periodUs > 0 ? 1e6f / (float)pkt.right.periodUs
                                           : 0.0f);
  rw.direction = pkt.right.direction;
  rw.count = (uint32_t)(pkt.right.pulseCount < 0 ? -pkt.right.pulseCount
                                                 : pkt.right.pulseCount);
  const SpeedOutput spd = speedCalc_.update(
      model_, lw, rw, steering.estimatedAngleDeg(), dt);

  // --- safety -------------------------------------------------------------------
  float battV = 0.0f;
  telemetry.update([&](VehicleTelemetry& t) { battV = t.power.busVoltage; });
  monitorSafety(rcSignal, battV);

  const bool commandActive = cmd.valid && (fabsf(cmd.throttle) > 0.02f ||
                                           fabsf(spd.vehicleSpeed) > 0.05f);
  safety.tick(commandActive, calibration.commissioned());

  // --- throttle -> speed target ----------------------------------------------
  const float shaped = shapeThrottle(cmd.valid ? cmd.throttle : 0.0f);
  const float maxFwd = config.f(DRV_MAX_SPEED);
  const float maxRev = config.f(DRV_MAX_REVERSE);
  float requestedSpeed = shaped >= 0 ? shaped * maxFwd : shaped * maxRev;
  if (cmd.brake || !cmd.valid || !safety.motionAllowed()) requestedSpeed = 0.0f;
  if (safety.faultActive(FLT_BATTERY_CUTOFF)) requestedSpeed = 0.0f;

  // Acceleration / deceleration limiting (slew on the speed target)
  const bool accelerating = fabsf(requestedSpeed) > fabsf(limitedSpeed_);
  const float rate =
      accelerating ? config.f(DRV_MAX_ACCEL) : config.f(DRV_MAX_DECEL);
  const float maxDelta = rate * dt;
  limitedSpeed_ +=
      constrain(requestedSpeed - limitedSpeed_, -maxDelta, maxDelta);
  if (fabsf(limitedSpeed_) < 0.005f) limitedSpeed_ = 0.0f;

  // --- differential steering split ---------------------------------------------
  const float steerAngle = steering.estimatedAngleDeg();
  const float steerDemandPct = fabsf(cmd.steering) * 100.0f;
  const DiffOutput split =
      diff_.update(model_, limitedSpeed_, steerAngle, steerDemandPct, dt);

  leftTarget_ = split.leftSpeed;
  rightTarget_ = split.rightSpeed;
  steeringCmd_ = cmd.valid ? cmd.steering : 0.0f;

  // --- telemetry ------------------------------------------------------------------
  telemetry.update([&](VehicleTelemetry& t) {
    t.drive.throttleInput = cmd.valid ? cmd.throttle : 0.0f;
    t.drive.requestedSpeed = requestedSpeed;
    t.drive.limitedSpeed = limitedSpeed_;
    t.drive.leftTargetSpeed = split.leftSpeed;
    t.drive.rightTargetSpeed = split.rightSpeed;
    t.drive.leftActualSpeed = spd.leftSpeed;
    t.drive.rightActualSpeed = spd.rightSpeed;
    t.drive.vehicleSpeed = spd.vehicleSpeed;
    t.drive.leftRpm = spd.leftRpm;
    t.drive.rightRpm = spd.rightRpm;
    t.drive.odometerM = spd.odometerM;
    t.drive.tripM = spd.tripM;
    t.drive.differentialBias = split.bias;
    t.drive.slipDetected = spd.slipDetected;
    t.steering.wheelInputPct = manualSteering * 100.0f;

    t.nano.rcValid = rcSignal;
    t.nano.rcValidMask = pkt.rcValidMask;
    t.nano.rcThrottle = rcThrottle;
    t.nano.rcSteering = rcSteering;
    memcpy((void*)t.nano.rcUs, pkt.rcPulseUs, sizeof(t.nano.rcUs));
    memcpy((void*)t.nano.rcAgeMs, pkt.rcAgeMs, sizeof(t.nano.rcAgeMs));
    t.nano.leftPulseCount = pkt.left.pulseCount;
    t.nano.rightPulseCount = pkt.right.pulseCount;
    t.nano.leftPeriodUs = pkt.left.periodUs;
    t.nano.rightPeriodUs = pkt.right.periodUs;
    t.nano.leftFreqHz = lw.freqHz;
    t.nano.rightFreqHz = rw.freqHz;
    t.nano.leftDir = pkt.left.direction;
    t.nano.rightDir = pkt.right.direction;
    t.nano.leftWheelFault = pkt.left.fault;
    t.nano.rightWheelFault = pkt.right.fault;
    t.nano.leftCount = lw.count;
    t.nano.rightCount = rw.count;
    t.nano.motorSenseARaw = pkt.motorSenseARaw;
    t.nano.motorSenseAFilt = pkt.motorSenseAFilt;
    t.nano.motorSenseBRaw = pkt.motorSenseBRaw;
    t.nano.motorSenseBFilt = pkt.motorSenseBFilt;
    t.nano.batteryRaw = pkt.batteryRaw;
    t.nano.batteryFilt = pkt.batteryFilt;
    t.nano.batteryMin = pkt.batteryMin;
    t.nano.batteryMax = pkt.batteryMax;
    t.nano.adc[0] = pkt.batteryFilt;
    t.nano.adc[1] = pkt.motorSenseAFilt;
    t.nano.adc[2] = pkt.motorSenseBFilt;
    t.nano.adc[3] = pkt.batteryRaw;
    t.nano.adc[4] = pkt.motorSenseARaw;
    t.nano.adc[5] = pkt.motorSenseBRaw;
    t.nano.outputState = pkt.outputState;
    t.nano.systemState = pkt.systemState;
    t.nano.faultBits = pkt.faultFlags;
    t.nano.protocolVersion = veio::proto::kProtocolVersion;

    t.nano.packetsReceived = nano.packetsReceived();
    t.nano.telemetryReceived = nano.telemetryReceived();
    t.nano.packetsLost = nano.packetsLost();
    t.nano.crcErrors = nano.crcErrors();
    t.nano.frameErrors = nano.frameErrors();
    t.nano.seqErrors = nano.seqErrors();
    t.nano.acksReceived = nano.acksReceived();
    t.nano.faultsReceived = nano.faultsReceived();
    t.nano.lastPacketMs = nano.lastPacketMs();
    t.nano.lastHeartbeatMs = nano.lastHeartbeatMs();
    t.nano.timestampUs = nano.lastTimestampUs();
    t.nano.packetRateHz = nano.packetRateHz();
    t.nano.jitterUs = nano.jitterUs();
    t.nano.online = nanoOnline;
    t.nano.haveVersion = nano.haveVersion();
    t.nano.haveDiagnostic = nano.haveDiagnostic();

    if (nano.haveVersion()) {
      const auto& v = nano.version();
      t.nano.nanoFwMajor = v.fwMajor;
      t.nano.nanoFwMinor = v.fwMinor;
      t.nano.nanoFwPatch = v.fwPatch;
      t.nano.bootReason = v.bootReason;
      t.nano.watchdogResets = (v.bootReason == 2) ? 1 : 0;
    }
    if (nano.haveDiagnostic()) {
      const auto& d = nano.diagnostic();
      t.nano.nanoLoopAvgUs = d.loopTimeAvgUs;
      t.nano.nanoLoopMaxUs = d.loopTimeMaxUs;
      t.nano.nanoCpuPct = d.cpuLoadPct;
      t.nano.nanoSubsystems = d.subsystems;
      t.nano.nanoSchedulerOverruns = d.schedulerOverruns;
      t.nano.nanoSamplerOverruns = d.samplerOverruns;
      t.nano.nanoRxCrcErrors = d.uartCrcErrors;
      t.nano.nanoFrameErrors = d.uartFrameErrors;
      t.nano.nanoTxDrops = d.uartTxDrops;
      t.nano.nanoCommandsReceived = d.commandsReceived;
      t.nano.nanoRcGlitches = d.rcGlitches;
      t.nano.nanoFreeRam = d.freeRamBytes;
    }
    t.nano.nanoUptimeMs = nano.heartbeat().uptimeMs;

    t.system.state = safety.state();
    t.system.controlSource = arbiter.activeSource();
    t.system.activeFaultMask = safety.activeFaultMask();
    t.system.commissioned = calibration.commissioned();
  });
}

}  // namespace vcm
