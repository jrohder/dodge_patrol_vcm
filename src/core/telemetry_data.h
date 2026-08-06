/**
 * @file telemetry_data.h
 * @brief The ONE canonical telemetry structure for the whole VCM.
 *
 * Every module writes into this structure (through TelemetryHub);
 * WebSocket, logger, dashboard, event recorder and diagnostics all read
 * from it. No module maintains a private copy of vehicle state.
 */
#pragma once

#include <cstdint>

#include "core/types.h"

/// Timestamped value with health, so stale data detection is trivial.
template <typename T>
struct Stamped {
  T value{};
  uint32_t updatedMs = 0;
  SensorHealth health = SensorHealth::NOT_PRESENT;

  void set(T v, uint32_t nowMs) {
    value = v;
    updatedMs = nowMs;
    health = SensorHealth::OK;
  }
  uint32_t age(uint32_t nowMs) const { return nowMs - updatedMs; }
};

struct SteeringTelemetry {
  float requestedPct = 50.0f;  ///< 0..100 actuator target position
  float actualPct = 50.0f;     ///< 0..100 filtered Firgelli feedback
  float requestedAngleDeg = 0.0f;
  float actualAngleDeg = 0.0f;
  float error = 0.0f;
  float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f;
  float output = 0.0f;  ///< -100..100 controller output before PWM
  float pwmPct = 0.0f;  ///< signed PWM actually applied
  uint16_t feedbackRaw = 0;
  float feedbackFiltered = 0.0f;
  uint16_t wheelInputRaw = 0;  ///< P3022 steering wheel encoder counts
  float wheelInputPct = 0.0f;  ///< -100..100 normalized wheel input
  float currentA = 0.0f;
  bool calibrated = false;
};

struct DriveTelemetry {
  float throttleInput = 0.0f;    ///< -1..1 from active control source
  float requestedSpeed = 0.0f;   ///< m/s before limits
  float limitedSpeed = 0.0f;     ///< m/s after accel/decel/speed limits
  float leftTargetSpeed = 0.0f;  ///< m/s after differential split
  float rightTargetSpeed = 0.0f;
  float leftActualSpeed = 0.0f;  ///< m/s from wheel sensors
  float rightActualSpeed = 0.0f;
  float leftPwmPct = 0.0f;  ///< signed -100..100
  float rightPwmPct = 0.0f;
  float vehicleSpeed = 0.0f;  ///< m/s derived
  float leftRpm = 0.0f, rightRpm = 0.0f;
  float odometerM = 0.0f, tripM = 0.0f;
  float differentialBias = 0.0f;  ///< -1..1 applied bias for visualization
  bool slipDetected = false;
};

struct NanoTelemetry {
  uint16_t rcUs[6] = {1500, 1500, 1500, 1500, 1500, 1500};
  float rcSteering = 0.0f;  ///< -1..1 mapped
  float rcThrottle = 0.0f;
  bool rcValid = false;
  float leftFreqHz = 0.0f, rightFreqHz = 0.0f;
  int8_t leftDir = 0, rightDir = 0;
  uint32_t leftCount = 0, rightCount = 0;
  uint16_t adc[4] = {0, 0, 0, 0};
  uint16_t faultFlags = 0;
  uint8_t protocolVersion = 0;
  uint8_t nanoFwMajor = 0, nanoFwMinor = 0;
  uint32_t packetsReceived = 0, packetsLost = 0;
  uint32_t crcErrors = 0, seqErrors = 0, timeouts = 0;
  uint32_t lastPacketMs = 0;
  float packetRateHz = 0.0f;
  bool online = false;
};

struct PowerTelemetry {
  float busVoltage = 0.0f;
  float leftCurrentA = 0.0f;
  float rightCurrentA = 0.0f;
  float steeringCurrentA = 0.0f;
  float peakCurrentA = 0.0f;
  SensorHealth inaHealth = SensorHealth::NOT_PRESENT;
};

struct ImuTelemetry {
  float rawAccel[3] = {0, 0, 0};  ///< g, sensor frame
  float rawGyro[3] = {0, 0, 0};   ///< deg/s, sensor frame
  float accel[3] = {0, 0, 0};     ///< g, vehicle frame (remap+invert+zero)
  float gyro[3] = {0, 0, 0};      ///< deg/s, vehicle frame
  float pitchDeg = 0.0f, rollDeg = 0.0f, yawRateDps = 0.0f;
  float tempC = 0.0f;
  SensorHealth health = SensorHealth::NOT_PRESENT;
};

struct SystemTelemetry {
  VehicleState state = VehicleState::BOOT;
  ControlSource controlSource = ControlSource::NONE;
  uint32_t uptimeS = 0;
  uint32_t freeHeap = 0, minFreeHeap = 0;
  float cpuLoadPct = 0.0f;
  int8_t wifiRssi = 0;
  uint8_t wifiClients = 0;
  bool commissioned = false;
  uint16_t activeFaultMask = 0;
  // Task timing (us): [avg, max] for dynamics(100Hz) and steering(200Hz)
  uint32_t dynAvgUs = 0, dynMaxUs = 0, dynMisses = 0;
  uint32_t steerAvgUs = 0, steerMaxUs = 0, steerMisses = 0;
  uint32_t sensorAvgUs = 0, sensorMaxUs = 0;
};

/// The canonical vehicle telemetry. Access ONLY through TelemetryHub.
struct VehicleTelemetry {
  SteeringTelemetry steering;
  DriveTelemetry drive;
  NanoTelemetry nano;
  PowerTelemetry power;
  ImuTelemetry imu;
  SystemTelemetry system;
};
