/**
 * @file types.h
 * @brief Shared value types: control sources, commands, states, sensor health.
 */
#pragma once

#include <cstdint>

/// Where a driving command originates. Exactly one source owns the vehicle.
enum class ControlSource : uint8_t {
  NONE = 0,        ///< No active control source; outputs safe/off
  RC_REMOTE,       ///< HOTRC transmitter (parent), decoded by the Nano
  MANUAL_VEHICLE,  ///< Original steering wheel (P3022) + pedal/shifter
  WEB_REMOTE,      ///< Virtual remote on the web dashboard (WebSocket)
  CALIBRATION,     ///< Calibration wizards own the actuators
  DIAGNOSTIC,      ///< Motor/steering test pages (commissioning)
};

const char* controlSourceName(ControlSource s);

/// Standardized command produced by every control source.
/// The vehicle controller does not care where a command came from.
struct ControlCommand {
  float throttle = 0.0f;  ///< -1..+1 (negative = reverse)
  float steering = 0.0f;  ///< -1..+1 (negative = left)
  bool brake = false;     ///< Active braking request
  bool lights = false;
  bool siren = false;
  ControlSource source = ControlSource::NONE;
  uint32_t timestampMs = 0;  ///< millis() when generated
  bool valid = false;
};

/// Safety state machine states. Every transition is logged.
enum class VehicleState : uint8_t {
  BOOT = 0,
  INITIALIZING,
  NOT_CALIBRATED,  ///< Commissioning incomplete; driving locked out
  READY,
  DRIVING,
  CALIBRATION,
  DIAGNOSTIC,
  OTA_UPDATE,
  FAULT,
  ESTOP,
};

const char* vehicleStateName(VehicleState s);

/// Per-sensor health status. Values are never blindly trusted.
enum class SensorHealth : uint8_t {
  NOT_PRESENT = 0,
  OK,
  STALE,
  INVALID,
  FAULT,
};

const char* sensorHealthName(SensorHealth h);

/// Commissioning / optional-hardware line status. Missing sensors are not
/// treated as catastrophic faults.
enum class SubsystemStatus : uint8_t {
  OK = 0,
  OFFLINE,         ///< expected to be present, not communicating
  NOT_INSTALLED,   ///< not wired yet (speed sensors, manual wheel, pedal)
  FAULT,
  WAITING,         ///< still probing
};

const char* subsystemStatusName(SubsystemStatus s);

/// Fault identifiers. Codes and descriptions live in safety.cpp.
enum FaultId : uint8_t {
  FLT_NANO_TIMEOUT = 0,   // COM-001
  FLT_NANO_CRC,           // COM-002
  FLT_NANO_PROTOCOL,      // COM-003
  FLT_RC_LOST,            // RC-001
  FLT_WEB_HEARTBEAT,      // WEB-001
  FLT_STEER_FEEDBACK,     // STR-001
  FLT_STEER_OVERCURRENT,  // STR-002
  FLT_STEER_NOT_CAL,      // STR-003
  FLT_LEFT_OVERCURRENT,   // DRV-001
  FLT_RIGHT_OVERCURRENT,  // DRV-002
  FLT_BATTERY_CUTOFF,     // PWR-001
  FLT_BATTERY_LOW,        // PWR-002
  FLT_IMU_FAIL,           // SNS-001
  FLT_INA_FAIL,           // SNS-002
  FLT_TASK_OVERRUN,       // SYS-001
  FLT_LOW_HEAP,           // SYS-002
  FLT_COUNT,
};

/// What the system does when a fault becomes active.
enum class FaultSeverity : uint8_t {
  WARNING,  ///< Logged and shown; vehicle keeps operating
  TRIP,     ///< Enters FAULT state; propulsion commanded safe
};
