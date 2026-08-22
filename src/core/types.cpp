#include "core/types.h"

const char* controlSourceName(ControlSource s) {
  switch (s) {
    case ControlSource::NONE: return "NONE";
    case ControlSource::RC_REMOTE: return "RC REMOTE";
    case ControlSource::MANUAL_VEHICLE: return "MANUAL";
    case ControlSource::WEB_REMOTE: return "WEB REMOTE";
    case ControlSource::CALIBRATION: return "CALIBRATION";
    case ControlSource::DIAGNOSTIC: return "DIAGNOSTIC";
  }
  return "?";
}

const char* vehicleStateName(VehicleState s) {
  switch (s) {
    case VehicleState::BOOT: return "BOOT";
    case VehicleState::INITIALIZING: return "INITIALIZING";
    case VehicleState::NOT_CALIBRATED: return "NOT CALIBRATED";
    case VehicleState::READY: return "READY";
    case VehicleState::DRIVING: return "DRIVING";
    case VehicleState::CALIBRATION: return "CALIBRATION";
    case VehicleState::DIAGNOSTIC: return "DIAGNOSTIC";
    case VehicleState::OTA_UPDATE: return "OTA UPDATE";
    case VehicleState::FAULT: return "FAULT";
    case VehicleState::ESTOP: return "ESTOP";
  }
  return "?";
}

const char* sensorHealthName(SensorHealth h) {
  switch (h) {
    case SensorHealth::NOT_PRESENT: return "NOT PRESENT";
    case SensorHealth::OK: return "OK";
    case SensorHealth::STALE: return "STALE";
    case SensorHealth::INVALID: return "INVALID";
    case SensorHealth::FAULT: return "FAULT";
  }
  return "?";
}

const char* subsystemStatusName(SubsystemStatus s) {
  switch (s) {
    case SubsystemStatus::OK: return "OK";
    case SubsystemStatus::OFFLINE: return "OFFLINE";
    case SubsystemStatus::NOT_INSTALLED: return "NOT INSTALLED";
    case SubsystemStatus::FAULT: return "FAULT";
    case SubsystemStatus::WAITING: return "WAITING";
  }
  return "?";
}
