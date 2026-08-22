#include "services/commissioning.h"

#include <string.h>

#include "config/config_registry.h"
#include "control/steering_controller.h"
#include "core/version.h"
#include "drivers/i2c/i2c_bus.h"
#include "drivers/imu/mpu6050.h"
#include "drivers/ina3221/ina3221.h"
#include "drivers/uart/nano_link.h"
#include "drivers/wifi/wifi_manager.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/telemetry.h"

namespace vcm {

CommissioningReport commissioning;

SubsystemStatus i2cExpectedStatus(SensorHealth h, bool probing) {
  switch (h) {
    case SensorHealth::OK:
      return SubsystemStatus::OK;
    case SensorHealth::FAULT:
      return SubsystemStatus::FAULT;
    case SensorHealth::STALE:
      return SubsystemStatus::OFFLINE;
    case SensorHealth::NOT_PRESENT:
    case SensorHealth::INVALID:
    default:
      return probing ? SubsystemStatus::WAITING : SubsystemStatus::OFFLINE;
  }
}

SubsystemStatus optionalStatus(SensorHealth h, bool installed) {
  if (!installed) return SubsystemStatus::NOT_INSTALLED;
  switch (h) {
    case SensorHealth::OK:
      return SubsystemStatus::OK;
    case SensorHealth::FAULT:
      return SubsystemStatus::FAULT;
    default:
      return SubsystemStatus::OFFLINE;
  }
}

void refreshCommissioning(CommissioningReport& out) {
  const VehicleTelemetry t = telemetry.snapshot();
  strncpy(out.firmware, VCM_FW_VERSION, sizeof(out.firmware) - 1);
  strncpy(out.overall, vehicleStateName(safety.state()), sizeof(out.overall) - 1);

  out.esp = SubsystemStatus::OK;
  out.freeHeap = ESP.getFreeHeap();
  out.psramFree = ESP.getFreePsram();
  out.heap = (out.freeHeap > 40 * 1024) ? SubsystemStatus::OK
                                        : SubsystemStatus::FAULT;
  out.psramStatus =
      ESP.getPsramSize() > 0 ? SubsystemStatus::OK : SubsystemStatus::OFFLINE;

  const VehicleState st = safety.state();
  if (st == VehicleState::FAULT || st == VehicleState::ESTOP)
    out.safety = SubsystemStatus::FAULT;
  else if (st == VehicleState::INITIALIZING || st == VehicleState::BOOT)
    out.safety = SubsystemStatus::WAITING;
  else
    out.safety = SubsystemStatus::OK;

  const bool nanoOn = nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT));
  out.nano = nanoOn ? SubsystemStatus::OK : SubsystemStatus::OFFLINE;
  if (t.nano.rcValid)
    out.rc = SubsystemStatus::OK;
  else if (nanoOn)
    out.rc = SubsystemStatus::OFFLINE;
  else
    out.rc = SubsystemStatus::WAITING;

  out.steerFb = i2cExpectedStatus(steering.feedbackSensor().health(), false);
  if (steering.feedbackSensor().health() == SensorHealth::OK)
    out.steerFb = SubsystemStatus::OK;
  else if (steering.feedbackSensor().health() == SensorHealth::FAULT)
    out.steerFb = SubsystemStatus::FAULT;
  else
    out.steerFb = SubsystemStatus::OFFLINE;

  out.steerAct = SubsystemStatus::OK;
  out.leftMot = SubsystemStatus::OK;
  out.rightMot = SubsystemStatus::OK;

  const bool inaProbing = i2cBus.inaFsm().state() == I2cDevState::UNKNOWN ||
                          i2cBus.inaFsm().state() == I2cDevState::PROBING;
  const bool imuProbing = i2cBus.imuFsm().state() == I2cDevState::UNKNOWN ||
                          i2cBus.imuFsm().state() == I2cDevState::PROBING;
  out.ina = i2cExpectedStatus(ina.health(), inaProbing);
  out.imu = i2cExpectedStatus(imu.health(), imuProbing);

  const bool speedInstalled = config.b(HW_SPEED_SENSORS);
  const bool wheelInstalled = config.b(HW_MANUAL_STEER);
  const bool pedalInstalled = config.b(HW_PEDAL);
  if (speedInstalled) {
    out.leftSpeed = (t.nano.leftFreqHz > 0.1f || t.nano.leftPulseCount != 0)
                        ? SubsystemStatus::OK
                        : SubsystemStatus::OFFLINE;
    out.rightSpeed = (t.nano.rightFreqHz > 0.1f || t.nano.rightPulseCount != 0)
                         ? SubsystemStatus::OK
                         : SubsystemStatus::OFFLINE;
  } else {
    out.leftSpeed = SubsystemStatus::NOT_INSTALLED;
    out.rightSpeed = SubsystemStatus::NOT_INSTALLED;
  }
  out.wheel = optionalStatus(steering.wheelSensor().health(), wheelInstalled);
  out.pedal = optionalStatus(SensorHealth::NOT_PRESENT, pedalInstalled);

  out.wifi = wifiManager.apActive() ? SubsystemStatus::OK : SubsystemStatus::OFFLINE;
  out.web = wifiManager.apActive() ? SubsystemStatus::OK : SubsystemStatus::WAITING;
  const String ip = wifiManager.ipAddress();
  strncpy(out.ip, ip.c_str(), sizeof(out.ip) - 1);
}

static void line(const char* name, SubsystemStatus s) {
  LOGI("VCM", "  %-20s %s", name, subsystemStatusName(s));
}

void logCommissioningTable(const CommissioningReport& r) {
  LOGI("VCM", "======== DODGE PATROL VCM ========");
  LOGI("VCM", "Firmware: %s", r.firmware);
  LOGI("VCM", "CORE");
  line("ESP32-S3", r.esp);
  LOGI("VCM", "  %-20s %u KB", "Free heap", (unsigned)(r.freeHeap / 1024));
  line("PSRAM", r.psramStatus);
  line("Safety", r.safety);
  LOGI("VCM", "CONTROL");
  line("RC receiver", r.rc);
  line("Nano link", r.nano);
  line("Steering feedback", r.steerFb);
  line("Steering actuator", r.steerAct);
  line("Left motor", r.leftMot);
  line("Right motor", r.rightMot);
  LOGI("VCM", "SENSORS");
  line("INA3221", r.ina);
  line("MPU6050", r.imu);
  line("Left speed", r.leftSpeed);
  line("Right speed", r.rightSpeed);
  line("Steering wheel", r.wheel);
  line("Pedal", r.pedal);
  LOGI("VCM", "NETWORK");
  line("Wi-Fi AP", r.wifi);
  LOGI("VCM", "  %-20s %s", "IP", r.ip[0] ? r.ip : "--");
  line("Web server", r.web);
  LOGI("VCM", "STATUS");
  LOGI("VCM", "  %s", r.overall);
  LOGI("VCM", "==================================");
}

}  // namespace vcm
