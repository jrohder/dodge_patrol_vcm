/**
 * @file commissioning.h
 * @brief Vehicle commissioning status table.
 *
 * Optional hardware (INA, IMU, wheel speed, manual wheel, pedal) is
 * reported as OFFLINE / NOT INSTALLED instead of blocking READY.
 */
#pragma once

#include "core/types.h"

namespace vcm {

struct CommissionLine {
  const char* name;
  SubsystemStatus status;
  const char* detail;
};

struct CommissioningReport {
  char firmware[24] = {};
  char overall[24] = {};

  SubsystemStatus esp = SubsystemStatus::OK;
  SubsystemStatus heap = SubsystemStatus::OK;
  SubsystemStatus psramStatus = SubsystemStatus::OK;
  SubsystemStatus safety = SubsystemStatus::WAITING;

  SubsystemStatus rc = SubsystemStatus::WAITING;
  SubsystemStatus nano = SubsystemStatus::WAITING;
  SubsystemStatus steerFb = SubsystemStatus::WAITING;
  SubsystemStatus steerAct = SubsystemStatus::OK;
  SubsystemStatus leftMot = SubsystemStatus::OK;
  SubsystemStatus rightMot = SubsystemStatus::OK;

  SubsystemStatus ina = SubsystemStatus::WAITING;
  SubsystemStatus imu = SubsystemStatus::WAITING;
  SubsystemStatus leftSpeed = SubsystemStatus::NOT_INSTALLED;
  SubsystemStatus rightSpeed = SubsystemStatus::NOT_INSTALLED;
  SubsystemStatus wheel = SubsystemStatus::NOT_INSTALLED;
  SubsystemStatus pedal = SubsystemStatus::NOT_INSTALLED;

  SubsystemStatus wifi = SubsystemStatus::WAITING;
  SubsystemStatus web = SubsystemStatus::WAITING;

  uint32_t freeHeap = 0;
  uint32_t psramFree = 0;
  char ip[16] = {};
};

/// Map an I²C device that is expected to be wired.
SubsystemStatus i2cExpectedStatus(SensorHealth h, bool probing);

/// Map optional hardware that is not installed yet.
SubsystemStatus optionalStatus(SensorHealth h, bool installed);

void refreshCommissioning(CommissioningReport& out);
void logCommissioningTable(const CommissioningReport& r);

extern CommissioningReport commissioning;

}  // namespace vcm
