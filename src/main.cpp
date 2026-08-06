/**
 * @file main.cpp
 * @brief Dodge Patrol VCM boot sequence and FreeRTOS task wiring.
 *
 * Task layout (core 1 = control, core 0 = comms/UI):
 *   steer_motor  200 Hz  core 1  steering PID + drive motor outputs
 *   dynamics     100 Hz  core 1  Nano link, arbitration, vehicle dynamics
 *   sensors      100 Hz  core 0  I2C (INA3221 + MPU6050)
 *   telemetry     20 Hz  core 0  WebSocket broadcast, recorder, LED
 *   system         1 Hz  core 0  diagnostics, WiFi, OTA auto-check, button
 *
 * Boot order guarantees motor outputs are forced OFF before anything else
 * runs (outputs default OFF; the ESP32 boots without unintended movement).
 */
#include <Arduino.h>
#include <Wire.h>

#include "config/config_registry.h"
#include "control/drive_controller.h"
#include "control/steering_controller.h"
#include "control/vehicle_dynamics.h"
#include "core/pins.h"
#include "core/version.h"
#include "drivers/imu/mpu6050.h"
#include "drivers/ina3221/ina3221.h"
#include "drivers/status_led.h"
#include "drivers/uart/nano_link.h"
#include "drivers/wifi/wifi_manager.h"
#include "services/calibration.h"
#include "services/diagnostics.h"
#include "services/event_recorder.h"
#include "services/logger.h"
#include "services/ota_service.h"
#include "services/safety.h"
#include "services/telemetry.h"
#include "web/web_server.h"

namespace vcm {

static Mpu6050 imu;
static Ina3221 ina;
static volatile bool imuZeroRequested = false;

void requestImuZero() { imuZeroRequested = true; }

static TaskMonitor steerMon(5000);    // 200 Hz -> 5 ms period
static TaskMonitor dynMon(10000);     // 100 Hz -> 10 ms period
static TaskMonitor sensorMon(10000);  // 100 Hz

// ------------------------------------------------------------------ tasks

/// 200 Hz: steering PID + drive motor outputs (highest priority)
static void steerMotorTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    steerMon.beginCycle();
    const VehicleTelemetry t = telemetry.snapshot();

    float leftT, rightT, steerCmd;
    dynamics.targets(leftT, rightT, steerCmd);

    const bool outputEnabled = !ota.busy() &&
                               safety.state() != VehicleState::ESTOP &&
                               safety.state() != VehicleState::FAULT;

    steering.step(steerCmd, t.power.steeringCurrentA, outputEnabled);
    drive.step(leftT, rightT, t.drive.leftActualSpeed,
               t.drive.rightActualSpeed, t.power.leftCurrentA,
               t.power.rightCurrentA, outputEnabled);

    telemetry.update([&](VehicleTelemetry& tt) {
      tt.drive.leftPwmPct = drive.leftPwm();
      tt.drive.rightPwmPct = drive.rightPwm();
      tt.system.steerAvgUs = steerMon.avgUs();
      tt.system.steerMaxUs = steerMon.maxUs();
      tt.system.steerMisses = steerMon.misses();
    });
    steerMon.endCycle();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
  }
}

/// 100 Hz: Nano link + control arbitration + vehicle dynamics
static void dynamicsTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint8_t hbDivider = 0;
  for (;;) {
    dynMon.beginCycle();
    nano.poll();
    if (++hbDivider >= 10) {  // 10 Hz heartbeat to the Nano
      hbDivider = 0;
      nano.sendHeartbeat();
    }
    dynamics.step();
    calibration.tickMotorTest();
    telemetry.update([&](VehicleTelemetry& t) {
      t.system.dynAvgUs = dynMon.avgUs();
      t.system.dynMaxUs = dynMon.maxUs();
      t.system.dynMisses = dynMon.misses();
    });
    dynMon.endCycle();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

/// 100 Hz: I2C sensor bus (INA3221 current monitor + MPU6050 IMU)
static void sensorTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    sensorMon.beginCycle();

    // Current monitor: role mapping is configuration, not hardwired
    ina.sample(config.f(CUR_SHUNT), config.f(CUR_OFFSET), config.f(CUR_SCALE));
    float roleA[3] = {0, 0, 0};  // 0=left 1=right 2=steering
    const int roles[3] = {config.i(CUR_CH1_ROLE), config.i(CUR_CH2_ROLE),
                          config.i(CUR_CH3_ROLE)};
    for (int ch = 0; ch < 3; ++ch) {
      roleA[constrain(roles[ch], 0, 2)] = fabsf(ina.currentA(ch));
    }

    // IMU with configurable mounting orientation
    Mpu6050::Orientation orient;
    orient.srcX = config.i(IMU_X_SRC);
    orient.srcY = config.i(IMU_Y_SRC);
    orient.srcZ = config.i(IMU_Z_SRC);
    orient.invX = config.b(IMU_INV_X);
    orient.invY = config.b(IMU_INV_Y);
    orient.invZ = config.b(IMU_INV_Z);
    imu.sample(orient, config.f(IMU_FILTER));
    if (imuZeroRequested) {
      imuZeroRequested = false;
      imu.zero();
    }

    if (ina.health() == SensorHealth::FAULT) safety.raiseFault(FLT_INA_FAIL);
    else if (ina.health() == SensorHealth::OK) safety.clearFault(FLT_INA_FAIL);
    if (imu.health() == SensorHealth::FAULT) safety.raiseFault(FLT_IMU_FAIL);
    else if (imu.health() == SensorHealth::OK) safety.clearFault(FLT_IMU_FAIL);

    telemetry.update([&](VehicleTelemetry& t) {
      t.power.busVoltage = ina.busVoltage();
      t.power.leftCurrentA = roleA[0];
      t.power.rightCurrentA = roleA[1];
      t.power.steeringCurrentA = roleA[2];
      const float maxA = max(roleA[0], max(roleA[1], roleA[2]));
      if (maxA > t.power.peakCurrentA) t.power.peakCurrentA = maxA;
      t.power.inaHealth = ina.health();

      memcpy(t.imu.rawAccel, imu.rawAccel(), sizeof(t.imu.rawAccel));
      memcpy(t.imu.rawGyro, imu.rawGyro(), sizeof(t.imu.rawGyro));
      memcpy(t.imu.accel, imu.accel(), sizeof(t.imu.accel));
      memcpy(t.imu.gyro, imu.gyro(), sizeof(t.imu.gyro));
      t.imu.pitchDeg = imu.pitchDeg();
      t.imu.rollDeg = imu.rollDeg();
      t.imu.yawRateDps = imu.yawRateDps();
      t.imu.tempC = imu.tempC();
      t.imu.health = imu.health();
      t.system.sensorAvgUs = sensorMon.avgUs();
      t.system.sensorMaxUs = sensorMon.maxUs();
    });
    sensorMon.endCycle();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
  }
}

/// 20 Hz: WebSocket telemetry, event recorder (10 Hz), status LED
static void telemetryTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint8_t divider = 0;
  for (;;) {
    const int rate = constrain(config.i(WEB_TELEM_RATE), 1, 30);
    static uint32_t lastSend = 0;
    if (millis() - lastSend >= (uint32_t)(1000 / rate)) {
      lastSend = millis();
      webServer.broadcastTelemetry();
    }
    if (++divider >= 2) {  // 10 Hz
      divider = 0;
      recorder.record(telemetry.snapshot());
      statusLed.update(safety.state(),
                       nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT)));
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
  }
}

/// 1 Hz: diagnostics, WiFi housekeeping, factory-reset button, OTA check
static void systemTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t buttonHeldSince = 0;
  uint32_t lastOtaCheck = 0;
  for (;;) {
    diagnostics.tick();
    wifiManager.tick();
    webServer.tick();

    // Boot button held 10 s at runtime = factory reset
    if (digitalRead(pins::BOOT_BUTTON) == LOW) {
      if (buttonHeldSince == 0) buttonHeldSince = millis();
      if (millis() - buttonHeldSince > 10000) {
        LOGW("SYS", "Boot button held 10 s - factory reset + reboot");
        config.factoryReset();
        delay(500);
        ESP.restart();
      }
    } else {
      buttonHeldSince = 0;
    }

    // Periodic GitHub release check (every 6 h, only when idle)
    if (config.b(OTA_AUTO_CHECK) && !ota.busy() &&
        (lastOtaCheck == 0 || millis() - lastOtaCheck > 6UL * 3600 * 1000) &&
        millis() > 60000) {
      lastOtaCheck = millis();
      ota.checkGitHub();
    }

    telemetry.update([&](VehicleTelemetry& t) {
      t.system.uptimeS = diagnostics.uptimeS();
      t.system.freeHeap = diagnostics.freeHeap();
      t.system.minFreeHeap = diagnostics.minFreeHeap();
      t.system.cpuLoadPct = diagnostics.cpuLoadPct();
      t.system.wifiRssi = wifiManager.rssi();
      t.system.wifiClients = wifiManager.clientCount();
    });
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
  }
}

// ------------------------------------------------------------------ boot

static void bootSelfTest() {
  LOGI("BOOT", "Self-test: NVS %s", config.revision() ? "OK" : "DEFAULTS");
  LOGI("BOOT", "Self-test: INA3221 %s", sensorHealthName(ina.health()));
  LOGI("BOOT", "Self-test: MPU6050 %s", sensorHealthName(imu.health()));
  LOGI("BOOT", "Self-test: P3022 %s",
       sensorHealthName(steering.wheelSensor().health()));
  LOGI("BOOT", "Self-test: Firgelli ADC %s",
       sensorHealthName(steering.feedbackSensor().health()));
  // Nano may still be booting; the link watchdog reports it live.
  LOGI("BOOT", "Self-test: Nano %s",
       nano.online(500) ? "OK" : "waiting for first packet");
}

static void bootstrap() {
  Serial.begin(115200);
  logger.begin();
  statusLed.begin();  // blue = booting
  LOGI("BOOT", "Dodge Patrol VCM %s (%s, built %s)", VCM_FW_VERSION,
       VCM_GIT_COMMIT, VCM_BUILD_DATE);

  pinMode(pins::BOOT_BUTTON, INPUT_PULLUP);

  // Order matters: config -> safety -> actuators (outputs OFF) -> comms
  config.begin();
  telemetry.begin();
  recorder.begin();
  safety.begin();
  calibration.begin();

  steering.begin();  // forces steering outputs off
  drive.begin();     // forces drive outputs off

  Wire.begin(pins::I2C_SDA, pins::I2C_SCL, 400000);
  ina.begin();
  imu.begin();
  nano.begin();
  dynamics.begin();

  wifiManager.begin();
  webServer.begin();
  ota.begin();

  bootSelfTest();

  // Leave INITIALIZING: commissioned vehicles go READY, otherwise the
  // vehicle stays locked out of normal driving (NOT COMMISSIONED).
  safety.requestState(calibration.commissioned()
                          ? VehicleState::READY
                          : VehicleState::NOT_CALIBRATED,
                      "boot complete");

  xTaskCreatePinnedToCore(steerMotorTask, "steer_motor", 6144, nullptr, 5,
                          nullptr, 1);
  xTaskCreatePinnedToCore(dynamicsTask, "dynamics", 8192, nullptr, 4, nullptr,
                          1);
  xTaskCreatePinnedToCore(sensorTask, "sensors", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 8192, nullptr, 2,
                          nullptr, 0);
  xTaskCreatePinnedToCore(systemTask, "system", 6144, nullptr, 1, nullptr, 0);
  LOGI("BOOT", "All tasks running");
}

}  // namespace vcm

void setup() { vcm::bootstrap(); }

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
