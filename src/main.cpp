/**
 * @file main.cpp
 * @brief Dodge Patrol VCM boot sequence and FreeRTOS task wiring.
 *
 * Task layout (core 1 = control, core 0 = comms/UI):
 *   steer_motor  200 Hz  core 1  steering PID + drive motor outputs
 *   dynamics     100 Hz  core 1  Nano link, arbitration, vehicle dynamics
 *   sensors      100 Hz  core 0  I2C (INA3221 + MPU6050), optional, never blocks control
 *   telemetry     20 Hz  core 0  WebSocket broadcast, recorder, LED
 *   system         1 Hz  core 0  diagnostics, WiFi, commissioning table, button
 *
 * Boot order (vehicle controller, not a bench sketch):
 *   GPIOs/motors OFF → safety → Nano/RC UART → OTA mutex → control tasks
 *   → Wi-Fi AP → web server → background I²C
 * RC never waits for Wi-Fi. Wi-Fi never waits for I²C.
 * Control tasks call ota.busy() every cycle, so the OTA mutex must exist
 * before they are created.
 */
#include <Arduino.h>
#include <cmath>

#include "config/config_registry.h"
#include "control/drive_controller.h"
#include "control/steering_controller.h"
#include "control/vehicle_dynamics.h"
#include "core/pins.h"
#include "core/version.h"
#include "drivers/i2c/i2c_bus.h"
#include "drivers/imu/mpu6050.h"
#include "drivers/ina3221/ina3221.h"
#include "drivers/status_led.h"
#include "drivers/uart/nano_link.h"
#include "drivers/wifi/wifi_manager.h"
#include "services/boot_report.h"
#include "services/calibration.h"
#include "services/commissioning.h"
#include "services/diagnostics.h"
#include "services/event_recorder.h"
#include "services/logger.h"
#include "services/ota_service.h"
#include "services/safety.h"
#include "services/steering_characterization.h"
#include "services/steering_recorder.h"
#include "services/telemetry.h"
#include "services/usb_console.h"
#include "web/web_server.h"

#include <nvs_flash.h>

namespace vcm {

static volatile bool imuZeroRequested = false;

void requestImuZero() { imuZeroRequested = true; }

static TaskMonitor steerMon(5000);    // 200 Hz -> 5 ms period
static TaskMonitor dynMon(10000);     // 100 Hz -> 10 ms period
static TaskMonitor sensorMon(10000);  // 100 Hz

static void forceMotorsOff() {
  const int pinsOff[] = {pins::STEER_RPWM, pins::STEER_LPWM, pins::LEFT_RPWM,
                         pins::LEFT_LPWM,  pins::RIGHT_RPWM, pins::RIGHT_LPWM};
  for (int p : pinsOff) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

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

    steering.step(steerCmd, t.power.steeringCurrentA, outputEnabled,
                  t.power.inaHealth, t.system.steerMisses);
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

static void probeOrSampleIna(uint32_t now) {
  I2cDeviceFsm& fsm = i2cBus.inaFsm();
  if (fsm.shouldProbe(now)) {
    fsm.onProbeStart(now);
    const bool ok = ina.begin();
    if (ok)
      fsm.onProbeOk(now);
    else
      fsm.onProbeFail(now);
    i2cBus.noteTxn("ina probe", ok, !ok);
    return;
  }
  if (!fsm.online()) return;
  const bool ok =
      ina.sample(config.f(CUR_SHUNT), config.f(CUR_OFFSET), config.f(CUR_SCALE));
  if (ok)
    fsm.onTxnOk(now);
  else
    fsm.onTxnFail(now);
  i2cBus.noteTxn("ina sample", ok, !ok);
}

static void probeOrSampleImu(uint32_t now) {
  I2cDeviceFsm& fsm = i2cBus.imuFsm();
  if (fsm.shouldProbe(now)) {
    fsm.onProbeStart(now);
    const bool ok = imu.begin();
    if (ok)
      fsm.onProbeOk(now);
    else
      fsm.onProbeFail(now);
    i2cBus.noteTxn("imu probe", ok, !ok);
    return;
  }
  if (!fsm.online()) return;
  Mpu6050::Orientation orient;
  orient.srcX = config.i(IMU_X_SRC);
  orient.srcY = config.i(IMU_Y_SRC);
  orient.srcZ = config.i(IMU_Z_SRC);
  orient.invX = config.b(IMU_INV_X);
  orient.invY = config.b(IMU_INV_Y);
  orient.invZ = config.b(IMU_INV_Z);
  const bool ok = imu.sample(orient, config.f(IMU_FILTER));
  if (ok)
    fsm.onTxnOk(now);
  else
    fsm.onTxnFail(now);
  i2cBus.noteTxn("imu sample", ok, !ok);
  if (ok && imuZeroRequested) {
    imuZeroRequested = false;
    imu.zero();
  }
}

/// 100 Hz: I2C sensor bus. A missing/failed device never blocks control.
static void sensorTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    sensorMon.beginCycle();
    const uint32_t now = millis();

    if (i2cBus.tryLock(2)) {
      if (i2cBus.recoverRequested()) {
        i2cBus.recover("web/usb");
      } else if (i2cBus.scanRequested()) {
        i2cBus.runScanIfRequested();
      } else {
        static uint32_t lastRecoverMs = 0;
        if ((!i2cBus.sdaHigh() || !i2cBus.sclHigh()) &&
            now - lastRecoverMs > 1000) {
          lastRecoverMs = now;
          i2cBus.recover("idle pins low");
        }
        probeOrSampleIna(now);
        probeOrSampleImu(now);
      }
      i2cBus.unlock();
    }

    const bool inaOk = i2cBus.inaFsm().online() && ina.health() == SensorHealth::OK;
    float roleA[3] = {0, 0, 0};
    if (inaOk) {
      const int roles[3] = {config.i(CUR_CH1_ROLE), config.i(CUR_CH2_ROLE),
                            config.i(CUR_CH3_ROLE)};
      for (int ch = 0; ch < 3; ++ch) {
        roleA[constrain(roles[ch], 0, 2)] = fabsf(ina.currentA(ch));
      }
    }

    // Optional sensors: NOT_PRESENT is commissioning, not a trip.
    if (ina.health() == SensorHealth::FAULT && i2cBus.inaFsm().everOnline())
      safety.raiseFault(FLT_INA_FAIL);
    else
      safety.clearFault(FLT_INA_FAIL);
    if (imu.health() == SensorHealth::FAULT && i2cBus.imuFsm().everOnline())
      safety.raiseFault(FLT_IMU_FAIL);
    else
      safety.clearFault(FLT_IMU_FAIL);

    telemetry.update([&](VehicleTelemetry& t) {
      t.power.busVoltage = inaOk ? ina.busVoltage() : 0.0f;
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
      webServer.broadcastSteerDiag();
    }
    pollUsbConsole();
    if (++divider >= 2) {  // 10 Hz
      divider = 0;
      recorder.record(telemetry.snapshot());
      statusLed.update(safety.state(),
                       nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT)));
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(50));
  }
}

/// 1 Hz: diagnostics, WiFi housekeeping, factory-reset button, OTA (manual)
static void systemTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  uint32_t buttonHeldSince = 0;
  uint32_t lastOtaCheck = 0;
  bool printedTable = false;
  for (;;) {
    diagnostics.tick();
    wifiManager.tick();
    webServer.tick();
    refreshCommissioning(commissioning);
    if (!printedTable && millis() > 3000) {
      printedTable = true;
      logCommissioningTable(commissioning);
    }

    static uint8_t nanoLogDiv = 0;
    if (++nanoLogDiv >= 10) {
      nanoLogDiv = 0;
      LOGI("NANO",
           "online=%d pkts=%lu telem=%lu crc=%lu frm=%lu bytes=%lu acks=%lu rx=%d",
           nano.online((uint32_t)config.i(SAF_NANO_TIMEOUT)) ? 1 : 0,
           (unsigned long)nano.packetsReceived(),
           (unsigned long)nano.telemetryReceived(),
           (unsigned long)nano.crcErrors(), (unsigned long)nano.frameErrors(),
           (unsigned long)nano.bytesReceived(),
           (unsigned long)nano.acksReceived(), nano.rxPinLevel());
    }

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

    // Auto GitHub check is OFF by default. If re-enabled, only while parked.
    const VehicleTelemetry snap = telemetry.snapshot();
    const bool parked = safety.state() != VehicleState::DRIVING &&
                        fabsf(snap.drive.vehicleSpeed) < 0.05f;
    if (config.b(OTA_AUTO_CHECK) && parked && !ota.busy() &&
        (lastOtaCheck == 0 || millis() - lastOtaCheck > 6UL * 3600 * 1000) &&
        millis() > 5UL * 60 * 1000) {
      lastOtaCheck = millis();
      LOGI("OTA", "auto-check (parked, explicitly enabled)");
      ota.checkGitHub();
    }

    const WifiStats ws = wifiManager.stats();
    telemetry.update([&](VehicleTelemetry& t) {
      t.system.uptimeS = diagnostics.uptimeS();
      t.system.freeHeap = diagnostics.freeHeap();
      t.system.minFreeHeap = diagnostics.minFreeHeap();
      t.system.psramFree = ESP.getFreePsram();
      t.system.largestHeap = ESP.getMaxAllocHeap();
      t.system.cpuLoadPct = diagnostics.cpuLoadPct();
      t.system.wifiRssi = wifiManager.rssi();
      t.system.wifiClients = wifiManager.clientCount();
      t.system.wifiAssoc = ws.assocCount;
      t.system.wifiDisc = ws.disconnectCount;
      t.system.wifiDhcpFail = ws.dhcpFailCount;
      t.system.bootCount = bootReport.report().bootCount;
      t.system.resetReason = bootReport.report().resetReason;
    });
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000));
  }
}

// ------------------------------------------------------------------ boot

static void bootstrap() {
  forceMotorsOff();

  // Confirm this OTA slot before USB CDC enumerates. The host toggling DTR
  // on the native USB port resets the chip; with rollback still pending the
  // bootloader would revert to the previous firmware.
  ota.confirmRunningImage();

  esp_err_t nvs = nvs_flash_init();
  if (nvs == ESP_ERR_NVS_NO_FREE_PAGES || nvs == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    nvs_flash_erase();
    nvs_flash_init();
  }

  Serial0.begin(115200);
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT
  Serial.setTxTimeoutMs(0);
#endif
  logger.begin();
  statusLed.begin();
  bootReport.captureReset();
  LOGI("BOOT", "Dodge Patrol VCM %s (%s, built %s)", VCM_FW_VERSION,
       VCM_GIT_COMMIT, VCM_BUILD_DATE);
  bootReport.logMem("early", bootReport.report().memEarly);

  pinMode(pins::BOOT_BUTTON, INPUT_PULLUP);

  config.begin();
  telemetry.begin();
  recorder.begin();
  bootReport.snap(bootReport.report().memAfterRecorders);
  bootReport.logMem("recorders", bootReport.report().memAfterRecorders);
  steerDiag.begin();
  safety.begin();
  calibration.begin();
  steerChar.begin();

  steering.begin();
  drive.begin();
  bootReport.mark("gpio/motors OFF", bootReport.report().tGpioMs);

  nano.begin();
  dynamics.begin();
  bootReport.mark("nano UART", bootReport.report().tNanoMs);

  // steer_motor calls ota.busy() on its first 5 ms cycle. Creating that
  // high-priority task before the OTA mutex exists panics:
  //   assert failed: xQueueSemaphoreTake queue.c (( pxQueue ))
  // and the chip reboot-loops — the "several minutes to boot" symptom.
  ota.begin();

  xTaskCreatePinnedToCore(steerMotorTask, "steer_motor", 8192, nullptr, 5,
                          nullptr, 1);
  xTaskCreatePinnedToCore(dynamicsTask, "dynamics", 8192, nullptr, 4, nullptr,
                          1);
  bootReport.mark("control tasks", bootReport.report().tControlMs);

  bootReport.snap(bootReport.report().memBeforeWifi);
  bootReport.logMem("before wifi", bootReport.report().memBeforeWifi);
  wifiManager.begin();
  bootReport.mark("wifi AP", bootReport.report().tWifiMs);
  bootReport.snap(bootReport.report().memAfterWifi);
  bootReport.logMem("after wifi", bootReport.report().memAfterWifi);

  webServer.begin();
  bootReport.mark("web server", bootReport.report().tWebMs);

  i2cBus.begin();
  bootReport.mark("i2c bus (no scan)", bootReport.report().tI2cMs);

  xTaskCreatePinnedToCore(sensorTask, "sensors", 6144, nullptr, 3, nullptr, 0);
  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 8192, nullptr, 2,
                          nullptr, 0);
  xTaskCreatePinnedToCore(systemTask, "system", 6144, nullptr, 1, nullptr, 0);

  safety.requestState(calibration.commissioned()
                          ? VehicleState::READY
                          : VehicleState::NOT_CALIBRATED,
                      "control live");
  bootReport.mark("ready", bootReport.report().tReadyMs);
  bootReport.snap(bootReport.report().memReady);

  refreshCommissioning(commissioning);
  logCommissioningTable(commissioning);
  LOGI("BOOT", "RC/steering live; I2C sensors probe in background");
}

}  // namespace vcm

void setup() { vcm::bootstrap(); }

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
