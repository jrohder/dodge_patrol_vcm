/**
 * @file api_routes.cpp
 * @brief REST API: system info, configuration, calibration, faults, logs.
 */
#include <AsyncJson.h>

#include "config/config_registry.h"
#include "control/vehicle_dynamics.h"
#include "core/version.h"
#include "drivers/i2c/i2c_bus.h"
#include "drivers/imu/mpu6050.h"
#include "drivers/ina3221/ina3221.h"
#include "drivers/uart/nano_link.h"
#include "drivers/wifi/wifi_manager.h"
#include "proto/protocol.h"
#include "services/boot_report.h"
#include "services/calibration.h"
#include "services/commissioning.h"
#include "services/diagnostics.h"
#include "services/event_recorder.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/telemetry.h"
#include "web/web_server.h"

#include <esp_ota_ops.h>

namespace vcm {

static void sendJson(AsyncWebServerRequest* req, JsonDocument& doc,
                     int code = 200) {
  String out;
  serializeJson(doc, out);
  req->send(code, "application/json", out);
}

static void sendOk(AsyncWebServerRequest* req, bool ok,
                   const char* err = nullptr) {
  JsonDocument doc;
  doc["ok"] = ok;
  if (err && !ok) doc["error"] = err;
  sendJson(req, doc, ok ? 200 : 400);
}

static const char* steeringCalStateName(SteeringCalState s) {
  switch (s) {
    case SteeringCalState::IDLE: return "IDLE";
    case SteeringCalState::MOVE_LEFT: return "MOVE_LEFT";
    case SteeringCalState::CONFIRM_LEFT: return "CONFIRM_LEFT";
    case SteeringCalState::SETTLE_LEFT: return "SETTLE_LEFT";
    case SteeringCalState::MOVE_RIGHT: return "MOVE_RIGHT";
    case SteeringCalState::CONFIRM_RIGHT: return "CONFIRM_RIGHT";
    case SteeringCalState::SETTLE_RIGHT: return "SETTLE_RIGHT";
    case SteeringCalState::DONE: return "DONE";
    case SteeringCalState::ABORTED: return "ABORTED";
  }
  return "?";
}

void VcmWebServer::setupApi() {
  // ---------------------------------------------------------------- system
  server_.on("/api/system", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["fw_version"] = VCM_FW_VERSION;
    doc["git_commit"] = VCM_GIT_COMMIT;
    doc["build_date"] = VCM_BUILD_DATE;
    doc["protocol_version"] = veio::proto::kProtocolVersion;
    doc["units"] = config.i(UI_UNITS) == 0 ? "IMPERIAL" : "METRIC";
    doc["units_id"] = config.i(UI_UNITS);
    doc["chip"] = ESP.getChipModel();
    doc["flash_kb"] = ESP.getFlashChipSize() / 1024;
    doc["sketch_kb"] = ESP.getSketchSize() / 1024;
    const esp_partition_t* part = esp_ota_get_running_partition();
    doc["partition"] = part ? part->label : "?";
    doc["uptime_s"] = diagnostics.uptimeS();
    doc["heap"] = ESP.getFreeHeap();
    doc["min_heap"] = ESP.getMinFreeHeap();
    doc["largest_heap"] = ESP.getMaxAllocHeap();
    doc["psram"] = ESP.getFreePsram();
    doc["psram_size"] = ESP.getPsramSize();
    doc["state"] = vehicleStateName(safety.state());
    doc["commissioned"] = calibration.commissioned();
    doc["ip"] = wifiManager.ipAddress();
    doc["ap_active"] = wifiManager.apActive();
    doc["hostname"] = config.s(S_HOSTNAME);
    doc["vehicle_name"] = config.s(S_VEHICLE_NAME);
    const BootReport& br = bootReport.report();
    doc["reset_reason"] = br.resetName;
    doc["reset_code"] = br.resetReason;
    doc["reset_cpu0"] = br.resetReasonCpu0;
    doc["brownout"] = br.brownout;
    doc["panic"] = br.panic;
    doc["watchdog"] = br.watchdog;
    doc["boot_count"] = br.bootCount;
    doc["boot_gpio_ms"] = br.tGpioMs;
    doc["boot_nano_ms"] = br.tNanoMs;
    doc["boot_control_ms"] = br.tControlMs;
    doc["boot_wifi_ms"] = br.tWifiMs;
    doc["boot_web_ms"] = br.tWebMs;
    doc["boot_i2c_ms"] = br.tI2cMs;
    doc["boot_ready_ms"] = br.tReadyMs;
    const WifiStats wst = wifiManager.stats();
    doc["ap_uptime_s"] = wifiManager.apUptimeS();
    doc["ap_assoc"] = wst.assocCount;
    doc["ap_disc"] = wst.disconnectCount;
    doc["ap_dhcp_ok"] = wst.dhcpAssignCount;
    doc["ap_dhcp_fail"] = wst.dhcpFailCount;
    doc["ap_stop"] = wst.apStopCount;
    doc["wifi_reset"] = wst.wifiResetCount;
    doc["wifi_rssi"] = wifiManager.rssi();
    doc["wifi_clients"] = wifiManager.clientCount();
    sendJson(req, doc);
  });

  server_.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest* req) {
    sendOk(req, true);
    LOGW("WEB", "Reboot requested from web UI");
    // Delay so the HTTP response gets out first
    xTaskCreate([](void*) { vTaskDelay(pdMS_TO_TICKS(500)); ESP.restart(); },
                "reboot", 2048, nullptr, 1, nullptr);
  });

  // ---------------------------------------------------------------- config
  server_.on("/api/config/schema", HTTP_GET, [](AsyncWebServerRequest* req) {
    // Schema is large: stream it in chunks from a heap-allocated string
    auto* doc = new JsonDocument();
    config.schemaJson(*doc);
    String* out = new String();
    serializeJson(*doc, *out);
    delete doc;
    AsyncWebServerResponse* res = req->beginChunkedResponse(
        "application/json",
        [out](uint8_t* buffer, size_t maxLen, size_t index) -> size_t {
          const size_t remaining = out->length() - index;
          const size_t n = min(maxLen, remaining);
          if (n == 0) {
            delete out;
            return 0;
          }
          memcpy(buffer, out->c_str() + index, n);
          return n;
        });
    req->send(res);
  });

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/config/apply",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        int applied = 0, failed = 0;
        for (JsonPair kv : json.as<JsonObject>()) {
          bool ok;
          if (kv.value().is<const char*>()) {
            ok = config.applyStringByKey(kv.key().c_str(),
                                         kv.value().as<const char*>());
          } else {
            ok = config.applyByKey(kv.key().c_str(), kv.value().as<float>());
          }
          ok ? applied++ : failed++;
        }
        LOGD("WEB", "Config apply: %d ok, %d failed", applied, failed);
        JsonDocument doc;
        doc["ok"] = failed == 0;
        doc["applied"] = applied;
        doc["failed"] = failed;
        sendJson(req, doc);
      }));

  server_.on("/api/config/save", HTTP_POST, [](AsyncWebServerRequest* req) {
    config.save();
    sendOk(req, true);
  });
  server_.on("/api/config/revert", HTTP_POST, [](AsyncWebServerRequest* req) {
    config.revert();
    sendOk(req, true);
  });
  server_.on("/api/config/factory-reset", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               config.factoryReset();
               sendOk(req, true);
             });

  server_.on("/api/config/export", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    config.exportJson(doc);
    doc["fw_version"] = VCM_FW_VERSION;
    String out;
    serializeJsonPretty(doc, out);
    AsyncWebServerResponse* res =
        req->beginResponse(200, "application/json", out);
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"dodge_patrol_vcm_config.json\"");
    req->send(res);
  });

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/config/import",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        JsonDocument doc;
        doc.set(json);
        const int applied = config.importJson(doc);
        JsonDocument res;
        res["ok"] = applied > 0;
        res["applied"] = applied;
        sendJson(req, res);
      }));

  // ---------------------------------------------------------------- faults
  server_.on("/api/faults", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    JsonArray arr = doc["faults"].to<JsonArray>();
    const FaultRecord* recs = safety.records();
    for (int i = 0; i < FLT_COUNT; ++i) {
      const FaultInfo& fi = SafetyManager::info((FaultId)i);
      if (!recs[i].active && recs[i].count == 0) continue;
      JsonObject o = arr.add<JsonObject>();
      o["code"] = fi.code;
      o["desc"] = fi.desc;
      o["severity"] =
          fi.severity == FaultSeverity::TRIP ? "TRIP" : "WARNING";
      o["active"] = recs[i].active;
      o["count"] = recs[i].count;
      o["first_ms"] = recs[i].firstMs;
      o["last_ms"] = recs[i].lastMs;
    }
    doc["state"] = vehicleStateName(safety.state());
    sendJson(req, doc);
  });
  server_.on("/api/faults/clear", HTTP_POST, [](AsyncWebServerRequest* req) {
    safety.clearHistory();
    sendOk(req, true);
  });
  server_.on("/api/estop", HTTP_POST, [](AsyncWebServerRequest* req) {
    safety.estop("web API");
    sendOk(req, true);
  });
  server_.on("/api/estop/clear", HTTP_POST, [](AsyncWebServerRequest* req) {
    safety.clearEstop();
    sendOk(req, true);
  });

  // ---------------------------------------------------------------- logs
  server_.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest* req) {
    uint32_t after = 0;
    if (req->hasParam("after"))
      after = req->getParam("after")->value().toInt();
    static LogEntry entries[48];
    const size_t n = logger.fetch(after, entries, 48);
    JsonDocument doc;
    JsonArray arr = doc["entries"].to<JsonArray>();
    for (size_t i = 0; i < n; ++i) {
      JsonObject o = arr.add<JsonObject>();
      o["seq"] = entries[i].seq;
      o["ms"] = entries[i].ms;
      o["lvl"] = Logger::levelName(entries[i].level);
      o["mod"] = entries[i].module;
      o["msg"] = entries[i].message;
    }
    doc["latest"] = logger.latestSeq();
    sendJson(req, doc);
  });
  server_.on("/api/logs/download", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* res =
        req->beginResponse(200, "text/plain", logger.dumpText());
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"vcm_logs.txt\"");
    req->send(res);
  });

  // ---------------------------------------------------------------- nano
  server_.on("/api/nano/version", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["have"] = nano.haveVersion();
    if (nano.haveVersion()) {
      const auto& v = nano.version();
      doc["fw"] = String(v.fwMajor) + "." + String(v.fwMinor) + "." +
                  String(v.fwPatch);
      doc["protocol"] = v.protocolVersion;
      doc["git_hash"] = v.gitHash;
      doc["build_date"] = String(v.buildDate);
      doc["build_time"] = String(v.buildTime);
      doc["board_id"] = v.boardId;
      doc["boot_reason"] = v.bootReason;
    }
    sendJson(req, doc);
  });
  server_.on("/api/nano/diagnostics", HTTP_GET,
             [](AsyncWebServerRequest* req) {
               // Request a single diagnostic snapshot from the Nano.
               uint8_t args[4] = {0, 0, 0, 0};
               nano.sendCommand(veio::proto::kCmdStartDiagnostics, args);
               sendOk(req, true);
             });
  server_.on("/api/nano/ping", HTTP_POST, [](AsyncWebServerRequest* req) {
    nano.sendPing();
    sendOk(req, true);
  });

  // ---------------------------------------------------------------- recorder
  server_.on("/api/recorder", HTTP_GET, [](AsyncWebServerRequest* req) {
    AsyncWebServerResponse* res =
        req->beginResponse(200, "text/csv", recorder.dumpCsv());
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"vcm_recorder.csv\"");
    req->send(res);
  });
  server_.on("/api/recorder/resume", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               recorder.unfreeze();
               sendOk(req, true);
             });

  // ---------------------------------------------------------------- calibration
  server_.on("/api/cal/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    const SteeringCalData& sc = calibration.steeringCal();
    JsonObject s = doc["steering"].to<JsonObject>();
    s["state"] = steeringCalStateName(calibration.steeringCalState());
    s["valid"] = sc.valid;
    s["left_adc"] = sc.leftAdc;
    s["right_adc"] = sc.rightAdc;
    s["center_adc"] = sc.centerAdc;
    s["timestamp"] = sc.timestamp;
    s["fw_version"] = sc.fwVersion;
    float sl, sr;
    calibration.softLimits(sl, sr);
    s["soft_left"] = sl;
    s["soft_right"] = sr;
    s["center_effective"] = calibration.centerAdc();
    const MotorTest& mt = calibration.motorTest();
    JsonObject m = doc["motor_test"].to<JsonObject>();
    m["active"] = mt.active;
    m["left"] = mt.left;
    m["right"] = mt.right;
    m["pwm"] = mt.pwmPct;
    doc["manual_steer"] = calibration.manualSteeringActive();
    doc["commissioned"] = calibration.commissioned();
    sendJson(req, doc);
  });

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/steering/start",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        SteeringCalParams p;
        p.pwmPct = json["pwm"] | p.pwmPct;
        p.currentThresholdA = json["current"] | p.currentThresholdA;
        p.noMotionMs = json["no_motion_ms"] | p.noMotionMs;
        sendOk(req, calibration.startSteeringCal(p),
               "cannot start (vehicle busy)");
      }));
  server_.on("/api/cal/steering/abort", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               calibration.abortSteeringCal();
               sendOk(req, true);
             });
  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/steering/manual",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        calibration.setSteeringLimits(json["left"] | 0.0f,
                                      json["right"] | 0.0f,
                                      json["center"] | 0.0f);
        sendOk(req, calibration.steeringCal().valid, "invalid limits");
      }));
  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/steering/target",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        calibration.setManualSteeringTarget(json["pct"] | 50.0f,
                                            json["enabled"] | false);
        sendOk(req, true);
      }));

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/motor-test",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        const bool ok = calibration.startMotorTest(
            json["left"] | false, json["right"] | false, json["pwm"] | 10.0f,
            json["duration_ms"] | 5000);
        sendOk(req, ok, "cannot start (vehicle busy)");
      }));
  server_.on("/api/cal/motor-test/stop", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               calibration.stopMotorTest();
               sendOk(req, true);
             });

  server_.on("/api/cal/imu/zero", HTTP_POST, [](AsyncWebServerRequest* req) {
    extern void requestImuZero();
    requestImuZero();
    sendOk(req, true);
  });
  server_.on("/api/cal/current/zero", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               const VehicleTelemetry t = telemetry.snapshot();
               // average of channels as measured zero offset
               calibration.captureCurrentOffset(
                   (t.power.leftCurrentA + t.power.rightCurrentA +
                    t.power.steeringCurrentA) / 3.0f);
               sendOk(req, true);
             });

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/commission",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        const bool done = json["done"] | true;
        if (done && !calibration.steeringCal().valid) {
          sendOk(req, false, "steering calibration required first");
          return;
        }
        calibration.setCommissioned(done);
        sendOk(req, true);
      }));

  server_.on("/api/trip/reset", HTTP_POST, [](AsyncWebServerRequest* req) {
    dynamics.resetTrip();
    sendOk(req, true);
  });

  // ---------------------------------------------------------------- I2C / commissioning
  server_.on("/api/i2c", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    const I2cBusStats st = i2cBus.stats();
    const I2cScanResult scan = i2cBus.lastScan();
    doc["sda"] = i2cBus.sdaPin();
    doc["scl"] = i2cBus.sclPin();
    doc["freq_hz"] = i2cBus.freqHz();
    doc["bus_state"] = i2cBus.busStateName();
    doc["sda_high"] = i2cBus.sdaHigh();
    doc["scl_high"] = i2cBus.sclHigh();
    doc["last_txn"] = st.lastTxn;
    doc["last_txn_ok"] = st.lastTxnOk;
    doc["last_txn_ms"] = st.lastTxnMs;
    doc["errors"] = st.errors;
    doc["timeouts"] = st.timeouts;
    doc["recoveries"] = st.recoveries;
    doc["scans"] = st.scans;
    doc["scan_pending"] = i2cBus.scanRequested();
    doc["scan_in_progress"] = scan.inProgress;
    doc["scan_ms"] = scan.durationMs;
    JsonArray found = doc["found"].to<JsonArray>();
    if (scan.valid) {
      for (uint8_t i = 0; i < scan.count && i < 16; ++i) {
        JsonObject o = found.add<JsonObject>();
        o["addr"] = scan.addrs[i];
        o["hex"] = String("0x") + String(scan.addrs[i], HEX);
        o["name"] = I2cBus::nameForAddr(scan.addrs[i]);
      }
    }
    JsonObject inaO = doc["ina"].to<JsonObject>();
    inaO["addr"] = ina.address();
    inaO["state"] = i2cDevStateName(i2cBus.inaFsm().state());
    inaO["health"] = sensorHealthName(ina.health());
    inaO["manuf"] = ina.manufId();
    JsonObject imuO = doc["imu"].to<JsonObject>();
    imuO["addr"] = imu.address();
    imuO["state"] = i2cDevStateName(i2cBus.imuFsm().state());
    imuO["health"] = sensorHealthName(imu.health());
    imuO["whoami"] = imu.whoAmI();
    sendJson(req, doc);
  });

  server_.on("/api/i2c/scan", HTTP_POST, [](AsyncWebServerRequest* req) {
    i2cBus.requestScan();
    sendOk(req, true);
  });
  server_.on("/api/i2c/recover", HTTP_POST, [](AsyncWebServerRequest* req) {
    i2cBus.requestRecover();
    sendOk(req, true);
  });

  server_.on("/api/commissioning", HTTP_GET, [](AsyncWebServerRequest* req) {
    refreshCommissioning(commissioning);
    const CommissioningReport& c = commissioning;
    JsonDocument doc;
    doc["firmware"] = c.firmware;
    doc["overall"] = c.overall;
    doc["ip"] = c.ip;
    doc["heap"] = c.freeHeap;
    auto add = [&](JsonArray arr, const char* name, SubsystemStatus s) {
      JsonObject o = arr.add<JsonObject>();
      o["name"] = name;
      o["status"] = subsystemStatusName(s);
    };
    JsonArray core = doc["core"].to<JsonArray>();
    add(core, "ESP32-S3", c.esp);
    add(core, "Heap", c.heap);
    add(core, "PSRAM", c.psramStatus);
    add(core, "Safety", c.safety);
    JsonArray ctl = doc["control"].to<JsonArray>();
    add(ctl, "RC receiver", c.rc);
    add(ctl, "Nano link", c.nano);
    add(ctl, "Steering feedback", c.steerFb);
    add(ctl, "Steering actuator", c.steerAct);
    add(ctl, "Left motor", c.leftMot);
    add(ctl, "Right motor", c.rightMot);
    JsonArray sns = doc["sensors"].to<JsonArray>();
    add(sns, "INA3221", c.ina);
    add(sns, "MPU6050", c.imu);
    add(sns, "Left speed", c.leftSpeed);
    add(sns, "Right speed", c.rightSpeed);
    add(sns, "Steering wheel", c.wheel);
    add(sns, "Pedal", c.pedal);
    JsonArray net = doc["network"].to<JsonArray>();
    add(net, "Wi-Fi AP", c.wifi);
    add(net, "Web server", c.web);
    sendJson(req, doc);
  });
}

}  // namespace vcm
