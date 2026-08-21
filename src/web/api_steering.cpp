/**
 * @file api_steering.cpp
 * @brief REST API for steering diagnostics and smart characterization.
 *
 * Heavy work (CSV/JSON generation) runs in the async web task, never
 * inside the 200 Hz steering loop. Samples are streamed; no giant String
 * concatenation.
 */
#include <AsyncJson.h>
#include <algorithm>
#include <cstdio>
#include <cstring>

#include "config/config_registry.h"
#include "control/steering_compensation.h"
#include "control/steering_types.h"
#include "core/version.h"
#include "services/calibration.h"
#include "services/steering_characterization.h"
#include "services/steering_recorder.h"
#include "services/telemetry.h"
#include "web/web_server.h"

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

static void fillChar(JsonObject o, const SteeringCharacterization& c) {
  o["valid"] = c.status != SteerCharStatus::INVALID;
  o["status"] = c.status == SteerCharStatus::OK       ? "OK"
                : c.status == SteerCharStatus::FAILED ? "FAILED"
                                                      : "INVALID";
  o["timestamp"] = c.timestamp;
  o["fw_version"] = c.fwVersion;
  o["min_start_left"] = serialized(String(c.minStartLeft, 2));
  o["min_start_right"] = serialized(String(c.minStartRight, 2));
  o["hold_left"] = serialized(String(c.holdLeft, 2));
  o["hold_right"] = serialized(String(c.holdRight, 2));
  o["preferred_left"] = serialized(String(c.preferredLeft, 2));
  o["preferred_right"] = serialized(String(c.preferredRight, 2));
  o["max_test_left"] = serialized(String(c.maxTestLeft, 2));
  o["max_test_right"] = serialized(String(c.maxTestRight, 2));
  o["max_vel_left"] = serialized(String(c.maxVelLeft, 3));
  o["max_vel_right"] = serialized(String(c.maxVelRight, 3));
  o["backlash_left"] = serialized(String(c.backlashLeft, 2));
  o["backlash_right"] = serialized(String(c.backlashRight, 2));
  o["overshoot"] = serialized(String(c.overshootPct, 2));
  o["settling_s"] = serialized(String(c.settlingTimeS, 3));
  o["final_error"] = serialized(String(c.finalErrorPct, 2));
  o["dir_reversals"] = c.dirReversals;
  o["fail_reason"] = c.failReason;
  JsonArray pwm = o["pwm"].to<JsonArray>();
  JsonArray vl = o["vel_left"].to<JsonArray>();
  JsonArray vr = o["vel_right"].to<JsonArray>();
  for (uint8_t i = 0; i < c.nPoints; ++i) {
    pwm.add(serialized(String(c.pwm[i], 2)));
    vl.add(serialized(String(c.velLeft[i], 3)));
    vr.add(serialized(String(c.velRight[i], 3)));
  }
}

static void fillRecs(JsonObject o, const SteerRecommendations& r) {
  o["valid"] = r.valid;
  o["kp"] = r.kp;
  o["ki"] = r.ki;
  o["kd"] = r.kd;
  o["far_p"] = r.farP;
  o["near_p"] = r.nearP;
  o["hold_p"] = r.holdP;
  o["start_left"] = r.startLeft;
  o["start_right"] = r.startRight;
  o["hold_left"] = r.holdLeft;
  o["hold_right"] = r.holdRight;
  o["deadband"] = r.deadband;
  o["settling_threshold"] = r.settlingThreshold;
  o["pwm_slew"] = r.pwmSlew;
  o["max_velocity"] = r.maxVelocity;
}

static bool applyRecommended() {
  const SteerRecommendations& r = steerChar.recommended();
  if (!r.valid) return false;
  config.applyByKey("steering.pid_kp", r.kp);
  config.applyByKey("steering.pid_ki", r.ki);
  config.applyByKey("steering.pid_kd", r.kd);
  config.applyByKey("steering.far_p_gain", r.farP);
  config.applyByKey("steering.near_p_gain", r.nearP);
  config.applyByKey("steering.hold_p_gain", r.holdP);
  config.applyByKey("steering.minimum_start_pwm_left", r.startLeft);
  config.applyByKey("steering.minimum_start_pwm_right", r.startRight);
  config.applyByKey("steering.minimum_hold_pwm_left", r.holdLeft);
  config.applyByKey("steering.minimum_hold_pwm_right", r.holdRight);
  config.applyByKey("steering.deadband", r.deadband);
  config.applyByKey("steering.settling_threshold", r.settlingThreshold);
  config.applyByKey("steering.max_pwm_ramp_up", r.pwmSlew);
  config.applyByKey("steering.max_pwm_ramp_down", r.pwmSlew);
  return true;
}

struct SteerCsvJob {
  size_t i = 0;
  size_t n = 0;
  uint32_t t0 = 0;
  bool header = false;
};

static const char kCsvHeader[] =
    "timestamp_us,time_s,setpoint,raw_feedback,filtered_feedback,error,pwm,"
    "direction,p_term,i_term,d_term,feedforward,velocity,control_state,"
    "current\n";

void VcmWebServer::setupSteeringApi() {
  server_.on("/api/steer/diag/status", HTTP_GET,
             [](AsyncWebServerRequest* req) {
               JsonDocument doc;
               doc["capacity"] = steerDiag.capacity();
               doc["count"] = steerDiag.count();
               doc["seq"] = steerDiag.seq();
               doc["recording"] = steerDiag.recording();
               doc["psram"] = steerDiag.usingPsram();
               const auto st = steerDiag.stats();
               doc["rms_error"] = serialized(String(st.rmsError, 3));
               doc["max_error"] = serialized(String(st.maxError, 3));
               doc["avg_error"] = serialized(String(st.avgError, 3));
               doc["peak_pwm"] = serialized(String(st.peakPwm, 2));
               doc["avg_pwm"] = serialized(String(st.avgPwm, 2));
               doc["max_velocity"] = serialized(String(st.maxVelocity, 2));
               doc["dir_changes"] = st.dirChanges;
               doc["pwm_reversals"] = st.pwmReversals;
               int pr = 0, ec = 0;
               bool hun = false;
               uint8_t sev = 0;
               steerDiag.computeHunting(pr, ec, hun, sev);
               doc["hunting"] = hun;
               doc["hunting_severity"] = sev;
               const VehicleTelemetry t = telemetry.snapshot();
               doc["health"] = serialized(String(
                   steeringHealthScore(st.rmsError, hun,
                                       t.steering.calibrated,
                                       config.f(STR_MIN_START_L),
                                       steerChar.baseline().minStartLeft),
                   0));
               sendJson(req, doc);
             });

  server_.on("/api/steer/diag/clear", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               steerDiag.requestClear();
               sendOk(req, true);
             });
  server_.on("/api/steer/diag/record", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               bool on = true;
               if (req->hasParam("on", true))
                 on = req->getParam("on", true)->value() != "0";
               else if (req->hasParam("on"))
                 on = req->getParam("on")->value() != "0";
               steerDiag.setRecording(on);
               sendOk(req, true);
             });

  server_.on("/api/steer/diag/csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    auto* job = new SteerCsvJob();
    job->n = steerDiag.count();
    SteeringDiagPacked first{};
    if (job->n > 0 && steerDiag.peekPacked(0, first)) job->t0 = first.t_us;
    AsyncWebServerResponse* res = req->beginChunkedResponse(
        "text/csv",
        [job](uint8_t* buffer, size_t maxLen, size_t) -> size_t {
          size_t used = 0;
          auto append = [&](const char* s, size_t n) -> bool {
            if (used + n > maxLen) return false;
            memcpy(buffer + used, s, n);
            used += n;
            return true;
          };
          if (!job->header) {
            const size_t n = sizeof(kCsvHeader) - 1;
            if (n > maxLen) return 0;
            memcpy(buffer, kCsvHeader, n);
            job->header = true;
            return n;
          }
          char line[192];
          while (job->i < job->n) {
            SteeringDiagPacked p{};
            if (!steerDiag.peekPacked(job->i, p)) break;
            const SteeringDiagnosticSample s = unpackSteerSample(p);
            const float tS =
                (s.timestamp_us - job->t0) * 1e-6f;
            int n;
            if (s.current_valid) {
              n = snprintf(
                  line, sizeof(line),
                  "%lu,%.6f,%.2f,%.0f,%.2f,%.3f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,"
                  "%.2f,%s,%.3f\n",
                  (unsigned long)s.timestamp_us, tS, s.setpoint,
                  s.raw_feedback, s.filtered_feedback, s.error, s.pwm,
                  (int)s.direction, s.p_term, s.i_term, s.d_term,
                  s.feedforward, s.actuator_velocity,
                  steerControlStateName(s.control_state), s.current_a);
            } else {
              n = snprintf(
                  line, sizeof(line),
                  "%lu,%.6f,%.2f,%.0f,%.2f,%.3f,%.2f,%d,%.2f,%.2f,%.2f,%.2f,"
                  "%.2f,%s,\n",
                  (unsigned long)s.timestamp_us, tS, s.setpoint,
                  s.raw_feedback, s.filtered_feedback, s.error, s.pwm,
                  (int)s.direction, s.p_term, s.i_term, s.d_term,
                  s.feedforward, s.actuator_velocity,
                  steerControlStateName(s.control_state));
            }
            if (n < 0) break;
            if (!append(line, (size_t)n)) break;
            job->i++;
          }
          if (used == 0) {
            delete job;
            return 0;
          }
          return used;
        });
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"steering_diag.csv\"");
    req->send(res);
  });

  server_.on("/api/steer/diag.bin", HTTP_GET, [](AsyncWebServerRequest* req) {
    struct BinJob {
      size_t i = 0;
      size_t n = 0;
      bool header = false;
    };
    auto* job = new BinJob();
    job->n = steerDiag.count();
    AsyncWebServerResponse* res = req->beginChunkedResponse(
        "application/octet-stream",
        [job](uint8_t* buffer, size_t maxLen, size_t) -> size_t {
          if (!job->header) {
            if (maxLen < 12) return 0;
            uint16_t n = (uint16_t)std::min(job->n, (size_t)65535);
            buffer[0] = 'S';
            buffer[1] = 'D';
            buffer[2] = 1;
            buffer[3] = 1;  // kind = snapshot
            memcpy(buffer + 4, &n, 2);
            uint32_t seq = steerDiag.seq();
            memcpy(buffer + 6, &seq, 4);
            job->header = true;
            return 12;
          }
          const size_t sampleSize = sizeof(SteeringDiagPacked);
          size_t maxSamp = maxLen / sampleSize;
          size_t wrote = 0;
          for (size_t k = 0; k < maxSamp && job->i < job->n; ++k) {
            SteeringDiagPacked p{};
            if (!steerDiag.peekPacked(job->i, p)) break;
            memcpy(buffer + wrote, &p, sampleSize);
            wrote += sampleSize;
            job->i++;
          }
          if (wrote == 0) {
            delete job;
            return 0;
          }
          return wrote;
        });
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"steering_diag.bin\"");
    req->send(res);
  });

  server_.on("/api/steer/diag/json", HTTP_GET, [](AsyncWebServerRequest* req) {
    struct JsJob {
      int phase = 0;
      size_t i = 0;
      size_t n = 0;
      size_t evI = 0;
      size_t evN = 0;
      SteerEventMarker ev[128];
    };
    auto* job = new JsJob();
    job->n = steerDiag.count();
    job->evN = steerDiag.copyEvents(job->ev, 128);
    AsyncWebServerResponse* res = req->beginChunkedResponse(
        "application/json",
        [job](uint8_t* buffer, size_t maxLen, size_t) -> size_t {
          char tmp[512];
          size_t used = 0;
          auto put = [&](const char* s) {
            const size_t n = strlen(s);
            if (used + n > maxLen) return false;
            memcpy(buffer + used, s, n);
            used += n;
            return true;
          };
          if (job->phase == 0) {
            const String veh = config.s(S_VEHICLE_NAME);
            char vehBuf[32];
            strncpy(vehBuf, veh.c_str(), sizeof(vehBuf) - 1);
            vehBuf[sizeof(vehBuf) - 1] = 0;
            snprintf(tmp, sizeof(tmp),
                     "{\"firmware\":\"%s\",\"git\":\"%s\",\"schema\":%u,"
                     "\"cal_version\":\"%s\",\"timestamp_ms\":%lu,"
                     "\"vehicle\":\"%s\","
                     "\"char_start_pwm\":%.1f,\"char_pwm_increment\":%.1f,"
                     "\"char_max_pwm\":%.1f,\"char_dwell_ms\":%d,"
                     "\"char_motion_threshold\":%.2f,\"count\":%u,\"samples\":[",
                     VCM_FW_VERSION, VCM_GIT_COMMIT,
                     (unsigned)ConfigRegistry::SCHEMA_VERSION,
                     calibration.steeringCal().fwVersion,
                     (unsigned long)millis(),
                     vehBuf,
                     config.f(STR_CHAR_START_PWM), config.f(STR_CHAR_PWM_STEP),
                     config.f(STR_CHAR_MAX_PWM), config.i(STR_CHAR_DWELL_MS),
                     config.f(STR_CHAR_MOTION_PCT),
                     (unsigned)job->n);
            if (!put(tmp)) return 0;
            job->phase = 1;
            return used;
          }
          if (job->phase == 1) {
            while (job->i < job->n) {
              SteeringDiagPacked p{};
              if (!steerDiag.peekPacked(job->i, p)) break;
              const auto s = unpackSteerSample(p);
              int n;
              if (s.current_valid) {
                n = snprintf(
                    tmp, sizeof(tmp),
                    "%s{\"t\":%lu,\"sp\":%.2f,\"raw\":%.0f,\"filt\":%.2f,"
                    "\"err\":%.3f,\"pwm\":%.2f,\"dir\":%d,\"p\":%.2f,"
                    "\"i\":%.2f,\"d\":%.2f,\"ff\":%.2f,\"vel\":%.2f,"
                    "\"st\":\"%s\",\"cur\":%.3f}",
                    job->i ? "," : "", (unsigned long)s.timestamp_us,
                    s.setpoint, s.raw_feedback, s.filtered_feedback, s.error,
                    s.pwm, (int)s.direction, s.p_term, s.i_term, s.d_term,
                    s.feedforward, s.actuator_velocity,
                    steerControlStateName(s.control_state), s.current_a);
              } else {
                n = snprintf(
                    tmp, sizeof(tmp),
                    "%s{\"t\":%lu,\"sp\":%.2f,\"raw\":%.0f,\"filt\":%.2f,"
                    "\"err\":%.3f,\"pwm\":%.2f,\"dir\":%d,\"p\":%.2f,"
                    "\"i\":%.2f,\"d\":%.2f,\"ff\":%.2f,\"vel\":%.2f,"
                    "\"st\":\"%s\"}",
                    job->i ? "," : "", (unsigned long)s.timestamp_us,
                    s.setpoint, s.raw_feedback, s.filtered_feedback, s.error,
                    s.pwm, (int)s.direction, s.p_term, s.i_term, s.d_term,
                    s.feedforward, s.actuator_velocity,
                    steerControlStateName(s.control_state));
              }
              if (n < 0 || used + (size_t)n > maxLen) break;
              memcpy(buffer + used, tmp, (size_t)n);
              used += (size_t)n;
              job->i++;
            }
            if (job->i >= job->n) job->phase = 2;
            if (used) return used;
          }
          if (job->phase == 2) {
            if (!put("],\"events\":[")) return 0;
            job->phase = 3;
            return used;
          }
          if (job->phase == 3) {
            while (job->evI < job->evN) {
              const SteerEventMarker& e = job->ev[job->evI];
              int n = snprintf(tmp, sizeof(tmp),
                               "%s{\"t\":%lu,\"type\":\"%s\",\"v\":%.1f}",
                               job->evI ? "," : "", (unsigned long)e.t_us,
                               steerDiagEventName(e.type), e.value_x10 / 10.0f);
              if (n < 0 || used + (size_t)n > maxLen) break;
              memcpy(buffer + used, tmp, (size_t)n);
              used += (size_t)n;
              job->evI++;
            }
            if (job->evI >= job->evN) job->phase = 4;
            if (used) return used;
          }
          if (job->phase == 4) {
            if (!put("]}")) return 0;
            job->phase = 5;
            return used;
          }
          delete job;
          return 0;
        });
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"steering_diag.json\"");
    req->send(res);
  });

  server_.on("/api/steer/diag/events", HTTP_GET,
             [](AsyncWebServerRequest* req) {
               SteerEventMarker ev[128];
               const size_t n = steerDiag.copyEvents(ev, 128);
               JsonDocument doc;
               JsonArray arr = doc["events"].to<JsonArray>();
               for (size_t i = 0; i < n; ++i) {
                 JsonObject o = arr.add<JsonObject>();
                 o["t"] = ev[i].t_us;
                 o["type"] = steerDiagEventName(ev[i].type);
                 o["code"] = (uint8_t)ev[i].type;
                 o["v"] = ev[i].value_x10 / 10.0f;
               }
               sendJson(req, doc);
             });

  // ---- characterization -------------------------------------------------
  server_.on("/api/cal/char/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    const auto& live = steerChar.live();
    doc["phase"] = SteeringCharacterizationService::phaseName(live.phase);
    doc["step"] = live.stepName;
    doc["message"] = live.message;
    doc["progress"] = live.progressPct;
    doc["active"] = steerChar.active();
    doc["open_loop"] = live.openLoop;
    doc["pwm"] = live.pwmCmd;
    doc["pwm_level"] = live.pwmLevel;
    fillChar(doc["current"].to<JsonObject>(), steerChar.result());
    fillChar(doc["baseline"].to<JsonObject>(), steerChar.baseline());
    fillRecs(doc["recommended"].to<JsonObject>(), steerChar.recommended());
    JsonObject cur = doc["settings"].to<JsonObject>();
    cur["kp"] = config.f(STR_PID_KP);
    cur["ki"] = config.f(STR_PID_KI);
    cur["kd"] = config.f(STR_PID_KD);
    cur["far_p"] = config.f(STR_FAR_P);
    cur["near_p"] = config.f(STR_NEAR_P);
    cur["hold_p"] = config.f(STR_HOLD_P);
    cur["start_left"] = config.f(STR_MIN_START_L);
    cur["start_right"] = config.f(STR_MIN_START_R);
    cur["hold_left"] = config.f(STR_MIN_HOLD_L);
    cur["hold_right"] = config.f(STR_MIN_HOLD_R);
    cur["deadband"] = config.f(STR_DEADBAND);
    cur["ff"] = config.b(STR_FF_EN);
    const SteerDeviation d =
        compareCharacterization(steerChar.baseline(), steerChar.result());
    JsonObject dev = doc["deviation"].to<JsonObject>();
    dev["comparable"] = d.comparable;
    dev["min_pwm_left"] = serialized(String(d.minPwmLeftPct * 100.0f, 1));
    dev["min_pwm_right"] = serialized(String(d.minPwmRightPct * 100.0f, 1));
    dev["max_vel_left"] = serialized(String(d.maxVelLeftPct * 100.0f, 1));
    dev["max_vel_right"] = serialized(String(d.maxVelRightPct * 100.0f, 1));
    dev["hold_left"] = serialized(String(d.holdLeftPct * 100.0f, 1));
    dev["hold_right"] = serialized(String(d.holdRightPct * 100.0f, 1));
    sendJson(req, doc);
  });

  server_.addHandler(new AsyncCallbackJsonWebHandler(
      "/api/cal/char/start",
      [](AsyncWebServerRequest* req, JsonVariant& json) {
        const bool confirm = json["confirm"] | false;
        const char* ack = json["ack"] | "";
        if (!confirm || strcmp(ack, "ACTUATOR WILL MOVE") != 0) {
          sendOk(req, false, "confirmation required");
          return;
        }
        sendOk(req, steerChar.start(true),
               "cannot start (busy, disabled, or unsafe)");
      }));
  server_.on("/api/cal/char/abort", HTTP_POST, [](AsyncWebServerRequest* req) {
    steerChar.abort("aborted by user");
    sendOk(req, true);
  });
  server_.on("/api/cal/char/apply-recommended", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               sendOk(req, applyRecommended(), "no recommendations");
             });
  server_.on("/api/cal/char/save-baseline", HTTP_POST,
             [](AsyncWebServerRequest* req) {
               sendOk(req, steerChar.saveAsBaseline(),
                      "current characterization is not OK");
             });
  server_.on("/api/cal/char/json", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    doc["firmware"] = VCM_FW_VERSION;
    doc["git"] = VCM_GIT_COMMIT;
    doc["schema"] = ConfigRegistry::SCHEMA_VERSION;
    fillChar(doc["characterization"].to<JsonObject>(), steerChar.result());
    fillChar(doc["baseline"].to<JsonObject>(), steerChar.baseline());
    fillRecs(doc["recommended"].to<JsonObject>(), steerChar.recommended());
    const SteeringCalData& sc = calibration.steeringCal();
    JsonObject cal = doc["position_cal"].to<JsonObject>();
    cal["valid"] = sc.valid;
    cal["left_adc"] = sc.leftAdc;
    cal["right_adc"] = sc.rightAdc;
    cal["center_adc"] = sc.centerAdc;
    cal["fw_version"] = sc.fwVersion;
    String out;
    serializeJsonPretty(doc, out);
    AsyncWebServerResponse* res =
        req->beginResponse(200, "application/json", out);
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"steering_characterization.json\"");
    req->send(res);
  });
  server_.on("/api/cal/char/csv", HTTP_GET, [](AsyncWebServerRequest* req) {
    const SteeringCharacterization& c = steerChar.result();
    String out;
    out.reserve(1024);
    out += "pwm,vel_left,vel_right\n";
    for (uint8_t i = 0; i < c.nPoints; ++i) {
      out += String(c.pwm[i], 2);
      out += ',';
      out += String(c.velLeft[i], 4);
      out += ',';
      out += String(c.velRight[i], 4);
      out += '\n';
    }
    AsyncWebServerResponse* res = req->beginResponse(200, "text/csv", out);
    res->addHeader("Content-Disposition",
                   "attachment; filename=\"steering_pwm_velocity.csv\"");
    req->send(res);
  });
  server_.on("/api/cal/char/history", HTTP_GET, [](AsyncWebServerRequest* req) {
    JsonDocument doc;
    JsonArray arr = doc["records"].to<JsonArray>();
    for (uint8_t i = 0; i < steerChar.historyCount(); ++i) {
      SteeringCharacterization h;
      if (!steerChar.loadHistorySlot(i, h)) continue;
      JsonObject o = arr.add<JsonObject>();
      o["index"] = i;
      fillChar(o, h);
    }
    sendJson(req, doc);
  });

  server_.on("/api/steer/export/metadata.json", HTTP_GET,
             [](AsyncWebServerRequest* req) {
               JsonDocument doc;
               doc["firmware"] = VCM_FW_VERSION;
               doc["git"] = VCM_GIT_COMMIT;
               doc["build_date"] = VCM_BUILD_DATE;
               doc["schema"] = ConfigRegistry::SCHEMA_VERSION;
               doc["vehicle"] = config.s(S_VEHICLE_NAME);
               const SteeringCalData& sc = calibration.steeringCal();
               doc["cal_fw"] = sc.fwVersion;
               doc["cal_valid"] = sc.valid;
               sendJson(req, doc);
             });
}

}  // namespace vcm
