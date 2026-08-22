#include "services/usb_console.h"

#include <Arduino.h>
#include <string.h>

#include "control/steering_types.h"
#include "config/config_registry.h"
#include "core/types.h"
#include "drivers/i2c/i2c_bus.h"
#include "services/calibration.h"
#include "services/commissioning.h"
#include "services/logger.h"
#include "services/safety.h"
#include "services/steering_characterization.h"
#include "services/telemetry.h"

namespace vcm {

static char line_[128];
static uint8_t len_ = 0;

static void i2cScan() {
  i2cBus.requestScan();
  LOGI("I2C", "scan queued (SDA=GPIO%d SCL=GPIO%d %s)", i2cBus.sdaPin(),
       i2cBus.sclPin(), i2cBus.busStateName());
}

static void dumpTel() {
  const VehicleTelemetry t = telemetry.snapshot();
  LOGI("TEL", "state=%s cal=%d fb raw=%u filt=%.1f pct=%.1f pwm=%.1f",
       vehicleStateName(safety.state()), t.steering.calibrated ? 1 : 0,
       t.steering.feedbackRaw, t.steering.feedbackFiltered,
       t.steering.actualPct, t.steering.pwmPct);
  LOGI("TEL", "ina=%s bus=%.2fV L=%.2f R=%.2f S=%.2fA  imu=%s p=%.1f r=%.1f",
       sensorHealthName(t.power.inaHealth), t.power.busVoltage,
       t.power.leftCurrentA, t.power.rightCurrentA, t.power.steeringCurrentA,
       sensorHealthName(t.imu.health), t.imu.pitchDeg, t.imu.rollDeg);
}

static void handle(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;
  if (!strncmp(line, "cmd ", 4)) line += 4;

  char* tok = strtok(line, " \t");
  if (!tok) return;

  if (!strcmp(tok, "help")) {
    LOGI("CMD", "help | i2c | recover | status | tel | faults | pwm <pct>|off");
    LOGI("CMD", "steer <pct>|off | cal start [pwm]|abort|status");
    LOGI("CMD", "char start|abort|apply|save | cfg KEY VAL | save | invert | commission");
    return;
  }
  if (!strcmp(tok, "i2c")) {
    i2cScan();
    return;
  }
  if (!strcmp(tok, "recover")) {
    i2cBus.requestRecover();
    LOGI("I2C", "recovery queued");
    return;
  }
  if (!strcmp(tok, "status")) {
    refreshCommissioning(commissioning);
    logCommissioningTable(commissioning);
    return;
  }
  if (!strcmp(tok, "tel")) {
    dumpTel();
    return;
  }
  if (!strcmp(tok, "faults")) {
    LOGI("CMD", "state=%s mask=0x%04X", vehicleStateName(safety.state()),
         safety.activeFaultMask());
    return;
  }
  if (!strcmp(tok, "cal")) {
    const char* sub = strtok(nullptr, " \t");
    if (sub && !strcmp(sub, "start")) {
      SteeringCalParams p;
      const char* pwm = strtok(nullptr, " \t");
      if (pwm) p.pwmPct = atof(pwm);
      const bool ok = calibration.startSteeringCal(p);
      LOGI("CMD", "cal start %s pwm=%.0f", ok ? "ok" : "FAIL", p.pwmPct);
    } else if (sub && !strcmp(sub, "abort")) {
      calibration.abortSteeringCal();
    } else {
      const SteeringCalData& sc = calibration.steeringCal();
      LOGI("CMD", "cal state=%d valid=%d L=%.0f C=%.0f R=%.0f",
           (int)calibration.steeringCalState(), sc.valid ? 1 : 0, sc.leftAdc,
           sc.centerAdc, sc.rightAdc);
    }
    return;
  }
  if (!strcmp(tok, "char")) {
    const char* sub = strtok(nullptr, " \t");
    if (sub && !strcmp(sub, "start")) {
      const bool ok = steerChar.start(true);
      LOGI("CMD", "char start %s", ok ? "ok" : "FAIL");
    } else if (sub && !strcmp(sub, "abort")) {
      steerChar.abort("usb abort");
    } else if (sub && !strcmp(sub, "apply")) {
      const SteerRecommendations& r = steerChar.recommended();
      if (!r.valid) {
        LOGW("CMD", "no recommendations");
        return;
      }
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
      LOGI("CMD", "applied start L/R %.1f/%.1f hold %.1f/%.1f kp=%.2f",
           r.startLeft, r.startRight, r.holdLeft, r.holdRight, r.kp);
    } else if (sub && !strcmp(sub, "save")) {
      LOGI("CMD", "baseline %s",
           steerChar.saveAsBaseline() ? "saved" : "FAIL");
    } else {
      LOGI("CMD", "char %s %s",
           SteeringCharacterizationService::phaseName(steerChar.live().phase),
           steerChar.live().message);
    }
    return;
  }
  if (!strcmp(tok, "pwm")) {
    const char* sub = strtok(nullptr, " \t");
    if (!sub || !strcmp(sub, "off") || !strcmp(sub, "0")) {
      calibration.setOpenLoopSteer(0, false);
    } else {
      calibration.setOpenLoopSteer(atof(sub), true);
    }
    return;
  }
  if (!strcmp(tok, "steer")) {
    const char* sub = strtok(nullptr, " \t");
    if (!sub || !strcmp(sub, "off")) {
      calibration.setManualSteeringTarget(50.0f, false);
      LOGI("CMD", "steer off");
    } else {
      calibration.setManualSteeringTarget(atof(sub), true);
      LOGI("CMD", "steer target %.1f", atof(sub));
    }
    return;
  }
  if (!strcmp(tok, "invert")) {
    const float next = config.b(STR_INVERT) ? 0.0f : 1.0f;
    config.applyByKey("steering.invert_output", next);
    LOGI("CMD", "invert=%d", (int)next);
    return;
  }
  if (!strcmp(tok, "cfg")) {
    const char* key = strtok(nullptr, " \t");
    const char* val = strtok(nullptr, " \t");
    if (!key || !val) {
      LOGW("CMD", "cfg KEY VALUE");
      return;
    }
    const bool ok = config.applyByKey(key, atof(val)) ||
                    config.applyStringByKey(key, val);
    LOGI("CMD", "cfg %s %s", key, ok ? "ok" : "FAIL");
    return;
  }
  if (!strcmp(tok, "save")) {
    config.save();
    return;
  }
  if (!strcmp(tok, "commission")) {
    if (!calibration.steeringCal().valid) {
      LOGW("CMD", "need steering cal first");
      return;
    }
    calibration.setCommissioned(true);
    return;
  }
  LOGW("CMD", "unknown '%s' (help)", tok);
}

static void feed(Stream& s) {
  while (s.available()) {
    const char c = (char)s.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line_[len_] = 0;
      if (len_) handle(line_);
      len_ = 0;
      continue;
    }
    if (len_ + 1 < sizeof(line_)) line_[len_++] = (char)c;
  }
}

void pollUsbConsole() {
  feed(Serial);
  feed(Serial0);
}

}  // namespace vcm
