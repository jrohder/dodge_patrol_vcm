#include "control/control_arbiter.h"

#include "config/config_registry.h"
#include "services/logger.h"
#include "services/safety.h"

namespace vcm {

ControlArbiter arbiter;

void ControlArbiter::submitWebCommand(float throttle, float steering,
                                      bool brake, bool lights, bool siren) {
  webCmd_.throttle = constrain(throttle, -1.0f, 1.0f);
  webCmd_.steering = constrain(steering, -1.0f, 1.0f);
  webCmd_.brake = brake;
  webCmd_.lights = lights;
  webCmd_.siren = siren;
  webCmd_.source = ControlSource::WEB_REMOTE;
  webCmd_.timestampMs = millis();
  webCmd_.valid = true;
  lastWebMs_ = webCmd_.timestampMs;
}

void ControlArbiter::setWebControlRequested(bool requested) {
  if (webRequested_ != requested)
    LOGI("ARBIT", "Web control %s", requested ? "requested" : "released");
  webRequested_ = requested;
  if (requested) lastWebMs_ = millis();
}

bool ControlArbiter::rcAwayFromNeutral(float throttle, float steering) const {
  const float thr = config.f(CTL_OVERRIDE_THRESH) / 100.0f;
  return fabsf(throttle) > thr || fabsf(steering) > thr;
}

ControlCommand ControlArbiter::arbitrate(float rcThrottle, float rcSteering,
                                         bool rcValid, float manualThrottle,
                                         float manualSteering,
                                         bool manualActive) {
  ControlCommand cmd;
  cmd.timestampMs = millis();
  const uint32_t now = cmd.timestampMs;

  // Calibration/diagnostic states own the actuators through their own paths
  if (safety.state() == VehicleState::CALIBRATION) {
    cmd.source = ControlSource::CALIBRATION;
    active_ = cmd.source;
    return cmd;
  }
  if (safety.state() == VehicleState::DIAGNOSTIC) {
    cmd.source = ControlSource::DIAGNOSTIC;
    active_ = cmd.source;
    return cmd;
  }

  const bool rcEnabled = config.b(CTL_RC_ENABLED) && rcValid;
  const bool webEnabled = config.b(CTL_WEB_ENABLED) && webRequested_;
  const bool manualEnabled = config.b(CTL_MANUAL_ENABLED);

  // RC override: stick moved away from neutral claims (and holds) control
  if (rcEnabled && config.b(CTL_RC_OVERRIDE) &&
      rcAwayFromNeutral(rcThrottle, rcSteering)) {
    rcActiveUntil_ = now + (uint32_t)config.i(CTL_OVERRIDE_HOLD);
  }
  const bool rcOwns =
      rcEnabled && (rcActiveUntil_ > now ||
                    (!webEnabled && !(manualEnabled && manualActive)));

  // Web heartbeat health: if web owns the vehicle and the WebSocket control
  // heartbeat disappears, command a safe stop (WEB-001).
  const bool webAlive =
      webEnabled && (now - lastWebMs_) < (uint32_t)config.i(SAF_WEB_TIMEOUT);

  ControlSource next = ControlSource::NONE;
  if (rcOwns) {
    next = ControlSource::RC_REMOTE;
  } else if (webEnabled) {
    next = ControlSource::WEB_REMOTE;
  } else if (manualEnabled) {
    next = ControlSource::MANUAL_VEHICLE;
  }

  if (next != active_) {
    LOGI("ARBIT", "Control source: %s -> %s", controlSourceName(active_),
         controlSourceName(next));
    active_ = next;
  }

  switch (next) {
    case ControlSource::RC_REMOTE:
      cmd.throttle = rcThrottle;
      cmd.steering = rcSteering;
      cmd.valid = true;
      safety.clearFault(FLT_WEB_HEARTBEAT);
      break;
    case ControlSource::WEB_REMOTE:
      if (webAlive) {
        cmd = webCmd_;
        cmd.timestampMs = now;
        safety.clearFault(FLT_WEB_HEARTBEAT);
      } else {
        // heartbeat lost while owning: safe stop
        safety.raiseFault(FLT_WEB_HEARTBEAT);
        cmd.valid = true;  // valid zero command = active braking to stop
      }
      break;
    case ControlSource::MANUAL_VEHICLE:
      cmd.throttle = manualThrottle;
      cmd.steering = manualSteering;
      cmd.valid = manualActive;
      safety.clearFault(FLT_WEB_HEARTBEAT);
      break;
    default:
      safety.clearFault(FLT_WEB_HEARTBEAT);
      break;
  }
  cmd.source = next;
  return cmd;
}

}  // namespace vcm
