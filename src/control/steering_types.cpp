#include "control/steering_types.h"

namespace vcm {

const char* steerControlStateName(SteerControlState s) {
  switch (s) {
    case SteerControlState::IDLE: return "IDLE";
    case SteerControlState::MOVING_LEFT: return "MOVING_LEFT";
    case SteerControlState::MOVING_RIGHT: return "MOVING_RIGHT";
    case SteerControlState::APPROACHING: return "APPROACHING";
    case SteerControlState::SETTLING: return "SETTLING";
    case SteerControlState::HOLDING: return "HOLDING";
    case SteerControlState::DEADBAND: return "DEADBAND";
    case SteerControlState::LIMIT: return "LIMIT";
    case SteerControlState::FAULT: return "FAULT";
    case SteerControlState::CALIBRATION: return "CALIBRATION";
  }
  return "?";
}

const char* steerDiagEventName(SteerDiagEvent e) {
  switch (e) {
    case SteerDiagEvent::CAL_START: return "Calibration Started";
    case SteerDiagEvent::CAL_FINISH: return "Calibration Finished";
    case SteerDiagEvent::DIR_CHANGE: return "Direction Changed";
    case SteerDiagEvent::LIMIT_DETECTED: return "Limit Detected";
    case SteerDiagEvent::PWM_LIMIT: return "PWM Limit Reached";
    case SteerDiagEvent::FAULT: return "Fault";
    case SteerDiagEvent::MODE_CHANGE: return "Control Mode Changed";
    case SteerDiagEvent::CHAR_STEP: return "Smart Characterization Step";
    case SteerDiagEvent::PWM_TEST: return "PWM Test";
    case SteerDiagEvent::CHAR_FAIL: return "Characterization Failed";
    case SteerDiagEvent::CHAR_DONE: return "Characterization Finished";
  }
  return "?";
}

}  // namespace vcm
