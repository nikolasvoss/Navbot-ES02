#ifndef SETTINGS_GROUPS_H
#define SETTINGS_GROUPS_H

#include <cmath>

#include "BalanceController.h"
#include "Calibration.h"

// The rules that decide whether a candidate replaces a settings group's running struct. The sketch owns the running
// structs and wraps these in its group types (signals.def.h names them). A refused candidate leaves the running values
// unchanged, so the balance controller never sees a configuration that it would latch as InvalidConfig.

// Proposed limit for a servo trim, in degrees. Uncommissioned.
constexpr float kServoTrimLimitDeg = 10.0f;

inline bool ApplyBalanceSettings(Config &running, const Config &candidate, const char **error) {
  if (!configValid(candidate)) {
    *error = "out of range or breaks a cross-field rule";
    return false;
  }
  running = candidate;
  return true;
}

inline bool ServoTrimInRange(float degrees) { return std::isfinite(degrees) && std::fabs(degrees) <= kServoTrimLimitDeg; }

inline bool ApplyServoTrims(ServoTrimsDegrees &running, const ServoTrimsDegrees &candidate, const char **error) {
  if (!ServoTrimInRange(candidate.servo1Degrees) || !ServoTrimInRange(candidate.servo2Degrees) ||
      !ServoTrimInRange(candidate.servo3Degrees) || !ServoTrimInRange(candidate.servo4Degrees)) {
    *error = "outside plus or minus 10 degrees";
    return false;
  }
  running = candidate;
  return true;
}

#endif
