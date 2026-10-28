#ifndef SETTINGS_GROUPS_H
#define SETTINGS_GROUPS_H

#include <cmath>

#include "BalanceController.h"
#include "Calibration.h"
#include "ServoLimits.h"

// The rules that decide whether a candidate replaces a settings group's running struct. The sketch owns the running
// structs and wraps these in its group types (signals.def.h names them). A refused candidate leaves the running values
// unchanged, so the balance controller never sees a configuration that it would latch as InvalidConfig.

inline bool ApplyBalanceSettings(Config &running, const Config &candidate, const char **error) {
  if (!configValid(candidate)) {
    *error = "out of range or breaks a cross-field rule";
    return false;
  }
  running = candidate;
  return true;
}

inline bool ServoTrimInRange(float degrees) {
  return std::isfinite(degrees) && std::fabs(degrees) <= kServoAngleLimitDeg;
}

// Checks only the trims that the candidate changes. A trim it leaves alone is not checked, so a value stored before the
// limit applied does not block a change to another servo.
inline bool ApplyServoTrims(ServoTrimsDegrees &running, const ServoTrimsDegrees &candidate, const char **error) {
  const float runningDegrees[4] = {running.servo1Degrees, running.servo2Degrees, running.servo3Degrees,
                                   running.servo4Degrees};
  const float candidateDegrees[4] = {candidate.servo1Degrees, candidate.servo2Degrees, candidate.servo3Degrees,
                                     candidate.servo4Degrees};
  for (int i = 0; i < 4; ++i) {
    if (candidateDegrees[i] != runningDegrees[i] && !ServoTrimInRange(candidateDegrees[i])) {
      *error = "outside plus or minus 90 degrees";
      return false;
    }
  }
  running = candidate;
  return true;
}

#endif
