#pragma once

#include <cmath>

struct AttitudeBiasDegrees {
  float rollDegrees;
  float pitchDegrees;
};

struct GyroBiasRawCounts {
  float xRawCounts;
  float yRawCounts;
  float zRawCounts;
};

struct ServoTrimsDegrees {
  float servo1Degrees;
  float servo2Degrees;
  float servo3Degrees;
  float servo4Degrees;
};

struct PersistedCalibrationValues {
  AttitudeBiasDegrees attitude;
  GyroBiasRawCounts gyro;
  ServoTrimsDegrees servos;
};

struct AttitudeObservationDegrees {
  float rollDegrees;
  float pitchDegrees;
};

struct AttitudeBiasRefs {
  float &rollDegrees;
  float &pitchDegrees;
};

struct GyroBiasRawCountRefs {
  float &xRawCounts;
  float &yRawCounts;
  float &zRawCounts;
};

struct ServoTrimDegreesRefs {
  float &servo1Degrees;
  float &servo2Degrees;
  float &servo3Degrees;
  float &servo4Degrees;
};

struct ServoTrimChangeMask {
  bool servo1Changed;
  bool servo2Changed;
  bool servo3Changed;
  bool servo4Changed;
};

inline bool anyServoChanged(ServoTrimChangeMask changed) {
  return changed.servo1Changed || changed.servo2Changed || changed.servo3Changed || changed.servo4Changed;
}

enum class CalibrationCompletion { None, Gyroscope, Attitude };

struct CalibrationResult {
  CalibrationCompletion completion;
  bool clearRequest;
  ServoTrimChangeMask changedServos;
};

// The calibrate action's refusal, or nullptr when the step may start. Step 0 ends the running step and is always allowed.
// Steps 1 to 3 wait until balance and motor tests are off: the gyroscope step blocks the loop.
inline const char *CalibrationStartRefusal(float step, bool balanceActive, bool motorTestActive) {
  if (!std::isfinite(step) || std::floor(step) != step) return "step must be a whole number";
  if (step == 0.0f) return nullptr;
  if (balanceActive) return "refused: balance active";
  if (motorTestActive) return "refused: motor test active";
  return nullptr;
}

class CalibrationStore;

class Calibration {
 public:
  CalibrationResult update(int request, AttitudeObservationDegrees observation, CalibrationStore &store,
                           AttitudeBiasRefs attitudeBias, GyroBiasRawCountRefs gyroBias,
                           ServoTrimDegreesRefs servoTrims, void (*calibrateGyro)(GyroBiasRawCountRefs));

 private:
  float rollSum_ = 0.0f;
  float pitchSum_ = 0.0f;
  int imuCallCount_ = 0;
  bool servoBaselineInitialized_ = false;
  ServoTrimsDegrees lastSeenServos_{};
};
