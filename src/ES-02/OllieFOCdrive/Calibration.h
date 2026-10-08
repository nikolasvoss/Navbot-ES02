#pragma once

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
  float& rollDegrees;
  float& pitchDegrees;
};

struct GyroBiasRawCountRefs {
  float& xRawCounts;
  float& yRawCounts;
  float& zRawCounts;
};

struct ServoTrimDegreesRefs {
  float& servo1Degrees;
  float& servo2Degrees;
  float& servo3Degrees;
  float& servo4Degrees;
};

struct ServoTrimChangeMask {
  bool servo1Changed;
  bool servo2Changed;
  bool servo3Changed;
  bool servo4Changed;
};

enum class CalibrationCompletion {
  None,
  Gyroscope,
  Attitude
};

struct CalibrationResult {
  CalibrationCompletion completion;
  bool clearRequest;
  ServoTrimChangeMask changedServos;
};

class CalibrationStore;

class Calibration {
public:
  CalibrationResult update(
      int request,
      AttitudeObservationDegrees observation,
      CalibrationStore& store,
      AttitudeBiasRefs attitudeBias,
      GyroBiasRawCountRefs gyroBias,
      ServoTrimDegreesRefs servoTrims,
      void (*calibrateGyro)());

private:
  float rollSum_ = 0.0f;
  float pitchSum_ = 0.0f;
  int imuCallCount_ = 0;
  bool servoBaselineInitialized_ = false;
  ServoTrimsDegrees lastSeenServos_;
};
