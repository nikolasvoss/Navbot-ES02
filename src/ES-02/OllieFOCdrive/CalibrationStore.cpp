#include "CalibrationStore.h"

#include <Preferences.h>

namespace {

constexpr char kPreferencesNamespace[] = "preferences";
constexpr char kRollKey[] = "roll";
constexpr char kPitchKey[] = "pitch";
constexpr char kGyroXKey[] = "gyroX";
constexpr char kGyroYKey[] = "gyroY";
constexpr char kGyroZKey[] = "gyroZ";
constexpr char kServo1Key[] = "servoAngle1";
constexpr char kServo2Key[] = "servoAngle2";
constexpr char kServo3Key[] = "servoAngle3";
constexpr char kServo4Key[] = "servoAngle4";

Preferences preferences;

PersistedCalibrationValues readAll() {
  return {
      {preferences.getFloat(kRollKey, 0.0f),
       preferences.getFloat(kPitchKey, 0.0f)},
      {preferences.getFloat(kGyroXKey, 0.0f),
       preferences.getFloat(kGyroYKey, 0.0f),
       preferences.getFloat(kGyroZKey, 0.0f)},
      {preferences.getFloat(kServo1Key, 0.0f),
       preferences.getFloat(kServo2Key, 0.0f),
       preferences.getFloat(kServo3Key, 0.0f),
       preferences.getFloat(kServo4Key, 0.0f)}};
}

}

PersistedCalibrationValues CalibrationStore::load() {
  preferences.begin(kPreferencesNamespace, false);
  const PersistedCalibrationValues values = readAll();
  preferences.end();
  return values;
}

AttitudeBiasDegrees CalibrationStore::saveAttitude(
    AttitudeBiasDegrees requested) {
  preferences.begin(kPreferencesNamespace, false);
  preferences.putFloat(kRollKey, requested.rollDegrees);
  preferences.putFloat(kPitchKey, requested.pitchDegrees);
  const AttitudeBiasDegrees saved = {
      preferences.getFloat(kRollKey, 0.0f),
      preferences.getFloat(kPitchKey, 0.0f)};
  preferences.end();
  return saved;
}

GyroBiasRawCounts CalibrationStore::saveGyro(GyroBiasRawCounts requested) {
  preferences.begin(kPreferencesNamespace, false);
  preferences.putFloat(kGyroXKey, requested.xRawCounts);
  preferences.putFloat(kGyroYKey, requested.yRawCounts);
  preferences.putFloat(kGyroZKey, requested.zRawCounts);
  const GyroBiasRawCounts saved = {
      preferences.getFloat(kGyroXKey, 0.0f),
      preferences.getFloat(kGyroYKey, 0.0f),
      preferences.getFloat(kGyroZKey, 0.0f)};
  preferences.end();
  return saved;
}

ServoTrimsDegrees CalibrationStore::saveChangedServos(
    ServoTrimsDegrees requested,
    ServoTrimChangeMask changed) {
  preferences.begin(kPreferencesNamespace, false);

  ServoTrimsDegrees saved = requested;
  if (changed.servo1Changed) {
    preferences.putFloat(kServo1Key, requested.servo1Degrees);
    saved.servo1Degrees = preferences.getFloat(kServo1Key, 0.0f);
  }
  if (changed.servo2Changed) {
    preferences.putFloat(kServo2Key, requested.servo2Degrees);
    saved.servo2Degrees = preferences.getFloat(kServo2Key, 0.0f);
  }
  if (changed.servo3Changed) {
    preferences.putFloat(kServo3Key, requested.servo3Degrees);
    saved.servo3Degrees = preferences.getFloat(kServo3Key, 0.0f);
  }
  if (changed.servo4Changed) {
    preferences.putFloat(kServo4Key, requested.servo4Degrees);
    saved.servo4Degrees = preferences.getFloat(kServo4Key, 0.0f);
  }

  preferences.end();
  return saved;
}
