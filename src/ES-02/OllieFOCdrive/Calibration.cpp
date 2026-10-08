#include "Calibration.h"

#include "CalibrationStore.h"

namespace {
constexpr int kAttitudeSampleCount = 100;

ServoTrimsDegrees readServos(ServoTrimDegreesRefs servos) {
  return {servos.servo1Degrees, servos.servo2Degrees,
          servos.servo3Degrees, servos.servo4Degrees};
}

ServoTrimChangeMask changedServos(ServoTrimsDegrees current,
                                  ServoTrimsDegrees previous) {
  return {current.servo1Degrees != previous.servo1Degrees,
          current.servo2Degrees != previous.servo2Degrees,
          current.servo3Degrees != previous.servo3Degrees,
          current.servo4Degrees != previous.servo4Degrees};
}

bool anyChanged(ServoTrimChangeMask changed) {
  return changed.servo1Changed || changed.servo2Changed ||
         changed.servo3Changed || changed.servo4Changed;
}

void applyServos(ServoTrimDegreesRefs target, ServoTrimsDegrees values) {
  target.servo1Degrees = values.servo1Degrees;
  target.servo2Degrees = values.servo2Degrees;
  target.servo3Degrees = values.servo3Degrees;
  target.servo4Degrees = values.servo4Degrees;
}
}

CalibrationResult Calibration::update(
    int request,
    AttitudeObservationDegrees observation,
    CalibrationStore& store,
    AttitudeBiasRefs attitudeBias,
    GyroBiasRawCountRefs gyroBias,
    ServoTrimDegreesRefs servoTrims,
    void (*calibrateGyro)()) {
  CalibrationResult result{CalibrationCompletion::None, false,
                           {false, false, false, false}};

  const ServoTrimsDegrees currentServos = readServos(servoTrims);
  if (!servoBaselineInitialized_) {
    lastSeenServos_ = currentServos;
    servoBaselineInitialized_ = true;
  }

  switch (request) {
    case 1: {
      calibrateGyro();
      const GyroBiasRawCounts readback = store.saveGyro(
          {gyroBias.xRawCounts, gyroBias.yRawCounts, gyroBias.zRawCounts});
      gyroBias.xRawCounts = readback.xRawCounts;
      gyroBias.yRawCounts = readback.yRawCounts;
      gyroBias.zRawCounts = readback.zRawCounts;
      result.completion = CalibrationCompletion::Gyroscope;
      result.clearRequest = true;
      break;
    }
    case 2: {
      rollSum_ += observation.rollDegrees;
      pitchSum_ += observation.pitchDegrees;
      ++imuCallCount_;
      if (imuCallCount_ >= kAttitudeSampleCount) {
        const AttitudeBiasDegrees readback = store.saveAttitude(
            {rollSum_ / kAttitudeSampleCount,
             pitchSum_ / kAttitudeSampleCount});
        attitudeBias.rollDegrees = readback.rollDegrees;
        attitudeBias.pitchDegrees = readback.pitchDegrees;
        rollSum_ = 0.0f;
        pitchSum_ = 0.0f;
        imuCallCount_ = 0;
        result.completion = CalibrationCompletion::Attitude;
        result.clearRequest = true;
      }
      break;
    }
    case 3: {
      result.changedServos = changedServos(currentServos, lastSeenServos_);
      if (anyChanged(result.changedServos)) {
        lastSeenServos_ = currentServos;
        applyServos(servoTrims,
                    store.saveChangedServos(currentServos, result.changedServos));
      } else {
        store.saveChangedServos(currentServos, result.changedServos);
      }
      break;
    }
    default:
      break;
  }

  return result;
}
