#include "CalibrationStore.h"

#include <Preferences.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

void requireSessionClosed() {
  require(Preferences::beginCount == Preferences::endCount,
          "every opened store session must end");
}

void testLoadPopulatedAndMissing() {
  Preferences::reset();
  Preferences::values = {{"roll", 1.25f},
                         {"pitch", -2.5f},
                         {"gyroX", 3.0f},
                         {"gyroY", 4.0f},
                         {"gyroZ", 5.0f},
                         {"servoAngle1", 6.0f},
                         {"servoAngle2", 7.0f},
                         {"servoAngle3", 8.0f},
                         {"servoAngle4", 9.0f}};

  const PersistedCalibrationValues loaded = CalibrationStore().load();
  require(loaded.attitude.rollDegrees == 1.25f &&
              loaded.attitude.pitchDegrees == -2.5f,
          "load returns persisted attitude fields");
  require(loaded.gyro.xRawCounts == 3.0f && loaded.gyro.yRawCounts == 4.0f &&
              loaded.gyro.zRawCounts == 5.0f,
          "load returns persisted gyro fields");
  require(loaded.servos.servo1Degrees == 6.0f &&
              loaded.servos.servo2Degrees == 7.0f &&
              loaded.servos.servo3Degrees == 8.0f &&
              loaded.servos.servo4Degrees == 9.0f,
          "load returns persisted servo fields");
  require(Preferences::readKeys.size() == 9,
          "load reads all nine named float keys");
  require(Preferences::namespaces == std::vector<std::string>{"preferences"},
          "load uses the preferences namespace");
  requireSessionClosed();

  Preferences::reset();
  const PersistedCalibrationValues missing = CalibrationStore().load();
  require(missing.attitude.rollDegrees == 0.0f &&
              missing.attitude.pitchDegrees == 0.0f &&
              missing.gyro.xRawCounts == 0.0f &&
              missing.gyro.yRawCounts == 0.0f &&
              missing.gyro.zRawCounts == 0.0f &&
              missing.servos.servo1Degrees == 0.0f &&
              missing.servos.servo2Degrees == 0.0f &&
              missing.servos.servo3Degrees == 0.0f &&
              missing.servos.servo4Degrees == 0.0f,
          "missing keys default to zero");
  requireSessionClosed();
}

void testAttitudeAndGyroSubsets() {
  Preferences::reset();
  Preferences::values = {{"gyroX", 13.0f}, {"servoAngle2", 22.0f}};

  const AttitudeBiasDegrees attitude =
      CalibrationStore().saveAttitude({1.5f, -2.5f});
  require(attitude.rollDegrees == 1.5f && attitude.pitchDegrees == -2.5f,
          "attitude save returns readback values");
  require(Preferences::writeKeys == std::vector<std::string>({"roll", "pitch"}),
          "attitude save writes only roll and pitch");
  require(Preferences::values.at("gyroX") == 13.0f &&
              Preferences::values.at("servoAngle2") == 22.0f,
          "attitude save preserves gyro and servo keys");
  requireSessionClosed();

  Preferences::reset();
  Preferences::values = {{"roll", 11.0f}, {"servoAngle2", 22.0f}};
  const GyroBiasRawCounts gyro =
      CalibrationStore().saveGyro({3.0f, 4.0f, 5.0f});
  require(gyro.xRawCounts == 3.0f && gyro.yRawCounts == 4.0f &&
              gyro.zRawCounts == 5.0f,
          "gyro save returns readback values");
  require(Preferences::writeKeys ==
              std::vector<std::string>({"gyroX", "gyroY", "gyroZ"}),
          "gyro save writes only gyro keys");
  require(Preferences::values.at("roll") == 11.0f &&
              Preferences::values.at("servoAngle2") == 22.0f,
          "gyro save preserves attitude and servo keys");
  requireSessionClosed();
}

void testChangedServoSubsetAndZeroChanges() {
  Preferences::reset();
  Preferences::values = {{"servoAngle1", 10.0f},
                         {"servoAngle2", 20.0f},
                         {"roll", 1.0f}};
  const ServoTrimsDegrees saved = CalibrationStore().saveChangedServos(
      {11.0f, 22.0f, 33.0f, 44.0f}, {false, true, false, true});
  require(saved.servo1Degrees == 11.0f && saved.servo2Degrees == 22.0f &&
              saved.servo3Degrees == 33.0f && saved.servo4Degrees == 44.0f,
          "servo save returns readback for changed fields and requested values otherwise");
  require(Preferences::writeKeys ==
              std::vector<std::string>({"servoAngle2", "servoAngle4"}),
          "servo save writes only changed channels");
  require(Preferences::readKeys ==
              std::vector<std::string>({"servoAngle2", "servoAngle4"}),
          "servo save reads back only changed channels");
  require(Preferences::values.at("servoAngle1") == 10.0f &&
              Preferences::values.at("roll") == 1.0f,
          "servo save preserves unchanged channels and other groups");
  requireSessionClosed();

  Preferences::reset();
  CalibrationStore().saveChangedServos(
      {1.0f, 2.0f, 3.0f, 4.0f}, {false, false, false, false});
  require(Preferences::writeKeys.empty() && Preferences::readKeys.empty(),
          "no changed servo causes no key reads or writes");
  requireSessionClosed();
}

void testFailedOpenWriteAndRead() {
  Preferences::reset();
  Preferences::failBegin = true;
  const AttitudeBiasDegrees unopened =
      CalibrationStore().saveAttitude({7.0f, 8.0f});
  require(unopened.rollDegrees == 0.0f && unopened.pitchDegrees == 0.0f,
          "failed open preserves unchecked zero-default readback");
  require(Preferences::writeKeys == std::vector<std::string>({"roll", "pitch"}) &&
              Preferences::readKeys == std::vector<std::string>({"roll", "pitch"}),
          "failed open still attempts writes and reads");
  requireSessionClosed();

  Preferences::reset();
  Preferences::failedWrites.insert("pitch");
  const AttitudeBiasDegrees partial =
      CalibrationStore().saveAttitude({7.0f, 8.0f});
  require(partial.rollDegrees == 7.0f && partial.pitchDegrees == 0.0f,
          "failed write remains visible through zero-default readback");
  require(Preferences::values.count("pitch") == 0,
          "failed write does not create the key in the fake store");
  requireSessionClosed();

  Preferences::reset();
  Preferences::values = {{"roll", 9.0f}, {"pitch", 10.0f}};
  Preferences::failedReads.insert("roll");
  const AttitudeBiasDegrees unreadable =
      CalibrationStore().saveAttitude({7.0f, 8.0f});
  require(unreadable.rollDegrees == 0.0f && unreadable.pitchDegrees == 8.0f,
          "failed read uses the existing zero default");
  require(Preferences::values.at("roll") == 7.0f,
          "failed read does not change the successful write");
  requireSessionClosed();
}

}  // namespace

int main() {
  testLoadPopulatedAndMissing();
  testAttitudeAndGyroSubsets();
  testChangedServoSubsetAndZeroChanges();
  testFailedOpenWriteAndRead();
  std::cout << "calibration store host checks passed\n";
}
