#include "Calibration.h"
#include "CalibrationStore.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
struct FixtureState {
  int attitudeSaves = 0;
  int gyroSaves = 0;
  int servoSaves = 0;
  AttitudeBiasDegrees attitudeWritten{};
  GyroBiasRawCounts gyroWritten{};
  ServoTrimChangeMask servoMask{};
};

FixtureState fixture;
int gyroCalls = 0;

void require(bool condition, const char* message) {
  if (!condition) {
    std::fprintf(stderr, "%s\n", message);
    std::exit(1);
  }
}

bool near(float actual, float expected) {
  return std::fabs(actual - expected) < 0.0001f;
}

void calibrateGyro() { ++gyroCalls; }

CalibrationResult update(Calibration& calibration, int request,
                         float roll, float pitch, CalibrationStore& store,
                         float& attitudeRoll, float& attitudePitch,
                         float& gyroX, float& gyroY, float& gyroZ,
                         float& servo1, float& servo2, float& servo3,
                         float& servo4) {
  return calibration.update(
      request, {roll, pitch}, store, {attitudeRoll, attitudePitch},
      {gyroX, gyroY, gyroZ}, {servo1, servo2, servo3, servo4}, calibrateGyro);
}

void gyroDispatchSavesReadbackAndClears() {
  Calibration calibration;
  CalibrationStore store;
  fixture = {};
  float roll = 0.0f, pitch = 0.0f;
  float gyroX = 11.0f, gyroY = -12.0f, gyroZ = 13.0f;
  float servo1 = 1.0f, servo2 = 2.0f, servo3 = 3.0f, servo4 = 4.0f;
  gyroCalls = 0;

  const CalibrationResult result = update(calibration, 1, 20.0f, 30.0f, store,
      roll, pitch, gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);

  require(gyroCalls == 1, "gyro request did not invoke the driver callback once");
  require(fixture.gyroSaves == 1 && near(fixture.gyroWritten.xRawCounts, 11.0f) &&
          near(fixture.gyroWritten.yRawCounts, -12.0f) &&
          near(fixture.gyroWritten.zRawCounts, 13.0f),
          "gyro request did not save all three raw bias values");
  require(near(gyroX, 12.0f) && near(gyroY, -10.0f) && near(gyroZ, 16.0f),
          "gyro readback was not applied to the active values");
  require(result.completion == CalibrationCompletion::Gyroscope && result.clearRequest,
          "gyro request did not report completion and request clearing");
  require(fixture.attitudeSaves == 0 && fixture.servoSaves == 0,
          "gyro request wrote an unrelated calibration group");
}

void attitudeSamplingSurvivesInterruptionAndResetsAtOneHundred() {
  Calibration calibration;
  CalibrationStore store;
  fixture = {};
  float roll = -8.0f, pitch = 9.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
  float servo1 = 0.0f, servo2 = 0.0f, servo3 = 0.0f, servo4 = 0.0f;

  for (int call = 0; call < 50; ++call) {
    const CalibrationResult result = update(calibration, 2, 1.0f, 3.0f, store,
        roll, pitch, gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
    require(!result.clearRequest && result.completion == CalibrationCompletion::None,
            "attitude request completed before call 100");
  }
  update(calibration, 0, 500.0f, 500.0f, store, roll, pitch,
         gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  update(calibration, 1, 500.0f, 500.0f, store, roll, pitch,
         gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.attitudeSaves == 0, "an interruption persisted partial attitude data");

  for (int call = 0; call < 49; ++call) {
    const CalibrationResult result = update(calibration, 2, 5.0f, -1.0f, store,
        roll, pitch, gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
    require(!result.clearRequest, "attitude request completed before its 100th sample");
  }
  require(fixture.attitudeSaves == 0, "attitude values persisted after only 99 samples");

  const CalibrationResult result = update(calibration, 2, 5.0f, -1.0f, store,
      roll, pitch, gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.attitudeSaves == 1 && near(fixture.attitudeWritten.rollDegrees, 3.0f) &&
          near(fixture.attitudeWritten.pitchDegrees, 1.0f),
          "the 100th call did not average all samples in accumulation order");
  require(near(roll, 4.0f) && near(pitch, 0.0f),
          "attitude readback was not applied to the active values");
  require(result.completion == CalibrationCompletion::Attitude && result.clearRequest,
          "100th attitude call did not report completion and request clearing");

  for (int call = 0; call < 99; ++call) {
    update(calibration, 2, 10.0f, 20.0f, store, roll, pitch,
           gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  }
  require(fixture.attitudeSaves == 1, "attitude accumulator did not reset after completion");
  update(calibration, 2, 10.0f, 20.0f, store, roll, pitch,
         gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.attitudeSaves == 2 && near(fixture.attitudeWritten.rollDegrees, 10.0f) &&
          near(fixture.attitudeWritten.pitchDegrees, 20.0f),
          "post-completion samples included values from the prior calibration");
}

void servoBaselineStartsOnFirstCallAndOnlyChangedChannelsAreReported() {
  Calibration calibration;
  CalibrationStore store;
  fixture = {};
  float roll = 0.0f, pitch = 0.0f;
  float gyroX = 0.0f, gyroY = 0.0f, gyroZ = 0.0f;
  float servo1 = 1.0f, servo2 = 2.0f, servo3 = 3.0f, servo4 = 4.0f;

  update(calibration, 0, 0.0f, 0.0f, store, roll, pitch,
         gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  servo1 = 10.0f;
  servo3 = 30.0f;
  CalibrationResult result = update(calibration, 3, 0.0f, 0.0f, store, roll, pitch,
      gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.servoSaves == 1 && fixture.servoMask.servo1Changed &&
          !fixture.servoMask.servo2Changed && fixture.servoMask.servo3Changed &&
          !fixture.servoMask.servo4Changed,
          "servo update did not restrict persistence to changed channels");
  require(result.changedServos.servo1Changed && result.changedServos.servo3Changed &&
          !result.clearRequest && result.completion == CalibrationCompletion::None,
          "servo mode did not report changes while remaining selected");

  result = update(calibration, 3, 0.0f, 0.0f, store, roll, pitch,
      gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.servoSaves == 2 && !result.changedServos.servo1Changed &&
          !result.changedServos.servo3Changed,
          "unchanged servo trims were marked for persistence");

  update(calibration, 0, 0.0f, 0.0f, store, roll, pitch,
         gyroX, gyroY, gyroZ, servo1, servo2, servo3, servo4);
  require(fixture.servoSaves == 2, "mode zero performed servo persistence");
}
}

AttitudeBiasDegrees CalibrationStore::saveAttitude(AttitudeBiasDegrees requested) {
  ++fixture.attitudeSaves;
  fixture.attitudeWritten = requested;
  return {requested.rollDegrees + 1.0f, requested.pitchDegrees - 1.0f};
}

GyroBiasRawCounts CalibrationStore::saveGyro(GyroBiasRawCounts requested) {
  ++fixture.gyroSaves;
  fixture.gyroWritten = requested;
  return {requested.xRawCounts + 1.0f, requested.yRawCounts + 2.0f,
          requested.zRawCounts + 3.0f};
}

ServoTrimsDegrees CalibrationStore::saveChangedServos(
    ServoTrimsDegrees requested, ServoTrimChangeMask changed) {
  ++fixture.servoSaves;
  fixture.servoMask = changed;
  return requested;
}

int main() {
  gyroDispatchSavesReadbackAndClears();
  attitudeSamplingSurvivesInterruptionAndResetsAtOneHundred();
  servoBaselineStartsOnFirstCallAndOnlyChangedChannelsAreReported();
  std::puts("calibration dispatch, accumulation, readback, and servo lifecycle verified");
}
