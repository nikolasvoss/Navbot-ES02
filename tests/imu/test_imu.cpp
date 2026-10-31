#include <Arduino.h>
#include "imu.h"
#include "ICM42688.h"
#include "Logging.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <deque>

HostSerial Serial;
namespace {
uint32_t nowUs = 0;
bool sampleReady = false;
ICM42688RawSample nextSample{};
uint16_t initializedRate = 0;
bool driverInitializationSucceeds = true;
std::deque<ICM42688RawSample> scriptedSamples;
bool streamSample = false;
}
namespace Logging {
void message(Level, const char*, const char*, ...) {}
}

uint32_t micros() { return nowUs; }
void delay(unsigned long milliseconds) { nowUs += milliseconds * 1000U; }
bool initICM42688(uint16_t sampleRateHz) {
  initializedRate = sampleRateHz;
  return driverInitializationSucceeds;
}
bool readIMUDataIfReady(ICM42688RawSample& sample) {
  if (!scriptedSamples.empty()) {
    sample = scriptedSamples.front();
    scriptedSamples.pop_front();
    return true;
  }
  if (streamSample) {
    sample = nextSample;
    return true;
  }
  if (!sampleReady) return false;
  sampleReady = false;
  sample = nextSample;
  return true;
}

static void acceptSample(uint32_t timeUs, ICM42688RawSample sample = {}) {
  nowUs = timeUs;
  nextSample = sample;
  sampleReady = true;
  assert(imu::service());
}

void verifyOnlyChangedCutoffResetsItsFilter() {
  struct Output { float gyro; float accel; };
  const ICM42688RawSample steady{30000, -20000, 12000, 10000, -15000, 5000, 0};
  const auto run = [&](bool changeGyro, bool changeAccel) {
    imu::Settings initial{};
    assert(imu::begin(initial, {}));
    for (uint32_t index = 1; index <= 40; ++index) acceptSample(index * 1000U, steady);
    auto changed = imu::settings();
    if (changeGyro) changed.gyroCutoffHz = 40;
    if (changeAccel) changed.accelCutoffHz = 40;
    assert(imu::setSettings(changed));
    acceptSample(41000U, steady);
    const auto& snapshot = imu::latest();
    return Output{snapshot.filteredAngularVelocityRadPerSec.x,
                  snapshot.filteredAccelerationG.x};
  };

  const Output unchanged = run(false, false);
  const Output gyroOnly = run(true, false);
  const Output both = run(true, true);
  assert(std::fabs(gyroOnly.gyro - unchanged.gyro) > 0.01f);
  assert(std::fabs(gyroOnly.accel - unchanged.accel) < 1e-6f);
  assert(std::fabs(both.gyro - gyroOnly.gyro) < 1e-6f);
  assert(std::fabs(both.accel - gyroOnly.accel) > 0.01f);
}

void verifyFailedBeginStopsAndClearsPreviousInstance() {
  assert(imu::begin(imu::Settings{}, {}));
  acceptSample(1000U);
  assert(imu::latest().valid);

  imu::Settings invalid{};
  invalid.gyroCutoffHz = 0;
  sampleReady = true;
  assert(!imu::begin(invalid, {}));
  assert(!imu::latest().valid);
  assert(imu::timingStatistics().acceptedSamples == 0);
  assert(!imu::service());
  assert(sampleReady);
  sampleReady = false;

  assert(imu::begin(imu::Settings{}, {}));
  acceptSample(2000U);
  assert(imu::latest().valid);
  driverInitializationSucceeds = false;
  sampleReady = true;
  assert(!imu::begin(imu::Settings{}, {}));
  driverInitializationSucceeds = true;
  assert(!imu::latest().valid);
  assert(imu::timingStatistics().acceptedSamples == 0);
  assert(!imu::service());
  assert(sampleReady);
}

void verifySetCalibrationRefreshesPublishedCorrectedAngles() {
  assert(imu::begin(imu::Settings{}, {}));
  acceptSample(1000U, {0, 0, 4096, 0, 0, 0, 0});
  const imu::Snapshot before = imu::latest();
  assert(before.valid);

  imu::CalibrationValues updated{};
  updated.attitude = {4.0f, -5.0f};
  imu::setCalibration(updated);

  const imu::Snapshot& after = imu::latest();
  assert(after.valid);
  assert(after.sequence == before.sequence);
  assert(after.readTimeUs == before.readTimeUs);
  assert(std::fabs(after.correctedRollDegrees - (-4.0f)) < 1e-5f);
  assert(std::fabs(after.correctedPitchDegrees - 5.0f) < 1e-5f);
}

void calibrateWith(ICM42688RawSample steady) {
  nextSample = steady;
  streamSample = true;
  imu::calibrateGyroBiasBlocking();
  streamSample = false;
}

void verifyGyroCalibrationAveragesLargeConstantOffset() {
  assert(imu::begin(imu::Settings{}, {}));
  calibrateWith({0, 0, 0, 40, -45, 49, 0});
  const imu::CalibrationValues result = imu::calibration();
  assert(result.gyro.xRawCounts == 40.0f);
  assert(result.gyro.yRawCounts == -45.0f);
  assert(result.gyro.zRawCounts == 49.0f);
}

void verifyGyroCalibrationRestartsAfterMotion() {
  assert(imu::begin(imu::Settings{}, {}));
  const ICM42688RawSample still{0, 0, 0, 0, 0, 0, 0};
  const ICM42688RawSample moved{0, 0, 0, 500, 0, 0, 0};
  for (int index = 0; index < 3; ++index) scriptedSamples.push_back(still);
  for (int index = 0; index < 4; ++index) scriptedSamples.push_back(moved);
  for (int index = 0; index < 2; ++index) scriptedSamples.push_back(still);
  scriptedSamples.push_back(moved);
  calibrateWith({0, 0, 0, 2, -2, 2, 0});
  const imu::CalibrationValues result = imu::calibration();
  assert(result.gyro.xRawCounts == 2.0f);
  assert(result.gyro.yRawCounts == -2.0f);
  assert(result.gyro.zRawCounts == 2.0f);
}

float gyroRateAfterBias(int16_t rawCount, float bias) {
  imu::CalibrationValues calibration{};
  calibration.gyro = {bias, 0.0f, 0.0f};
  assert(imu::begin(imu::Settings{}, calibration));
  acceptSample(1000U, {0, 0, 0, rawCount, 0, 0, 0});
  return imu::latest().angularVelocityRadPerSec.x;
}

void verifyBiasCorrectionKeepsFractionAndSaturates() {
  const float tolerance = 1e-6f;
  // 1000 - 40 = 960 counts * 2000 / 32768 dps, in rad/s.
  assert(std::fabs(gyroRateAfterBias(1000, 40.0f) - 1.0226538586f) < tolerance);
  assert(std::fabs(gyroRateAfterBias(1000, -40.0f) - 1.1078750135f) < tolerance);
  assert(std::fabs(gyroRateAfterBias(1000, 0.0f) - 1.0652644360f) < tolerance);
  // 100 - 2.7 = 97.3 rounds to 97 counts; truncating the bias would give 98.
  assert(std::fabs(gyroRateAfterBias(100, 2.7f) - 0.1033306503f) < tolerance);
  // Full scale saturates instead of wrapping to the opposite sign.
  assert(std::fabs(gyroRateAfterBias(-32768, 10.0f) - (-34.9065850399f)) < 1e-4f);
  assert(std::fabs(gyroRateAfterBias(32767, -10.0f) - 34.9055197755f) < 1e-4f);
}

int main() {
  imu::Settings settings{};
  imu::CalibrationValues calibration{};
  assert(imu::begin(settings, calibration));
  assert(initializedRate == imu::kSampleRateHz);
  assert(!imu::service());
  assert(!imu::latest().valid);

  acceptSample(1000, {0, 0, 4096, 10, -20, 30, 132});
  const imu::Snapshot first = imu::latest();
  assert(first.valid && first.sequence == 1 && first.readTimeUs == 1000);
  assert(first.dtSec == 0.0f);
  assert(std::fabs(first.accelerationG.z - 1.0f) < 1e-6f);
  assert(std::fabs(first.temperatureC - (132.0f / 132.48f + 25.0f)) < 1e-5f);

  // A poll without DATA_RDY must not publish a duplicate or advance timing.
  assert(!imu::service());
  assert(imu::latest().sequence == first.sequence);
  assert(imu::timingStatistics().acceptedSamples == 1);

  acceptSample(2000);
  assert(imu::latest().sequence == 2);
  assert(std::fabs(imu::latest().dtSec - 0.001f) < 1e-7f);
  assert(imu::timingStatistics().maxIntervalUs == 1000);

  acceptSample(3500);
  assert(imu::timingStatistics().longGapCount == 0); // exactly 1.5 ms is not long
  acceptSample(5001);
  auto stats = imu::timingStatistics();
  assert(stats.longGapCount == 1 && stats.maxIntervalUs == 1501);

  // Unsigned subtraction keeps host read timing valid across micros() wrap.
  imu::begin(settings, calibration);
  acceptSample(UINT32_MAX - 499U);
  acceptSample(500U);
  assert(std::fabs(imu::latest().dtSec - 0.001f) < 1e-7f);
  assert(imu::timingStatistics().maxIntervalUs == 1000U);

  auto changed = imu::settings();
  changed.gyroCutoffHz = 0;
  assert(!imu::setSettings(changed));
  assert(imu::settings().gyroCutoffHz == settings.gyroCutoffHz);
  changed = imu::settings();
  changed.accelCutoffHz = imu::kSampleRateHz / 2;
  assert(!imu::setSettings(changed));
  changed = imu::settings();
  changed.mahonyTwoKp = -1.0f;
  assert(!imu::setSettings(changed));
  changed = imu::settings();
  changed.gyroCutoffHz = 30;
  changed.mahonyTwoKp = 0.7f;
  assert(imu::setSettings(changed));
  assert(imu::settings().gyroCutoffHz == 30);
  assert(imu::settings().accelCutoffHz == settings.accelCutoffHz);
  assert(std::fabs(imu::settings().mahonyTwoKp - 0.7f) < 1e-7f);

  imu::CalibrationValues updated{};
  updated.gyro = {1.25f, -2.5f, 3.75f};
  updated.attitude = {4.0f, -5.0f};
  imu::setCalibration(updated);
  const auto readback = imu::calibration();
  assert(readback.gyro.xRawCounts == updated.gyro.xRawCounts);
  assert(readback.attitude.pitchDegrees == updated.attitude.pitchDegrees);

  verifyOnlyChangedCutoffResetsItsFilter();
  verifySetCalibrationRefreshesPublishedCorrectedAngles();
  verifyFailedBeginStopsAndClearsPreviousInstance();
  verifyGyroCalibrationAveragesLargeConstantOffset();
  verifyGyroCalibrationRestartsAfterMotion();
  verifyBiasCorrectionKeepsFractionAndSaturates();
  return 0;
}
