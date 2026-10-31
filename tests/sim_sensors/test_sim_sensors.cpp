#include <Arduino.h>

#include "ICM42688.h"
#include "imu.h"
#include "Logging.h"
#include "sim.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <random>

namespace Logging {
void message(Level, const char *, const char *, ...) {}
}

namespace {

sim::SensorParams quietSensors(double imuHeightM = 0.0) {
  sim::SensorParams sensors;
  sensors.imuHeightM = imuHeightM;
  sensors.gyroNoiseRadPerSec = 0.0;
  sensors.accelNoiseG = 0.0;
  sensors.batteryAdcNoiseCounts = 0.0;
  return sensors;
}

ICM42688RawSample sampleOf(const sim::PlantState &state, const sim::SensorParams &sensors) {
  std::mt19937 rng(1);
  return sim::imuSample(state, sim::PlantParams{}, sensors, rng);
}

void resetHal() { sim::hal() = sim::Hal{}; }

void verifyUprightAtRest() {
  const ICM42688RawSample raw = sampleOf(sim::PlantState{}, quietSensors());
  assert(raw.accelerationX == 0);
  assert(raw.accelerationY == 0);
  assert(raw.accelerationZ == 4096);
  assert(raw.gyroscopeX == 0);
  assert(raw.gyroscopeY == 0);
  assert(raw.gyroscopeZ == 0);
}

void verifyForwardLeanAtRest() {
  sim::PlantState state;
  state.pitchRad = 0.1;
  const ICM42688RawSample raw = sampleOf(state, quietSensors());
  assert(raw.accelerationX == 0);
  assert(raw.accelerationY == -409);  // -sin(0.1) * 4096 = -408.92
  assert(raw.accelerationZ == 4076);  // cos(0.1) * 4096 = 4075.54
}

void verifyPitchRateSignAndFlip() {
  sim::PlantState state;
  state.pitchRateRadPerSec = 1.0;
  assert(sampleOf(state, quietSensors()).gyroscopeX == -939);  // -1 rad/s = -57.30 deg/s * 16.384
  sim::SensorParams flipped = quietSensors();
  flipped.flipGyroX = true;
  assert(sampleOf(state, flipped).gyroscopeX == 939);
}

void verifyYawRateUpright() {
  sim::PlantState state;
  state.yawRateRadPerSec = 1.0;
  const ICM42688RawSample raw = sampleOf(state, quietSensors());
  assert(raw.gyroscopeX == 0);
  assert(raw.gyroscopeZ == 939);  // 57.30 deg/s * 16.384
}

void verifyForwardAccelerationReadsOnY() {
  sim::PlantState state;
  state.accelMps2 = 9.81;
  const ICM42688RawSample raw = sampleOf(state, quietSensors());
  assert(raw.accelerationY == 4096);  // 9.81 m/s2 forward adds 1 g on y
  assert(raw.accelerationZ == 4096);  // gravity on z is unchanged
}

// Forward lean at rest through the real imu.cpp and Mahony filter, as on the robot. Pitch 0.1 rad is 5.73 degrees.
void verifyImuSignChainLean() {
  resetHal();
  assert(imu::begin(imu::Settings{}, imu::CalibrationValues{}));
  sim::PlantState state;
  state.pitchRad = 0.1;
  std::mt19937 rng(1);
  const ICM42688RawSample raw = sim::imuSample(state, sim::PlantParams{}, quietSensors(), rng);
  for (uint32_t index = 1; index <= 20000; ++index) {
    sim::hal().nowUs = index * 1000U;
    sim::hal().imuSample = raw;
    sim::hal().imuReady = true;
    assert(imu::service());
  }
  const double roll = imu::latest().correctedRollDegrees;
  std::printf("imu sign chain: correctedRollDegrees after 20 s = %.4f\n", roll);
  assert(std::fabs(roll - (-5.73)) < 0.5);
}

void verifyEncoderAndBattery() {
  sim::SensorParams sensors = quietSensors();
  std::mt19937 rng(1);
  // 1.0 rad with 4096 counts: round(1.0 / (2 pi / 4096)) = 652 counts.
  assert(std::fabs(sim::encoderAngle(1.0, sensors) - 1.000155473701438) < 1e-12);
  assert(sim::batteryAdc(7.77, sensors, rng) == 813);
  assert(sim::batteryAdc(100.0, sensors, rng) == 4095);
  assert(sim::batteryAdc(0.0, sensors, rng) == 0);
}

void verifyClockAndDriver() {
  resetHal();
  assert(micros() == 0);
  assert(millis() == 0);
  delay(3);
  assert(micros() == 3000);
  assert(millis() == 3);
  assert(sim::hal().nowUs == 3000);

  assert(initICM42688(imu::kSampleRateHz));
  sim::hal().imuInitOk = false;
  assert(!initICM42688(imu::kSampleRateHz));
  sim::hal().imuInitOk = true;

  ICM42688RawSample sample{};
  assert(!readIMUDataIfReady(sample));
  sim::hal().imuSample = ICM42688RawSample{1, -2, 4096, 3, -4, 5, 6};
  sim::hal().imuReady = true;
  assert(readIMUDataIfReady(sample));
  assert(sample.accelerationX == 1 && sample.accelerationY == -2 && sample.accelerationZ == 4096);
  assert(sample.gyroscopeX == 3 && sample.gyroscopeY == -4 && sample.gyroscopeZ == 5 && sample.temperature == 6);
  assert(!readIMUDataIfReady(sample));
}

}  // namespace

int main() {
  verifyUprightAtRest();
  verifyForwardLeanAtRest();
  verifyPitchRateSignAndFlip();
  verifyYawRateUpright();
  verifyForwardAccelerationReadsOnY();
  verifyImuSignChainLean();
  verifyEncoderAndBattery();
  verifyClockAndDriver();
  std::printf("sim_sensors: all tests passed\n");
  return 0;
}
