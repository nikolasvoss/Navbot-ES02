// The sensors of the simulated robot: IMU, encoder and battery ADC (plan, Sensors).

#include <algorithm>
#include <cmath>

#include "BoardConfig.h"
#include "MathConstants.h"
#include "sim.h"

namespace sim {
namespace {

// imu.cpp's scales: +-8 g and +-2000 deg/s over 32768 counts.
constexpr double kAccelCountsPerG = 32768.0 / 8.0;
constexpr double kGyroCountsPerDegPerSec = 32768.0 / 2000.0;
constexpr double kRadToDeg = 180.0 / kPi;

double gaussian(std::mt19937 &rng, double sigma) {
  if (sigma <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> noise(0.0, sigma);
  return noise(rng);
}

int16_t saturatedCount(double counts) {
  return static_cast<int16_t>(std::clamp(std::round(counts), -32768.0, 32767.0));
}

}  // namespace

ICM42688RawSample imuSample(const PlantState &state, const PlantParams &plant, const SensorParams &sensors,
                            std::mt19937 &rng) {
  const double g = plant.gravityMps2;
  const double h = sensors.imuHeightM;
  const double cosPitch = std::cos(state.pitchRad);
  const double sinPitch = std::sin(state.pitchRad);
  const double pitchRate = state.pitchRateRadPerSec;
  const double pitchAccel = state.pitchAccelRadPerSec2;

  // Acceleration of the IMU point in the world frame, then the specific force in the body frame in g.
  const double ax = state.accelMps2 + h * (cosPitch * pitchAccel - sinPitch * pitchRate * pitchRate);
  const double az = h * (-sinPitch * pitchAccel - cosPitch * pitchRate * pitchRate);
  const double fx = -(state.vMps * state.yawRateRadPerSec) / g;
  const double fy = (ax * cosPitch - (az + g) * sinPitch) / g;
  const double fz = (ax * sinPitch + (az + g) * cosPitch) / g;

  // Body rates in rad/s.
  const double gx = (sensors.flipGyroX ? 1.0 : -1.0) * pitchRate;
  const double gy = -state.yawRateRadPerSec * sinPitch;
  const double gz = state.yawRateRadPerSec * cosPitch;

  ICM42688RawSample raw{};
  raw.accelerationX = saturatedCount((fx + gaussian(rng, sensors.accelNoiseG)) * kAccelCountsPerG);
  raw.accelerationY = saturatedCount((fy + gaussian(rng, sensors.accelNoiseG)) * kAccelCountsPerG);
  raw.accelerationZ = saturatedCount((fz + gaussian(rng, sensors.accelNoiseG)) * kAccelCountsPerG);
  raw.gyroscopeX = saturatedCount((gx + sensors.gyroBiasRadPerSec[0] + gaussian(rng, sensors.gyroNoiseRadPerSec)) *
                                  kRadToDeg * kGyroCountsPerDegPerSec);
  raw.gyroscopeY = saturatedCount((gy + sensors.gyroBiasRadPerSec[1] + gaussian(rng, sensors.gyroNoiseRadPerSec)) *
                                  kRadToDeg * kGyroCountsPerDegPerSec);
  raw.gyroscopeZ = saturatedCount((gz + sensors.gyroBiasRadPerSec[2] + gaussian(rng, sensors.gyroNoiseRadPerSec)) *
                                  kRadToDeg * kGyroCountsPerDegPerSec);
  raw.temperature = 0;
  return raw;
}

double encoderAngle(double angleRad, const SensorParams &sensors) {
  const double step = 2.0 * kPi / sensors.encoderCountsPerTurn;
  return std::round(angleRad / step) * step;
}

uint16_t batteryAdc(double busV, const SensorParams &sensors, std::mt19937 &rng) {
  const double counts = busV / board::kBatteryVoltsPerAdcCount + gaussian(rng, sensors.batteryAdcNoiseCounts);
  return static_cast<uint16_t>(std::clamp(std::round(counts), 0.0, 4095.0));
}

}  // namespace sim
