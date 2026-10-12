#pragma once

#include <cstdint>

#include "Calibration.h"

namespace imu {

inline constexpr uint16_t kSampleRateHz = 1000;

struct Vector3 {
  float x{};
  float y{};
  float z{};
};

struct EulerDegrees {
  float roll{};
  float pitch{};
  float yaw{};
};

struct Settings {
  uint16_t gyroCutoffHz = 25;
  uint16_t accelCutoffHz = 25;
  float mahonyTwoKp = 0.4f;
  float mahonyTwoKi = 0.001f;
};

struct CalibrationValues {
  GyroBiasRawCounts gyro{};
  AttitudeBiasDegrees attitude{};
};

struct Snapshot {
  bool valid = false;
  uint32_t sequence = 0;    // Accepted reads, not sensor conversions.
  uint32_t readTimeUs = 0;  // Host timestamp, not sensor timestamp.
  float dtSec = 0;          // Zero on the first accepted read.

  Vector3 accelerationG{};             // Unfiltered.
  Vector3 angularVelocityRadPerSec{};  // Bias-corrected, unfiltered.
  Vector3 filteredAccelerationG{};
  Vector3 filteredAngularVelocityRadPerSec{};
  float temperatureC = 0;

  EulerDegrees attitudeDegrees{};
  // Published attitude minus the active calibration offsets; refreshed by setCalibration().
  float correctedRollDegrees = 0;
  float correctedPitchDegrees = 0;

  Vector3 complementaryAnglesDegrees{};   // Existing angleX/Y/Z.
  Vector3 integratedGyroAnglesDegrees{};  // Existing angleGyroX/Y/Z.
};

struct TimingStatistics {
  uint32_t acceptedSamples = 0;
  uint32_t longGapCount = 0;
  uint32_t maxIntervalUs = 0;
};

// Starts a fresh instance. Failure leaves the module stopped with an invalid snapshot.
bool begin(const Settings &settings, const CalibrationValues &calibration);

// Polls once; returns true only after processing and publishing a ready read.
bool service();

// The returned snapshot remains stable until the next accepted service() call.
const Snapshot &latest();
Settings settings();
CalibrationValues calibration();
TimingStatistics timingStatistics();

bool setSettings(const Settings &settings);
// Applies offsets immediately to the corrected angles in the published snapshot.
void setCalibration(const CalibrationValues &calibration);

// Preserves the existing explicit, blocking calibration operation.
void calibrateGyroBiasBlocking();

}  // namespace imu
