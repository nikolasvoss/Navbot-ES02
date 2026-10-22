#pragma once

#include <cstdint>

struct Measurements {
  uint32_t imuSequence;
  uint32_t imuReadTimeUs;
  bool imuValid;
  float tiltRad;
  float tiltRateRadPerSec;
  float yawRateRadPerSec;
  uint32_t wheelReadTimeUs;
  bool wheelsValid;
  float leftWheelRadPerSec;   // forward positive
  float rightWheelRadPerSec;  // forward positive
  uint32_t busReadTimeUs;
  float busVoltageV;
};

struct Request {
  uint32_t receivedTimeUs;
  bool sourceValid;
  float speedMps;
  float yawRateRadPerSec;
  bool enableSwitch;
  bool commissioningSwitch;
  bool saveSwitch;
  bool knobValid;
  float knobTrimDeg;
};

enum class State : uint8_t { Disabled, Arming, Run, Commissioning, Fault };

enum class FaultReason : uint8_t {
  None,
  StaleImu,
  StaleWheels,
  StaleBus,
  InvalidInput,
  BadDt,
  Fall,
  Overspeed,
};

struct Diagnostics {
  float c = 0.0f;
  float d = 0.0f;
  float cMin = 0.0f;
  float cMax = 0.0f;
  float dMin = 0.0f;
  float dMax = 0.0f;
  bool tiltClipped = false;
  bool effortClipped = false;
  bool voltageClipped = false;
  bool currentClipped = false;
  bool forcedDifferential = false;
  float tiltRefRad = 0.0f;
  float trimDeg = 0.0f;
  float speedP = 0.0f;
  float speedI = 0.0f;
  float reserve = 0.0f;
  float demandScale = 0.0f;
};

struct Output {
  bool driveEnabled = false;
  float leftA = 0.0f;   // forward positive
  float rightA = 0.0f;  // forward positive
  State state = State::Disabled;
  FaultReason fault = FaultReason::None;
  Diagnostics diag;
  bool saveTrimRequested = false;
  float trimToSaveDeg = 0.0f;
};

struct Config {
  // Arming: all values uncommissioned.
  uint32_t armingTimeoutUs = 0;          // uncommissioned
  float armingTiltWindowRad = 0.0f;      // uncommissioned
  float armingTiltRateRadPerSec = 0.0f;  // uncommissioned
  float armingRequestSpeedMps = 0.0f;    // uncommissioned
  float armingRequestYawRadPerSec = 0.0f;  // uncommissioned

  // Measurement freshness: maximum age of each acquisition timestamp. Uncommissioned.
  uint32_t imuMaxAgeUs = 0;    // uncommissioned
  uint32_t wheelMaxAgeUs = 0;  // uncommissioned
  uint32_t busMaxAgeUs = 0;    // uncommissioned

  // Largest allowed gap between consecutive processed IMU samples. Uncommissioned.
  uint32_t dtMaxUs = 0;  // uncommissioned

  // Fall: tilt beyond fallAngleRad for longer than fallDurationUs. Uncommissioned.
  float fallAngleRad = 0.0f;     // uncommissioned
  uint32_t fallDurationUs = 0;   // uncommissioned

  // Overspeed: either wheel beyond wheelOverspeedRadPerSec for longer than overspeedDurationUs. Uncommissioned.
  float wheelOverspeedRadPerSec = 0.0f;  // uncommissioned
  uint32_t overspeedDurationUs = 0;      // uncommissioned

  // Inputs supplied by the sketch from the stored trim record and pose check, not commissioning constants.
  bool trimValidated = false;
  float storedTrimDeg = 0.0f;
  bool fixedPoseReachable = false;
};

class BalanceController {
 public:
  Output update(const Measurements &measurements, const Request &request, const Config &config,
                uint32_t nowUs);

 private:
  void resetRunTracking();
  void checkRunning(const Measurements &measurements, const Config &config, uint32_t nowUs);
  void latchFault(FaultReason reason);

  State state_ = State::Disabled;
  FaultReason fault_ = FaultReason::None;
  bool enableLevelKnown_ = false;
  bool previousEnableSwitch_ = false;
  uint32_t armingStartUs_ = 0;

  // IMU sequence and timestamp of the last processed sample. Sequence persists across arming.
  bool imuSequenceSeen_ = false;
  uint32_t lastImuSequence_ = 0;
  bool imuTimestampValid_ = false;  // false until the first processed sample after entering Run
  uint32_t lastImuReadTimeUs_ = 0;
  float dtSec_ = 0.0f;

  bool fallBeyond_ = false;
  uint32_t fallStartUs_ = 0;
  bool overspeedActive_ = false;
  uint32_t overspeedStartUs_ = 0;
};
