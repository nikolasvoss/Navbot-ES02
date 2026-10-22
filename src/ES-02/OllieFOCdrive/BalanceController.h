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
  InvalidEnvelope,
  InvalidConfig,
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
  float leftMinA = 0.0f;   // forward positive effort envelope
  float leftMaxA = 0.0f;
  float rightMinA = 0.0f;
  float rightMaxA = 0.0f;
  float shapedSpeedMps = 0.0f;
  float shapedYawRateRadPerSec = 0.0f;
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

  // Effort envelope per wheel. All uncommissioned; zero defaults make the envelope invalid.
  float phaseResistanceOhm = 0.0f;  // uncommissioned
  float kvRating = 0.0f;            // uncommissioned, rpm/V
  float voltageLimitV = 0.0f;       // uncommissioned
  float currentLimitA = 0.0f;       // uncommissioned
  float sagMarginV = 0.0f;          // uncommissioned
  float modulationFactor = 0.0f;    // uncommissioned, scales the usable bus voltage
  int leftWheelSign = 0;            // uncommissioned, +1 or -1: motor speed = sign * forward speed
  int rightWheelSign = 0;           // uncommissioned, +1 or -1
  float busMinV = 0.0f;             // uncommissioned, commissioned model range
  float busMaxV = 0.0f;             // uncommissioned, commissioned model range

  // Demand shaping. Speed cap, yaw cap and acceleration are scaled by demandScale; the brake ramp is not.
  float speedCapMps = 0.0f;          // uncommissioned
  float accelCapMps2 = 0.0f;         // uncommissioned
  float brakeCapMps2 = 0.0f;         // uncommissioned, rate of speed magnitude decrease
  float yawRateCapRadPerSec = 0.0f;  // uncommissioned
  uint32_t requestMaxAgeUs = 0;      // uncommissioned, older motion requests count as zero

  // Inputs supplied by the sketch from the stored trim record and pose check, not commissioning constants.
  bool trimValidated = false;
  float storedTrimDeg = 0.0f;
  bool fixedPoseReachable = false;
};

// Requested motion after shaping. Speed is forward positive.
struct Demand {
  float speedMps = 0.0f;
  float yawRateRadPerSec = 0.0f;
};

class BalanceController {
 public:
  // Ramps the requested speed toward the capped target and clamps the requested yaw rate. A NaN
  // request counts as zero, and a NaN demandScale counts as zero.
  static Demand shapeDemand(Demand previous, Demand requested, float demandScale, float dtSec,
                            const Config &config);

  Output update(const Measurements &measurements, const Request &request, const Config &config,
                uint32_t nowUs);

 private:
  void resetRunTracking();
  void checkRunning(const Measurements &measurements, const Request &request, const Config &config,
                    uint32_t nowUs);
  void latchFault(FaultReason reason);
  void checkEnvelopes(const Measurements &measurements, const Config &config);

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

  // Forward-positive effort envelope of each wheel from the last Run or Commissioning tick.
  float leftMinA_ = 0.0f;
  float leftMaxA_ = 0.0f;
  float rightMinA_ = 0.0f;
  float rightMaxA_ = 0.0f;

  // Scales both demand caps. Stays 1.0 until the reserve logic sets it.
  float demandScale_ = 1.0f;
  // Shaped demand of the last processed IMU sample; zero outside Run and on entering Run.
  Demand shapedDemand_;

  bool fallBeyond_ = false;
  uint32_t fallStartUs_ = 0;
  bool overspeedActive_ = false;
  uint32_t overspeedStartUs_ = 0;
};
