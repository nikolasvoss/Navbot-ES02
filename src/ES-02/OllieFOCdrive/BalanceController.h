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
  // Sketch-supplied clip flags of the effort path, from the SimpleFOC estimated-current voltage clamp and the
  // current_limit clamp. Either one freezes the speed integral on the next loop run.
  bool voltageClipped = false;
  bool currentClipped = false;
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
  AuthorityExhausted,
  TiltSaturated,
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
  bool knobEngaged = false;
  bool commissioningRefused = false;  // Run stayed, because the commissioning switch was on and the transfer failed or the request was lost
  bool saveRefused = false;           // a save edge in Commissioning was refused on this tick; no pulse
};

struct Output {
  bool driveEnabled = false;
  float leftA = 0.0f;   // forward positive
  float rightA = 0.0f;  // forward positive
  State state = State::Disabled;
  FaultReason fault = FaultReason::None;
  Diagnostics diag;
  // One-tick pulse on a granted save. The module does not persist or validate the trim: storage and the 60 s
  // hold happen outside it, and the caller stores trimToSaveDeg on this pulse.
  bool saveTrimRequested = false;
  float trimToSaveDeg = 0.0f;
};

struct Config {
  // Arming: all values uncommissioned.
  uint32_t armingTimeoutUs = 0;          // 0 holds Arming until the start conditions hold or the switch goes off
  float armingTiltWindowRad = 0.0f;      // uncommissioned
  float armingTiltRateRadPerSec = 0.0f;  // uncommissioned
  float armingRequestSpeedMps = 0.0f;    // uncommissioned
  float armingRequestYawRadPerSec = 0.0f;  // uncommissioned
  float armingWheelSpeedRadPerSec = 0.0f;  // uncommissioned, each wheel's |speed| must be within it

  // Measurement freshness: maximum age of each acquisition timestamp. Uncommissioned.
  uint32_t imuMaxAgeUs = 0;    // uncommissioned
  uint32_t wheelMaxAgeUs = 0;  // uncommissioned
  uint32_t busMaxAgeUs = 0;    // uncommissioned

  // Largest allowed gap between consecutive processed IMU samples. Uncommissioned.
  uint32_t dtMaxUs = 0;  // uncommissioned

  // Fall: tilt beyond fallAngleRad for longer than fallDurationUs. Uncommissioned.
  float fallAngleRad = 0.0f;     // uncommissioned
  uint32_t fallDurationUs = 0;   // uncommissioned
  // After a Fall fault with the switch still on, the tilt must stay inside armingTiltWindowRad for this long before
  // the controller re-enters Arming. Zero keeps the Fall fault latched until the switch is cycled.
  uint32_t fallRecoveryDelayUs = 0;  // uncommissioned

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

  // Demand shaping. Under reserve pressure, acceleration and yaw give way first, then the cruise cap. The brake
  // ramp is never scaled.
  float speedCapMps = 0.0f;          // uncommissioned
  float accelCapMps2 = 0.0f;         // uncommissioned
  float brakeCapMps2 = 0.0f;         // uncommissioned, rate of speed magnitude decrease
  float yawRateCapRadPerSec = 0.0f;  // uncommissioned
  uint32_t requestMaxAgeUs = 0;      // uncommissioned, older motion requests count as zero
  // demandScale below which the cruise cap starts to fall. Must be in (0, 1) in configValid.
  float demandScaleCapStart = 0.0f;  // uncommissioned

  // Speed loop. The provisional 50 Hz rate is 20000 us; it is set by the commissioning record, not here.
  float wheelRadiusM = 0.0f;          // uncommissioned
  uint32_t speedLoopPeriodUs = 0;     // uncommissioned
  float speedKpRadPerMps = 0.0f;      // uncommissioned
  float speedKiRadPerMpsS = 0.0f;     // uncommissioned
  bool speedIntegralEnabled = false;  // off until the speed integral is commissioned
  float tiltRefLimitRad = 0.0f;       // uncommissioned, bounds |trim + speed correction|
  float tiltRefSlewRadPerSec = 0.0f;  // uncommissioned
  // Applied-unclipped tilt reference beyond tiltRefLimitRad for longer than this faults with TiltSaturated.
  uint32_t tiltClipDurationUs = 0;    // uncommissioned, must be > 0 in configValid

  // Trim and standstill commissioning. Trim is the balance reference offset in radians, not a gain.
  float trimEnvelopeRad = 0.0f;               // uncommissioned, max |trim|, at most tiltRefLimitRad
  float trimSlewRadPerSec = 0.0f;             // uncommissioned, knob-driven trim change rate
  float knobPickupBandRad = 0.0f;             // uncommissioned, knob engages inside this band around the trim
  float standstillWheelSpeedTolMps = 0.0f;    // uncommissioned, measured speed allowed for the transfer
  float standstillTiltErrorTolRad = 0.0f;     // uncommissioned, |tilt - applied reference| allowed for the transfer
  float transferToleranceRad = 0.0f;          // uncommissioned, numerical tolerance for an unchanged reference
  // Raw request near-neutral bands for a save and for the Run -> Commissioning transfer. Not the arming bands.
  float saveRequestSpeedTolMps = 0.0f;        // uncommissioned, finite and >= 0
  float saveRequestYawTolRadPerSec = 0.0f;    // uncommissioned, finite and >= 0

  // Balance and yaw law, in amperes. Gains may be negative; the stage-1 sign test checks the signs.
  float tiltPAPerRad = 0.0f;              // uncommissioned, A per rad of tilt error
  float gyroDampingAPerRadPerSec = 0.0f;  // uncommissioned, A per rad/s of tilt rate
  float yawPAPerRadPerSec = 0.0f;         // uncommissioned, A per rad/s of yaw-rate error

  // Reserve below reserveLowA scales demand down linearly to zero at zero reserve.
  float reserveLowA = 0.0f;         // uncommissioned, must be positive
  // Clipped balance effort for longer than this faults with AuthorityExhausted.
  uint32_t authorityDurationUs = 0;  // uncommissioned

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

// Paired wheel targets in amperes. Left is c - d and right is c + d, both forward positive.
struct Allocation {
  float c = 0.0f;
  float d = 0.0f;
  float cMin = 0.0f;
  float cMax = 0.0f;
  float dMin = 0.0f;
  float dMax = 0.0f;
  bool effortClipped = false;        // c differs from the balance request
  bool differentialClipped = false;  // d differs from the yaw request
  bool forcedDifferential = false;   // same as differentialClipped: the envelope forced d away from the request
  float leftA = 0.0f;
  float rightA = 0.0f;
  float reserve = 0.0f;
};

class BalanceController {
 public:
  // Requires finite envelopes with min <= max on each wheel, as checked on each tick by update().
  // Balance takes priority over the yaw request. With asymmetric envelopes, a common effort near either
  // end forces a differential that the yaw request did not ask for.
  static Allocation allocateEffort(float balanceCommonA, float yawDifferentialA, float leftMinA, float leftMaxA,
                                   float rightMinA, float rightMaxA);

  // Ramps the requested speed toward the capped target and clamps the requested yaw rate. A NaN
  // request counts as zero, and a NaN demandScale counts as zero. Acceleration and yaw caps fall from 1 at
  // demandScale 1 to 0 at demandScaleCapStart; the speed cap falls from 1 at demandScaleCapStart to 0 at 0.
  static Demand shapeDemand(Demand previous, Demand requested, float demandScale, float dtSec,
                            const Config &config);

  // Angles are radians. Speed I is held in speedIRad; the tilt reference is clamped to +/-tiltRefLimitRad.
  struct SpeedLoopResult {
    float speedPRad;
    float speedIRad;
    float desiredTiltRad;
    bool tiltClipped;
  };
  // Applies the anti-windup precedence: freeze on downstream limits, unwind-only when the tilt reference
  // is clipped, otherwise integrate.
  static SpeedLoopResult speedLoopStep(float speedIPrevRad, float speedErrorMps, float trimRad,
                                       bool downstreamLimited, float dtSec, const Config &config);
  // Moves previousRad toward desiredRad by at most tiltRefSlewRadPerSec * dtSec, landing exactly on it.
  static float slewTiltRef(float previousRad, float desiredRad, float dtSec, const Config &config);

  Output update(const Measurements &measurements, const Request &request, const Config &config,
                uint32_t nowUs);

 private:
  bool driving() const { return state_ == State::Run || state_ == State::Commissioning; }
  void resetRunTracking(const Config &config, uint32_t nowUs);
  void checkRunning(const Measurements &measurements, const Request &request, const Config &config,
                    uint32_t nowUs);
  bool tryEnterCommissioning(const Measurements &measurements, const Request &request, const Config &config,
                             bool clippedSinceLoop);
  bool saveAllowed(const Measurements &measurements, const Request &request, const Config &config,
                   bool lost) const;
  void stepKnob(const Request &request, const Config &config);
  void latchFault(FaultReason reason);
  void recoverFromFall(const Measurements &measurements, const Config &config, uint32_t nowUs,
                       bool lost);
  void checkEnvelopes(const Measurements &measurements, const Config &config);
  void computeEffort(const Measurements &measurements, const Config &config, uint32_t nowUs, bool newImuSample);

  State state_ = State::Disabled;
  FaultReason fault_ = FaultReason::None;
  bool enableLevelKnown_ = false;
  bool previousEnableSwitch_ = false;
  // Save switch level of the previous update() call; a save is an edge of this level in Commissioning.
  bool previousSaveSwitch_ = false;
  // False after a lost request, so the first valid frame rebases the level without an edge.
  bool saveTracked_ = false;
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

  // Scales acceleration, yaw and the cruise cap (see shapeDemand). Set from the reserve on every Run or
  // Commissioning tick; 1.0 on entering Run.
  float demandScale_ = 1.0f;
  // Balance and yaw outputs of the last processed sample in Run; zero outside Run.
  Allocation allocation_;
  float balanceCommonA_ = 0.0f;
  float yawDifferentialA_ = 0.0f;
  bool haveEffortDemand_ = false;
  // Start of the current run of clipped balance effort, valid while authorityClipped_ is set.
  bool authorityClipped_ = false;
  uint32_t authorityStartUs_ = 0;
  // Shaped demand of the last processed IMU sample; zero outside Run and on entering Run.
  Demand shapedDemand_;

  bool fallRecoveryInside_ = false;
  uint32_t fallRecoveryStartUs_ = 0;
  bool fallBeyond_ = false;
  uint32_t fallStartUs_ = 0;
  bool overspeedActive_ = false;
  uint32_t overspeedStartUs_ = 0;

  // Speed loop and tilt reference. Zero outside Run and Commissioning; the trim changes only by the knob or the commissioning transfer.
  float trimRad_ = 0.0f;
  bool knobEngaged_ = false;  // only in Commissioning; cleared on entry, on RC loss and on an invalid knob
  bool commissioningRefused_ = false;  // set on the tick the Run -> Commissioning transfer was refused
  float speedPRad_ = 0.0f;
  float speedIRad_ = 0.0f;
  float desiredTiltRad_ = 0.0f;  // stored result of the last speed loop run
  float tiltRefRad_ = 0.0f;      // applied reference, slewed toward desiredTiltRad_ every processed sample
  bool tiltClipped_ = false;
  uint32_t lastSpeedLoopUs_ = 0;
  // OR-latched on every Run tick since the last speed loop run. The loop reads them, then they are cleared.
  bool effortClippedSinceLoop_ = false;
  bool reserveLowSinceLoop_ = false;
  bool voltageOrCurrentClippedSinceLoop_ = false;
  // Start of the current run of clipped tilt reference, valid while tiltClipRunning_ is set. Checked every tick.
  bool tiltClipRunning_ = false;
  uint32_t tiltClipStartUs_ = 0;
};
