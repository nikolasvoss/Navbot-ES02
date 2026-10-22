#include "BalanceController.h"

#include <algorithm>
#include <cmath>

#include "MathConstants.h"

namespace {
// Equals SimpleFOC's _RPM_TO_RADS: radians per second per rpm.
constexpr float kRpmToRadPerSec = 0.10471975512f;

struct WheelInterval {
  float minA;
  float maxA;
  bool finite;
};

bool isUnitSign(int sign) { return sign == 1 || sign == -1; }

// Forward-positive current interval of one wheel. The motor sees sign * forward speed, and forward
// current is sign * motor current, so a -1 wheel mirrors the motor-frame interval.
WheelInterval wheelInterval(float forwardRadPerSec, int sign, float vMaxV, const Config &config) {
  const float bemfV = (sign * forwardRadPerSec) / (config.kvRating * std::sqrt(3.0f)) / kRpmToRadPerSec;
  const float maxMotorA = std::min(config.currentLimitA, (vMaxV - bemfV) / config.phaseResistanceOhm);
  const float minMotorA = std::max(-config.currentLimitA, (-vMaxV - bemfV) / config.phaseResistanceOhm);
  WheelInterval out = sign > 0 ? WheelInterval{minMotorA, maxMotorA, false}
                               : WheelInterval{-maxMotorA, -minMotorA, false};
  // std::min and std::max would hide a NaN back-EMF behind the current limit, so check it here.
  out.finite = std::isfinite(bemfV) && std::isfinite(out.minA) && std::isfinite(out.maxA);
  return out;
}

bool elapsedLongerThan(uint32_t nowUs, uint32_t startUs, uint32_t durationUs) {
  return static_cast<uint32_t>(nowUs - startUs) > durationUs;
}

// A timestamp ahead of nowUs wraps to a huge unsigned age and counts as stale.
bool isStale(uint32_t nowUs, uint32_t readTimeUs, uint32_t maxAgeUs) {
  return static_cast<uint32_t>(nowUs - readTimeUs) > maxAgeUs;
}

bool armingConditionsMet(const Measurements &measurements, const Request &request, const Config &config,
                         uint32_t nowUs) {
  const float trimRad = config.storedTrimDeg * kDegreesToRadiansF;
  return measurements.imuValid && measurements.wheelsValid &&
         !isStale(nowUs, measurements.imuReadTimeUs, config.imuMaxAgeUs) &&
         !isStale(nowUs, measurements.wheelReadTimeUs, config.wheelMaxAgeUs) &&
         !isStale(nowUs, measurements.busReadTimeUs, config.busMaxAgeUs) &&
         std::fabs(measurements.tiltRad - trimRad) <= config.armingTiltWindowRad &&
         std::fabs(measurements.tiltRateRadPerSec) <= config.armingTiltRateRadPerSec &&
         std::fabs(request.speedMps) <= config.armingRequestSpeedMps &&
         std::fabs(request.yawRateRadPerSec) <= config.armingRequestYawRadPerSec;
}

// Demand caps must be usable: a zero brake cap would hold the last speed when the request times out,
// and a negative or NaN rate would move speed the wrong way. A zero slew rate would freeze the tilt
// reference, and a zero wheel radius or loop period would divide by zero or never run the loop.
bool configValid(const Config &config) {
  return std::isfinite(config.speedCapMps) && config.speedCapMps >= 0.0f &&
         std::isfinite(config.accelCapMps2) && config.accelCapMps2 > 0.0f &&
         std::isfinite(config.brakeCapMps2) && config.brakeCapMps2 > 0.0f &&
         std::isfinite(config.yawRateCapRadPerSec) && config.yawRateCapRadPerSec >= 0.0f &&
         std::isfinite(config.wheelRadiusM) && config.wheelRadiusM > 0.0f &&
         config.speedLoopPeriodUs > 0 && std::isfinite(config.speedKpRadPerMps) &&
         std::isfinite(config.speedKiRadPerMpsS) && std::isfinite(config.tiltRefLimitRad) &&
         config.tiltRefLimitRad >= 0.0f && std::isfinite(config.tiltRefSlewRadPerSec) &&
         config.tiltRefSlewRadPerSec > 0.0f &&
         std::fabs(config.storedTrimDeg * kDegreesToRadiansF) <= config.tiltRefLimitRad &&
         std::isfinite(config.tiltPAPerRad) && std::isfinite(config.gyroDampingAPerRadPerSec) &&
         std::isfinite(config.yawPAPerRadPerSec) && std::isfinite(config.reserveLowA) &&
         config.reserveLowA > 0.0f;
}

float nanToZero(float value) { return std::isnan(value) ? 0.0f : value; }

// Moves speed toward target within one step. Growing the magnitude uses accelRate; shrinking it uses
// brakeRate. A reversal brakes to zero and spends the rest of the step accelerating the other way.
float shapeSpeed(float previous, float target, float accelRate, float brakeRate, float dtSec) {
  float speed = previous;
  float timeLeftSec = dtSec;
  const bool shrinking = (speed > 0.0f && target < speed) || (speed < 0.0f && target > speed);
  if (shrinking) {
    // Brake to the target when it lies on this side of zero, otherwise to zero.
    const float goal = speed > 0.0f ? std::max(target, 0.0f) : std::min(target, 0.0f);
    if (brakeRate <= 0.0f) {
      return speed;
    }
    const float brakeTimeSec = std::fabs(speed - goal) / brakeRate;
    if (brakeTimeSec > timeLeftSec) {
      return speed + (goal > speed ? brakeRate : -brakeRate) * timeLeftSec;
    }
    speed = goal;
    timeLeftSec -= brakeTimeSec;
  }
  const float stepSpeed = std::min(std::fabs(target - speed), accelRate * timeLeftSec);
  return speed + (target > speed ? stepSpeed : -stepSpeed);
}
}  // namespace

Demand BalanceController::shapeDemand(Demand previous, Demand requested, float demandScale, float dtSec,
                                      const Config &config) {
  const float scale = std::isnan(demandScale) ? 0.0f : std::clamp(demandScale, 0.0f, 1.0f);
  const float speedCapMps = config.speedCapMps * scale;
  const float yawCapRadPerSec = config.yawRateCapRadPerSec * scale;

  const float speedTarget = std::max(-speedCapMps, std::min(speedCapMps, nanToZero(requested.speedMps)));
  const float yawRate =
      std::max(-yawCapRadPerSec, std::min(yawCapRadPerSec, nanToZero(requested.yawRateRadPerSec)));

  Demand shaped;
  shaped.speedMps = shapeSpeed(previous.speedMps, speedTarget, config.accelCapMps2 * scale,
                               config.brakeCapMps2, dtSec);
  shaped.yawRateRadPerSec = yawRate;
  return shaped;
}

BalanceController::SpeedLoopResult BalanceController::speedLoopStep(float speedIPrevRad, float speedErrorMps,
                                                                    float trimRad, bool downstreamLimited,
                                                                    float dtSec, const Config &config) {
  SpeedLoopResult out;
  out.speedPRad = config.speedKpRadPerMps * speedErrorMps;
  if (!config.speedIntegralEnabled) {
    out.speedIRad = 0.0f;
  } else if (downstreamLimited) {
    out.speedIRad = speedIPrevRad;
  } else {
    const float candidate = speedIPrevRad + config.speedKiRadPerMpsS * speedErrorMps * dtSec;
    const float sum = trimRad + out.speedPRad + candidate;
    // Once the tilt reference is clipped, only an integral step toward the feasible band is accepted, even
    // when one step does not reach it; requiring a full return would deadlock with small steps.
    const bool movesFurtherOut = (sum > config.tiltRefLimitRad && candidate > speedIPrevRad) ||
                                 (sum < -config.tiltRefLimitRad && candidate < speedIPrevRad);
    out.speedIRad = movesFurtherOut ? speedIPrevRad : candidate;
  }

  const float unlimitedRad = trimRad + out.speedPRad + out.speedIRad;
  out.tiltClipped = std::fabs(unlimitedRad) > config.tiltRefLimitRad;
  // Clamping the sum rather than the correction keeps |desired| within the limit exactly, even in float.
  out.desiredTiltRad = std::max(-config.tiltRefLimitRad, std::min(config.tiltRefLimitRad, unlimitedRad));
  return out;
}

float BalanceController::slewTiltRef(float previousRad, float desiredRad, float dtSec, const Config &config) {
  const float maxStepRad = config.tiltRefSlewRadPerSec * dtSec;
  const float differenceRad = desiredRad - previousRad;
  if (std::fabs(differenceRad) <= maxStepRad) {
    return desiredRad;
  }
  return previousRad + (differenceRad > 0.0f ? maxStepRad : -maxStepRad);
}

Allocation BalanceController::allocateEffort(float balanceCommonA, float yawDifferentialA, float leftMinA,
                                             float leftMaxA, float rightMinA, float rightMaxA) {
  Allocation out;
  out.cMin = 0.5f * (leftMinA + rightMinA);
  out.cMax = 0.5f * (leftMaxA + rightMaxA);
  out.c = std::max(out.cMin, std::min(out.cMax, balanceCommonA));
  // left = c - d stays in [leftMin, leftMax] and right = c + d in [rightMin, rightMax].
  out.dMin = std::max(out.c - leftMaxA, rightMinA - out.c);
  out.dMax = std::min(out.c - leftMinA, rightMaxA - out.c);
  // min/max rather than std::clamp: rounding can leave dMin a hair above dMax at a point interval.
  out.d = std::max(out.dMin, std::min(out.dMax, yawDifferentialA));
  // The final clamp removes the last-ulp rounding excess, so each output lies inside its envelope.
  out.leftA = std::max(leftMinA, std::min(leftMaxA, out.c - out.d));
  out.rightA = std::max(rightMinA, std::min(rightMaxA, out.c + out.d));
  out.reserve = std::min(out.cMax - out.c, out.c - out.cMin);
  out.effortClipped = out.c != balanceCommonA;
  out.differentialClipped = out.d != yawDifferentialA;
  // Balance takes priority: the envelope left no room for the requested differential.
  out.forcedDifferential = out.differentialClipped;
  return out;
}

Output BalanceController::update(const Measurements &measurements, const Request &request, const Config &config,
                                 uint32_t nowUs) {
  // The first call only records the switch level, so a switch that is already on at boot is not an edge.
  const bool enableEdge = enableLevelKnown_ && request.enableSwitch && !previousEnableSwitch_;
  enableLevelKnown_ = true;
  previousEnableSwitch_ = request.enableSwitch;

  if (!request.enableSwitch) {
    state_ = State::Disabled;
    fault_ = FaultReason::None;
  } else {
    switch (state_) {
      case State::Disabled:
        if (enableEdge && config.trimValidated && config.fixedPoseReachable && fault_ == FaultReason::None) {
          state_ = State::Arming;
          armingStartUs_ = nowUs;
        }
        break;
      case State::Arming:
        if (armingConditionsMet(measurements, request, config, nowUs)) {
          state_ = State::Run;
          resetRunTracking(config, nowUs);
          checkEnvelopes(measurements, config);  // drivers must not be enabled on an invalid model
        } else if (elapsedLongerThan(nowUs, armingStartUs_, config.armingTimeoutUs)) {
          state_ = State::Disabled;
        }
        break;
      case State::Run:
      case State::Commissioning:  // Run <-> Commissioning transitions arrive in package 4 step 7.
        checkRunning(measurements, request, config, nowUs);
        break;
      case State::Fault:  // Only the switch leaves Fault, and it leads to Disabled, never to Run.
        break;
    }
  }

  if (state_ != State::Run && state_ != State::Commissioning) {
    leftMinA_ = leftMaxA_ = rightMinA_ = rightMaxA_ = 0.0f;
    shapedDemand_ = Demand{};
    trimRad_ = speedPRad_ = speedIRad_ = desiredTiltRad_ = tiltRefRad_ = 0.0f;
    tiltClipped_ = false;
    effortClippedPrev_ = reserveLowPrev_ = false;
    demandScale_ = 1.0f;
  }
  if (state_ != State::Run) {
    allocation_ = Allocation{};  // wheel effort is only commanded in Run until step 7 wires Commissioning
  }

  Output output;
  output.driveEnabled = state_ == State::Run;
  output.state = state_;
  output.fault = fault_;
  output.leftA = allocation_.leftA;
  output.rightA = allocation_.rightA;
  output.diag.c = allocation_.c;
  output.diag.d = allocation_.d;
  output.diag.cMin = allocation_.cMin;
  output.diag.cMax = allocation_.cMax;
  output.diag.dMin = allocation_.dMin;
  output.diag.dMax = allocation_.dMax;
  output.diag.effortClipped = allocation_.effortClipped;
  output.diag.forcedDifferential = allocation_.forcedDifferential;
  output.diag.reserve = allocation_.reserve;
  output.diag.leftMinA = leftMinA_;
  output.diag.leftMaxA = leftMaxA_;
  output.diag.rightMinA = rightMinA_;
  output.diag.rightMaxA = rightMaxA_;
  output.diag.demandScale = demandScale_;
  output.diag.shapedSpeedMps = shapedDemand_.speedMps;
  output.diag.shapedYawRateRadPerSec = shapedDemand_.yawRateRadPerSec;
  output.diag.tiltRefRad = tiltRefRad_;
  output.diag.trimDeg = trimRad_ * kRadiansToDegreesF;
  output.diag.speedP = speedPRad_;
  output.diag.speedI = speedIRad_;
  output.diag.tiltClipped = tiltClipped_;
  return output;
}

void BalanceController::resetRunTracking(const Config &config, uint32_t nowUs) {
  imuTimestampValid_ = false;
  fallBeyond_ = false;
  overspeedActive_ = false;
  shapedDemand_ = Demand{};

  trimRad_ = config.storedTrimDeg * kDegreesToRadiansF;
  speedPRad_ = 0.0f;
  speedIRad_ = 0.0f;
  desiredTiltRad_ = trimRad_;
  tiltRefRad_ = trimRad_;
  tiltClipped_ = false;
  lastSpeedLoopUs_ = nowUs;
  effortClippedPrev_ = reserveLowPrev_ = false;
  allocation_ = Allocation{};
  haveEffortDemand_ = false;
  demandScale_ = 1.0f;
  authorityClipped_ = false;
}

void BalanceController::latchFault(FaultReason reason) {
  state_ = State::Fault;
  fault_ = reason;
  leftMinA_ = leftMaxA_ = rightMinA_ = rightMaxA_ = 0.0f;
}

void BalanceController::checkRunning(const Measurements &measurements, const Request &request,
                                    const Config &config, uint32_t nowUs) {
  // Freshness is checked on every tick, so a stalled IMU goes stale even though its sequence does not change.
  if (isStale(nowUs, measurements.imuReadTimeUs, config.imuMaxAgeUs)) {
    latchFault(FaultReason::StaleImu);
    return;
  }
  if (isStale(nowUs, measurements.wheelReadTimeUs, config.wheelMaxAgeUs)) {
    latchFault(FaultReason::StaleWheels);
    return;
  }
  if (isStale(nowUs, measurements.busReadTimeUs, config.busMaxAgeUs)) {
    latchFault(FaultReason::StaleBus);
    return;
  }
  if (!measurements.imuValid || !measurements.wheelsValid) {
    latchFault(FaultReason::InvalidInput);
    return;
  }

  // Only a new IMU sample feeds dt and the fall timer. An unchanged sequence is not a fresh sample.
  const bool newImuSample = !imuSequenceSeen_ || measurements.imuSequence != lastImuSequence_;
  if (newImuSample) {
    imuSequenceSeen_ = true;
    lastImuSequence_ = measurements.imuSequence;

    if (imuTimestampValid_) {
      const uint32_t dtUs = measurements.imuReadTimeUs - lastImuReadTimeUs_;
      if (dtUs == 0 || dtUs > config.dtMaxUs) {
        latchFault(FaultReason::BadDt);
        return;
      }
      dtSec_ = static_cast<float>(dtUs) * 1e-6f;

      // A stale or invalid motion request is treated as a zero request and ramps down at the brake cap.
      Demand requested;
      if (request.sourceValid && !isStale(nowUs, request.receivedTimeUs, config.requestMaxAgeUs)) {
        requested.speedMps = request.speedMps;
        requested.yawRateRadPerSec = request.yawRateRadPerSec;
      }
      shapedDemand_ = shapeDemand(shapedDemand_, requested, demandScale_, dtSec_, config);

      // The loop runs on the first processed sample at or after its period, and integrates over that elapsed time.
      const uint32_t loopElapsedUs = nowUs - lastSpeedLoopUs_;
      if (loopElapsedUs >= config.speedLoopPeriodUs) {
        const float measuredSpeedMps =
            0.5f * (measurements.leftWheelRadPerSec + measurements.rightWheelRadPerSec) * config.wheelRadiusM;
        const SpeedLoopResult loop = speedLoopStep(speedIRad_, shapedDemand_.speedMps - measuredSpeedMps,
                                                   trimRad_, effortClippedPrev_ || reserveLowPrev_,
                                                   static_cast<float>(loopElapsedUs) * 1e-6f, config);
        speedPRad_ = loop.speedPRad;
        speedIRad_ = loop.speedIRad;
        desiredTiltRad_ = loop.desiredTiltRad;
        tiltClipped_ = loop.tiltClipped;
        lastSpeedLoopUs_ = nowUs;
      }
      tiltRefRad_ = slewTiltRef(tiltRefRad_, desiredTiltRad_, dtSec_, config);
    }
    imuTimestampValid_ = true;
    lastImuReadTimeUs_ = measurements.imuReadTimeUs;

    if (std::fabs(measurements.tiltRad) <= config.fallAngleRad) {
      fallBeyond_ = false;
    } else if (!fallBeyond_) {
      fallBeyond_ = true;
      fallStartUs_ = nowUs;
    } else if (elapsedLongerThan(nowUs, fallStartUs_, config.fallDurationUs)) {
      latchFault(FaultReason::Fall);
      return;
    }
  }

  // Written so that a NaN speed counts as over the limit.
  const bool withinLimit = std::fabs(measurements.leftWheelRadPerSec) <= config.wheelOverspeedRadPerSec &&
                           std::fabs(measurements.rightWheelRadPerSec) <= config.wheelOverspeedRadPerSec;
  if (withinLimit) {
    overspeedActive_ = false;
  } else if (!overspeedActive_) {
    overspeedActive_ = true;
    overspeedStartUs_ = nowUs;
  } else if (elapsedLongerThan(nowUs, overspeedStartUs_, config.overspeedDurationUs)) {
    latchFault(FaultReason::Overspeed);
    return;
  }

  checkEnvelopes(measurements, config);
  if (state_ == State::Run) {
    computeEffort(measurements, config, nowUs, newImuSample);
  }
  // Checked on every tick, so the authority timer runs while the held effort stays clipped.
  if (state_ == State::Run && authorityClipped_ &&
      elapsedLongerThan(nowUs, authorityStartUs_, config.authorityDurationUs)) {
    latchFault(FaultReason::AuthorityExhausted);
  }
}

// The balance and yaw demands are updated on each new IMU sample. The allocation is redone every tick,
// so held demands are always clamped to the envelopes of the current wheel speeds and bus voltage.
void BalanceController::computeEffort(const Measurements &measurements, const Config &config, uint32_t nowUs,
                                      bool newImuSample) {
  if (newImuSample) {
    balanceCommonA_ = config.tiltPAPerRad * (measurements.tiltRad - tiltRefRad_) +
                      config.gyroDampingAPerRadPerSec * measurements.tiltRateRadPerSec;
    yawDifferentialA_ = config.yawPAPerRadPerSec * (shapedDemand_.yawRateRadPerSec - measurements.yawRateRadPerSec);
    // A NaN or infinite input would reach the motor targets through the allocation, so it faults instead.
    if (!std::isfinite(balanceCommonA_) || !std::isfinite(yawDifferentialA_)) {
      latchFault(FaultReason::InvalidInput);
      return;
    }
    haveEffortDemand_ = true;
  }
  if (!haveEffortDemand_) {
    return;
  }

  allocation_ = allocateEffort(balanceCommonA_, yawDifferentialA_, leftMinA_, leftMaxA_, rightMinA_, rightMaxA_);
  effortClippedPrev_ = allocation_.effortClipped;
  reserveLowPrev_ = allocation_.reserve < config.reserveLowA;
  demandScale_ = std::clamp(allocation_.reserve / config.reserveLowA, 0.0f, 1.0f);

  if (!allocation_.effortClipped) {
    authorityClipped_ = false;
  } else if (!authorityClipped_) {
    authorityClipped_ = true;
    authorityStartUs_ = nowUs;
  }
}

void BalanceController::checkEnvelopes(const Measurements &measurements, const Config &config) {
  if (!configValid(config)) {
    latchFault(FaultReason::InvalidConfig);
    return;
  }
  const float usableV = (measurements.busVoltageV - config.sagMarginV) * config.modulationFactor;
  const float vMaxV = std::min(config.voltageLimitV, usableV);
  // Written so that a NaN input fails the check.
  const bool modelValid = std::isfinite(usableV) && std::isfinite(vMaxV) && vMaxV > 0.0f &&
                          measurements.busVoltageV >= config.busMinV &&
                          measurements.busVoltageV <= config.busMaxV && config.kvRating > 0.0f &&
                          config.phaseResistanceOhm > 0.0f && isUnitSign(config.leftWheelSign) &&
                          isUnitSign(config.rightWheelSign);
  if (!modelValid) {
    latchFault(FaultReason::InvalidEnvelope);
    return;
  }

  const WheelInterval left =
      wheelInterval(measurements.leftWheelRadPerSec, config.leftWheelSign, vMaxV, config);
  const WheelInterval right =
      wheelInterval(measurements.rightWheelRadPerSec, config.rightWheelSign, vMaxV, config);
  leftMinA_ = left.minA;
  leftMaxA_ = left.maxA;
  rightMinA_ = right.minA;
  rightMaxA_ = right.maxA;

  const bool valid = left.finite && right.finite && left.minA <= left.maxA && right.minA <= right.maxA;
  if (!valid) {
    latchFault(FaultReason::InvalidEnvelope);
  }
}
