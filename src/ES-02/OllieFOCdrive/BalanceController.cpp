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

// The motion request counts only while its source is valid and it is not stale. Shaping, the commissioning
// switch and the save switch all read the request through this predicate.
bool requestLost(const Request &request, const Config &config, uint32_t nowUs) {
  return !request.sourceValid || isStale(nowUs, request.receivedTimeUs, config.requestMaxAgeUs);
}

// Moves value toward target by at most maxStep, landing exactly on target.
float approach(float previous, float target, float maxStep) {
  const float difference = target - previous;
  if (std::fabs(difference) <= maxStep) {
    return target;
  }
  return previous + (difference > 0.0f ? maxStep : -maxStep);
}

// Forward speed of the chassis from the mean of both wheels.
float measuredSpeedMps(const Measurements &measurements, const Config &config) {
  return 0.5f * (measurements.leftWheelRadPerSec + measurements.rightWheelRadPerSec) * config.wheelRadiusM;
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
         std::isfinite(config.demandScaleCapStart) && config.demandScaleCapStart > 0.0f &&
         config.demandScaleCapStart < 1.0f &&
         std::isfinite(config.brakeCapMps2) && config.brakeCapMps2 > 0.0f &&
         std::isfinite(config.yawRateCapRadPerSec) && config.yawRateCapRadPerSec >= 0.0f &&
         std::isfinite(config.wheelRadiusM) && config.wheelRadiusM > 0.0f &&
         config.speedLoopPeriodUs > 0 && std::isfinite(config.speedKpRadPerMps) &&
         std::isfinite(config.speedKiRadPerMpsS) && std::isfinite(config.tiltRefLimitRad) &&
         config.tiltRefLimitRad >= 0.0f && std::isfinite(config.tiltRefSlewRadPerSec) &&
         config.tiltRefSlewRadPerSec > 0.0f && config.tiltClipDurationUs > 0 &&
         std::isfinite(config.trimEnvelopeRad) &&
         config.trimEnvelopeRad > 0.0f && config.trimEnvelopeRad <= config.tiltRefLimitRad &&
         std::isfinite(config.trimSlewRadPerSec) && config.trimSlewRadPerSec > 0.0f &&
         std::isfinite(config.knobPickupBandRad) && config.knobPickupBandRad > 0.0f &&
         std::isfinite(config.standstillWheelSpeedTolMps) && config.standstillWheelSpeedTolMps > 0.0f &&
         std::isfinite(config.standstillTiltErrorTolRad) && config.standstillTiltErrorTolRad > 0.0f &&
         std::isfinite(config.transferToleranceRad) && config.transferToleranceRad > 0.0f &&
         std::isfinite(config.saveRequestSpeedTolMps) && config.saveRequestSpeedTolMps >= 0.0f &&
         std::isfinite(config.saveRequestYawTolRadPerSec) && config.saveRequestYawTolRadPerSec >= 0.0f &&
         std::fabs(config.storedTrimDeg * kDegreesToRadiansF) <= config.trimEnvelopeRad &&
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
  // Acceleration and yaw give way first, from 1 at scale 1 to 0 at demandScaleCapStart. The cruise cap holds
  // until scale falls below demandScaleCapStart, then falls to 0 at scale 0.
  const float accelFactor = std::clamp((scale - config.demandScaleCapStart) / (1.0f - config.demandScaleCapStart),
                                       0.0f, 1.0f);
  const float capFactor = std::clamp(scale / config.demandScaleCapStart, 0.0f, 1.0f);
  const float speedCapMps = config.speedCapMps * capFactor;
  const float yawCapRadPerSec = config.yawRateCapRadPerSec * accelFactor;

  const float speedTarget = std::max(-speedCapMps, std::min(speedCapMps, nanToZero(requested.speedMps)));
  const float yawRate =
      std::max(-yawCapRadPerSec, std::min(yawCapRadPerSec, nanToZero(requested.yawRateRadPerSec)));

  Demand shaped;
  shaped.speedMps = shapeSpeed(previous.speedMps, speedTarget, config.accelCapMps2 * accelFactor,
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
  return approach(previousRad, desiredRad, config.tiltRefSlewRadPerSec * dtSec);
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
  commissioningRefused_ = false;
  // Level tracked on every call. The edge counts only when the state is Commissioning on this tick and was
  // Commissioning on the previous one, so a level present at entry never counts.
  const bool lost = requestLost(request, config, nowUs);
  // Save edges come only from valid frames. A lost frame ends tracking; the first valid frame after it rebases the
  // level, so a switch that moved while RC was lost does not save when RC returns.
  bool saveEdge = false;
  if (lost) {
    saveTracked_ = false;
  } else {
    saveEdge = saveTracked_ && request.saveSwitch && !previousSaveSwitch_;
    saveTracked_ = true;
    previousSaveSwitch_ = request.saveSwitch;
  }
  const bool wasCommissioning = state_ == State::Commissioning;

  if (!request.enableSwitch) {
    state_ = State::Disabled;
    fault_ = FaultReason::None;
  } else {
    switch (state_) {
      case State::Disabled:
        if (enableEdge && config.trimValidated && config.fixedPoseReachable && fault_ == FaultReason::None && !lost) {
          state_ = State::Arming;
          armingStartUs_ = nowUs;
        }
        break;
      case State::Arming:
        // A lost request holds Arming; the arming timeout below still applies.
        if (!lost && armingConditionsMet(measurements, request, config, nowUs)) {
          state_ = State::Run;
          resetRunTracking(config, nowUs);
          checkEnvelopes(measurements, config);  // drivers must not be enabled on an invalid model
        } else if (elapsedLongerThan(nowUs, armingStartUs_, config.armingTimeoutUs)) {
          state_ = State::Disabled;
        }
        break;
      case State::Run:  // Run -> Commissioning is decided inside checkRunning, on a processed IMU sample.
        checkRunning(measurements, request, config, nowUs);
        break;
      case State::Commissioning:
        // Commissioning ends only with a live request, the switch off, a validated trim, and the working trim equal
        // to the stored one. An unsaved change keeps Commissioning.
        if (!lost && !request.commissioningSwitch && config.trimValidated &&
            std::fabs(trimRad_ - config.storedTrimDeg * kDegreesToRadiansF) <= config.transferToleranceRad) {
          state_ = State::Run;
          knobEngaged_ = false;
        }
        checkRunning(measurements, request, config, nowUs);
        break;
      case State::Fault:  // Only the switch leaves Fault, and it leads to Disabled, never to Run.
        break;
    }
  }

  // A save is granted or refused on the edge, on the tick it occurs. Only the pulse is output; storage is outside.
  bool saveGranted = false;
  bool saveRefused = false;
  if (wasCommissioning && saveEdge && state_ == State::Commissioning) {
    saveGranted = saveAllowed(measurements, request, config, lost);
    saveRefused = !saveGranted;
  }

  if (!driving()) {
    leftMinA_ = leftMaxA_ = rightMinA_ = rightMaxA_ = 0.0f;
    shapedDemand_ = Demand{};
    trimRad_ = speedPRad_ = speedIRad_ = desiredTiltRad_ = tiltRefRad_ = 0.0f;
    tiltClipped_ = false;
    effortClippedPrev_ = reserveLowPrev_ = voltageOrCurrentClippedPrev_ = false;
    demandScale_ = 1.0f;
    knobEngaged_ = false;
    allocation_ = Allocation{};
  }

  Output output;
  output.driveEnabled = driving();
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
  output.diag.voltageClipped = driving() && measurements.voltageClipped;
  output.diag.currentClipped = driving() && measurements.currentClipped;
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
  output.diag.knobEngaged = knobEngaged_;
  output.diag.commissioningRefused = commissioningRefused_;
  output.diag.saveRefused = saveRefused;
  output.saveTrimRequested = saveGranted;
  output.trimToSaveDeg = saveGranted ? trimRad_ * kRadiansToDegreesF : 0.0f;
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
  effortClippedPrev_ = reserveLowPrev_ = voltageOrCurrentClippedPrev_ = false;
  allocation_ = Allocation{};
  haveEffortDemand_ = false;
  demandScale_ = 1.0f;
  authorityClipped_ = false;
  knobEngaged_ = false;
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

  // Request loss uses the same predicate as demand shaping. Losing it disengages the knob; the trim is held.
  const bool rcLost = requestLost(request, config, nowUs);
  const bool knobUsable = !rcLost && request.knobValid && std::isfinite(request.knobTrimDeg);
  if (!knobUsable) {
    knobEngaged_ = false;
  }
  // Commissioning, and the commissioning switch in Run, hold the requested speed and yaw at zero.
  const bool zeroDemand = state_ == State::Commissioning || request.commissioningSwitch;

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
      if (!zeroDemand && !rcLost) {
        requested.speedMps = request.speedMps;
        requested.yawRateRadPerSec = request.yawRateRadPerSec;
      }
      shapedDemand_ = shapeDemand(shapedDemand_, requested, demandScale_, dtSec_, config);

      // The loop runs on the first processed sample at or after its period, and integrates over that elapsed time.
      const uint32_t loopElapsedUs = nowUs - lastSpeedLoopUs_;
      if (loopElapsedUs >= config.speedLoopPeriodUs) {
        // Commissioning holds the speed integral at zero; it runs only in Run.
        Config loopConfig = config;
        loopConfig.speedIntegralEnabled = config.speedIntegralEnabled && state_ == State::Run;
        const SpeedLoopResult loop =
            speedLoopStep(speedIRad_, shapedDemand_.speedMps - measuredSpeedMps(measurements, config), trimRad_,
                          effortClippedPrev_ || reserveLowPrev_ || voltageOrCurrentClippedPrev_,
                          static_cast<float>(loopElapsedUs) * 1e-6f, loopConfig);
        speedPRad_ = loop.speedPRad;
        speedIRad_ = loop.speedIRad;
        desiredTiltRad_ = loop.desiredTiltRad;
        tiltClipped_ = loop.tiltClipped;
        lastSpeedLoopUs_ = nowUs;
      }
      tiltRefRad_ = slewTiltRef(tiltRefRad_, desiredTiltRad_, dtSec_, config);

      if (state_ == State::Run) {
        // A lost request does not count as the commissioning switch, so Run stays and the entry is flagged refused.
        if (request.commissioningSwitch && (rcLost || !tryEnterCommissioning(measurements, request, config))) {
          commissioningRefused_ = true;
        }
      } else if (knobUsable && state_ == State::Commissioning) {
        stepKnob(request, config);
      }
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
  if (driving()) {
    computeEffort(measurements, config, nowUs, newImuSample);
  }
  // Checked on every tick, so the authority timer runs while the held effort stays clipped.
  if (driving() && authorityClipped_ &&
      elapsedLongerThan(nowUs, authorityStartUs_, config.authorityDurationUs)) {
    latchFault(FaultReason::AuthorityExhausted);
  }
  // Same timing for a tilt reference held beyond its limit. Not driving means a fault was latched on this tick,
  // and that fault keeps its reason.
  if (!driving() || !tiltClipped_) {
    tiltClipRunning_ = false;
  } else if (!tiltClipRunning_) {
    tiltClipRunning_ = true;
    tiltClipStartUs_ = nowUs;
  } else if (elapsedLongerThan(nowUs, tiltClipStartUs_, config.tiltClipDurationUs)) {
    latchFault(FaultReason::TiltSaturated);
  }
}

// Run -> Commissioning, on one processed IMU sample. Changes nothing unless every check holds.
bool BalanceController::tryEnterCommissioning(const Measurements &measurements, const Request &request,
                                              const Config &config) {
  const float speedMps = measuredSpeedMps(measurements, config);
  // The raw stick must be near neutral, not just the shaped demand: shaping ramps to zero under the switch.
  // No ongoing clip or slew, and the previous tick had neither effort nor reserve limits.
  const bool settled = std::fabs(request.speedMps) <= config.saveRequestSpeedTolMps &&
                       std::fabs(request.yawRateRadPerSec) <= config.saveRequestYawTolRadPerSec && !knobEngaged_ &&
                       shapedDemand_.speedMps == 0.0f && shapedDemand_.yawRateRadPerSec == 0.0f &&
                       std::fabs(speedMps) <= config.standstillWheelSpeedTolMps &&
                       std::fabs(measurements.tiltRad - tiltRefRad_) <= config.standstillTiltErrorTolRad &&
                       !tiltClipped_ && !effortClippedPrev_ && !reserveLowPrev_ && !measurements.voltageClipped &&
                       !measurements.currentClipped && tiltRefRad_ == desiredTiltRad_;
  if (!settled) {
    return false;
  }

  const float trimCandidateRad = trimRad_ + speedIRad_;
  // The speed P term is recomputed at this tick. A stored P from an earlier loop run would make the
  // reference jump by its change since then, and the check below refuses that.
  const float speedPRad = config.speedKpRadPerMps * (shapedDemand_.speedMps - speedMps);
  const float unclampedRad = trimCandidateRad + speedPRad;
  const float limitRad = config.tiltRefLimitRad;
  const float proposedRad = std::max(-limitRad, std::min(limitRad, unclampedRad));
  const bool holds = std::fabs(trimCandidateRad) <= config.trimEnvelopeRad && std::fabs(unclampedRad) <= limitRad &&
                     std::fabs(proposedRad - tiltRefRad_) <= config.transferToleranceRad;
  if (!holds) {
    return false;
  }

  speedIRad_ = 0.0f;
  trimRad_ = trimCandidateRad;
  speedPRad_ = speedPRad;
  desiredTiltRad_ = proposedRad;
  knobEngaged_ = false;
  state_ = State::Commissioning;
  return true;
}

// Save check, run on a save edge in Commissioning. Every condition must hold. The trim is saved as it is now, so
// a trim still slewing toward a knob target is refused.
bool BalanceController::saveAllowed(const Measurements &measurements, const Request &request, const Config &config,
                                    bool lost) const {
  const float speedMps = measuredSpeedMps(measurements, config);
  const float knobTargetRad = std::max(-config.trimEnvelopeRad,
                                       std::min(config.trimEnvelopeRad, request.knobTrimDeg * kDegreesToRadiansF));
  const bool knobPending = knobEngaged_ && knobTargetRad != trimRad_;
  return !lost && speedIRad_ == 0.0f && shapedDemand_.speedMps == 0.0f &&
         shapedDemand_.yawRateRadPerSec == 0.0f && std::fabs(request.speedMps) <= config.saveRequestSpeedTolMps &&
         std::fabs(request.yawRateRadPerSec) <= config.saveRequestYawTolRadPerSec &&
         std::fabs(speedMps) <= config.standstillWheelSpeedTolMps &&
         std::fabs(measurements.tiltRad - tiltRefRad_) <= config.standstillTiltErrorTolRad && !tiltClipped_ &&
         !allocation_.effortClipped && allocation_.reserve >= config.reserveLowA && !measurements.voltageClipped &&
         !measurements.currentClipped && tiltRefRad_ == desiredTiltRad_ && !knobPending;
}

// Commissioning only. Pickup needs the knob inside the band around the working trim; once engaged, the trim
// follows the knob at the slew rate, clamped to the envelope.
void BalanceController::stepKnob(const Request &request, const Config &config) {
  const float knobRad = request.knobTrimDeg * kDegreesToRadiansF;
  if (!knobEngaged_ && std::fabs(knobRad - trimRad_) < config.knobPickupBandRad) {
    knobEngaged_ = true;
  }
  if (knobEngaged_) {
    const float targetRad = std::max(-config.trimEnvelopeRad, std::min(config.trimEnvelopeRad, knobRad));
    trimRad_ = approach(trimRad_, targetRad, config.trimSlewRadPerSec * dtSec_);
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
  voltageOrCurrentClippedPrev_ = measurements.voltageClipped || measurements.currentClipped;
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
