#include "RobotConfig.h"

#include <algorithm>

#include "MathConstants.h"
#include "OverspeedGuard.h"
#include "WheelBoundary.h"

Config defaultBalanceConfig() {
  Config c;
  // Arming
  c.armingTimeoutUs = 3000000;                 // uncommissioned: proposed starting value, 3 s
  c.armingTiltWindowRad = 0.0872665f;          // uncommissioned: proposed starting value, rad (5 deg)
  c.armingTiltRateRadPerSec = 0.1f;            // uncommissioned: proposed starting value, rad/s
  c.armingRequestSpeedMps = 0.05f;             // uncommissioned: proposed starting value, m/s
  c.armingRequestYawRadPerSec = 0.05f;         // uncommissioned: proposed starting value, rad/s
  c.armingWheelSpeedRadPerSec = 2.0f;          // uncommissioned: proposed starting value, rad/s per wheel

  // Measurement freshness and loop gap
  c.imuMaxAgeUs = 20000;                       // uncommissioned: proposed starting value, 20 ms
  c.wheelMaxAgeUs = 20000;                     // uncommissioned: proposed starting value, 20 ms
  c.busMaxAgeUs = 100000;                      // uncommissioned: proposed starting value, 100 ms
  c.dtMaxUs = 20000;                           // uncommissioned: proposed starting value, 20 ms

  // Fall: the 35 deg tumble threshold of the former fall detector (existing constant)
  c.fallAngleRad = 35.0f * kDegreesToRadiansF;  // existing tumble threshold, rad
  c.fallDurationUs = 20000;                    // upstream: 20 control cycles at the nominal 1 kHz gate, 20 ms

  // Overspeed: the existing OverspeedGuard limit and debounce
  c.wheelOverspeedRadPerSec = OverspeedGuard::kLimitRadPerSec;   // existing constant, rad/s
  c.overspeedDurationUs = OverspeedGuard::kDebounceMs * 1000U;   // existing constant, us

  // Effort envelope. One KV is used for both wheels: 76.7 (right) and 79.5 (left) measured, about 3.6 percent apart,
  // so the envelope error is small. The lower value is used.
  c.phaseResistanceOhm = kLeftWheelMotor.phaseResistanceOhm;     // existing constant, ohm per phase (5.75 not verified)
  // Same rating as handed to SimpleFOC, so the envelope matches the voltage the drive really outputs. 
  c.kvRating = std::min(kLeftWheelMotor.kvRatingRpmPerV, kRightWheelMotor.kvRatingRpmPerV) * kWheelKvSafetyFactor;  // rpm/V
  c.voltageLimitV = kWheelVoltageLimitV;                         // existing constant, V
  c.currentLimitA = kWheelVoltageLimitV / c.phaseResistanceOhm;  // as configureWheelMotor(), A
  c.sagMarginV = 0.5f;                         // uncommissioned: proposed starting value, V
  c.modulationFactor = 0.57735027f;            // plan: SpaceVectorPWM usable amplitude is bus / sqrt(3)
  // Motor speed = sign * forward speed. Forward is negative motor-local on both wheels (WheelBoundary.h).
  c.leftWheelSign = static_cast<int>(WheelBoundary::kLeftMotorSign);
  c.rightWheelSign = static_cast<int>(WheelBoundary::kRightMotorSign);
  c.busMinV = 6.0f;                            // uncommissioned: proposed starting value, V
  c.busMaxV = 9.0f;                            // uncommissioned: proposed starting value, V

  // Demand shaping
  c.speedCapMps = 0.3f;                        // uncommissioned: proposed starting value, m/s
  c.accelCapMps2 = 0.5f;                       // uncommissioned: proposed starting value, m/s^2
  c.brakeCapMps2 = 1.0f;                       // uncommissioned: proposed starting value, m/s^2
  c.yawRateCapRadPerSec = 1.0f;                // uncommissioned: doubled after the stick test 2026-10-24, rad/s
  c.requestMaxAgeUs = kRcFrameTimeoutMs * 1000U;  // existing constant (RC frame timeout), us
  c.demandScaleCapStart = 0.5f;                // uncommissioned: proposed starting value, dimensionless

  // Speed loop (gains zero, integral off)
  c.wheelRadiusM = 0.019f;                     // measured 2026-10-24: diameter 38.3 mm, 38 mm used, m
  c.speedLoopPeriodUs = 20000;                 // plan: provisional 50 Hz speed loop, us
  c.speedKpRadPerMps = 0.25f;                  // uncommissioned: tilt rad per m/s (0.2 m/s error asks 0.05 rad)
  c.speedKiRadPerMpsS = 0.08f;                 // uncommissioned: tilt rad per m (0.1 m/s error for 1 s adds 0.008 rad)
  c.speedIntegralEnabled = true;               // uncommissioned: on, learns the lean that cancels an off-centre centre of mass
  c.tiltRefLimitRad = 0.0872665f;              // uncommissioned: proposed starting value, rad (5 deg)
  c.tiltRefSlewRadPerSec = 0.5f;               // uncommissioned: proposed starting value, rad/s
  c.tiltClipDurationUs = 500000;               // uncommissioned: proposed starting value, 0.5 s

  // Trim and standstill commissioning
  c.trimEnvelopeRad = 0.0523599f;              // uncommissioned: proposed starting value, rad (3 deg)
  c.trimSlewRadPerSec = 0.05f;                 // uncommissioned: proposed starting value, rad/s
  c.knobPickupBandRad = 0.01f;                 // uncommissioned: proposed starting value, rad
  c.standstillWheelSpeedTolMps = 0.02f;        // plan: average standstill wheel speed below 0.02 m/s
  c.standstillTiltErrorTolRad = 0.01f;         // uncommissioned: proposed starting value, rad
  c.transferToleranceRad = 0.001f;             // uncommissioned: proposed starting value, rad
  c.saveRequestSpeedTolMps = 0.02f;            // uncommissioned: proposed starting value, m/s
  c.saveRequestYawTolRadPerSec = 0.05f;        // uncommissioned: proposed starting value, rad/s

  // Balance and yaw law
  c.tiltPAPerRad = 8.0f;                       // uncommissioned: A/rad (5 deg lean asks 0.70 A)
  c.gyroDampingAPerRadPerSec = 0.4f;           // uncommissioned: A/(rad/s) (1 rad/s asks 0.4 A)
  c.yawPAPerRadPerSec = 0.6f;                  // uncommissioned: A/(rad/s) (1 rad/s yaw error asks 0.6 A per side)

  // Reserve and authority
  c.reserveLowA = 0.2f;                        // uncommissioned: proposed starting value, A
  c.authorityDurationUs = 2000000;             // uncommissioned: proposed starting value, 2 s

  // Sketch-supplied inputs: first commissioning, trim taken as validated at zero (user decision 2026-10-22)
  c.trimValidated = true;
  c.storedTrimDeg = 0.0f;                      // deg
  c.fixedPoseReachable = true;                 // plan: fixed pose, all joint angles 0
  return c;
}
