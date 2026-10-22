#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
#include <limits>

#include "../../src/ES-02/OllieFOCdrive/BalanceController.h"

namespace {
// Stored trim of 10 degrees, expressed in radians by the test (not via the module).
constexpr float kTrimTiltRad = 0.1745329f;
constexpr uint32_t kTimeoutUs = 1000000;

Config makeConfig() {
  Config config;
  config.armingTimeoutUs = kTimeoutUs;
  config.armingTiltWindowRad = 0.05f;
  config.armingTiltRateRadPerSec = 0.1f;
  config.armingRequestSpeedMps = 0.05f;
  config.armingRequestYawRadPerSec = 0.05f;
  config.armingWheelSpeedRadPerSec = 5.0f;  // test value
  config.imuMaxAgeUs = 20000;
  config.wheelMaxAgeUs = 20000;
  config.busMaxAgeUs = 100000;
  config.dtMaxUs = 50000;
  config.fallAngleRad = 0.6f;
  config.fallDurationUs = 500000;
  config.wheelOverspeedRadPerSec = 100.0f;
  config.overspeedDurationUs = 200000;
  config.trimValidated = true;
  config.storedTrimDeg = 10.0f;
  config.fixedPoseReachable = true;
  // Envelope: R 22 ohm, KV 100 rpm/V, so bemf at 100 rad/s is 5.5133 V (100 / (100 * sqrt(3)) / 0.10472).
  config.phaseResistanceOhm = 22.0f;
  config.kvRating = 100.0f;
  config.voltageLimitV = 8.4f;
  config.currentLimitA = 5.0f;
  config.sagMarginV = 0.4f;
  config.modulationFactor = 1.0f;
  config.leftWheelSign = 1;
  config.rightWheelSign = 1;
  config.busMinV = 6.0f;
  config.busMaxV = 30.0f;
  // Demand shaping: 1 m/s cap, 2 m/s^2 accel, 4 m/s^2 brake, 0.5 rad/s yaw cap, 50 ms request age.
  config.speedCapMps = 1.0f;
  config.accelCapMps2 = 2.0f;
  config.brakeCapMps2 = 4.0f;
  config.yawRateCapRadPerSec = 0.5f;
  config.requestMaxAgeUs = 50000;
  // Speed loop: 0.1 m wheel radius, 50 Hz, Kp 0.5 rad per m/s, integral off by default, tilt limit 0.3 rad
  // (above the 10 degree trim), slew 1 rad/s.
  config.wheelRadiusM = 0.1f;
  config.speedLoopPeriodUs = 20000;
  config.speedKpRadPerMps = 0.5f;
  config.speedKiRadPerMpsS = 0.2f;
  config.speedIntegralEnabled = false;
  config.tiltRefLimitRad = 0.3f;
  config.tiltRefSlewRadPerSec = 1.0f;
  // Tilt clip duration: 1 s, longer than any test in this file holds a clipped tilt reference.
  config.tiltClipDurationUs = 1000000;
  // Balance and reserve. The balance and yaw gains stay zero here, so the earlier steps see no effort.
  // reserveLowA is a test value: at standstill the envelope is about +/-0.381 A, so the reserve (0.381 A)
  // stays above it and demandScale stays 1.
  config.reserveLowA = 0.1f;
  config.authorityDurationUs = 100000;
  // Trim (step 7): envelope 0.2 rad (above the 10 degree stored trim, below the 0.3 rad tilt limit), knob slew
  // 0.1 rad/s, pickup band 0.02 rad, standstill tolerances 0.01 m/s and 0.01 rad, transfer tolerance 1e-4 rad.
  config.trimEnvelopeRad = 0.2f;
  config.trimSlewRadPerSec = 0.1f;
  config.knobPickupBandRad = 0.02f;
  config.standstillWheelSpeedTolMps = 0.01f;
  config.standstillTiltErrorTolRad = 0.01f;
  config.transferToleranceRad = 1e-4f;
  // Save and transfer request bands, deliberately different from the 0.05 arming bands: 0.02 m/s and 0.03 rad/s.
  config.saveRequestSpeedTolMps = 0.02f;
  config.saveRequestYawTolRadPerSec = 0.03f;
  // Demand scaling: acceleration and yaw give way from demandScale 1 down to 0.5; the cruise cap falls below 0.5.
  config.demandScaleCapStart = 0.5f;
  return config;
}

bool closeTo(float actual, float expected) { return std::fabs(actual - expected) <= 1e-4f; }

bool nearlyTo(float actual, float expected, float tolerance) { return std::fabs(actual - expected) <= tolerance; }

Measurements makeMeasurements() {
  Measurements m{};
  m.imuSequence = 1;
  m.imuReadTimeUs = 0;
  m.imuValid = true;
  m.tiltRad = kTrimTiltRad;
  m.tiltRateRadPerSec = 0.0f;
  m.yawRateRadPerSec = 0.0f;
  m.wheelReadTimeUs = 0;
  m.wheelsValid = true;
  m.leftWheelRadPerSec = 0.0f;
  m.rightWheelRadPerSec = 0.0f;
  m.busReadTimeUs = 0;
  m.busVoltageV = 24.0f;
  return m;
}

// Sets every acquisition timestamp to nowUs, as fresh sensors would.
Measurements atTime(Measurements m, uint32_t nowUs) {
  m.imuReadTimeUs = nowUs;
  m.wheelReadTimeUs = nowUs;
  m.busReadTimeUs = nowUs;
  return m;
}

Request makeRequest(bool enableSwitch) {
  Request r{};
  r.receivedTimeUs = 0;
  r.sourceValid = true;
  r.speedMps = 0.0f;
  r.yawRateRadPerSec = 0.0f;
  r.enableSwitch = enableSwitch;
  r.commissioningSwitch = false;
  r.saveSwitch = false;
  r.knobValid = false;
  r.knobTrimDeg = 0.0f;
  return r;
}

Output tick(BalanceController &controller, const Measurements &m, const Request &r, const Config &c,
            uint32_t nowUs) {
  return controller.update(atTime(m, nowUs), r, c, nowUs);
}

// A new IMU sample taken at nowUs with the given sequence; the other sensors are fresh.
Output sample(BalanceController &controller, const Measurements &m, const Config &c, uint32_t nowUs,
              uint32_t imuSequence) {
  Measurements s = atTime(m, nowUs);
  s.imuSequence = imuSequence;
  return controller.update(s, makeRequest(true), c, nowUs);
}

// Off, then on (Arming), then Run once the arming conditions hold. Run is entered at startUs + 2000.
// A request with the given switch, received at nowUs, so it is live at nowUs.
Request liveSwitch(uint32_t nowUs, bool enableSwitch) {
  Request r = makeRequest(enableSwitch);
  r.receivedTimeUs = nowUs;
  return r;
}

void enterRun(BalanceController &controller, const Config &config, uint32_t startUs) {
  tick(controller, makeMeasurements(), liveSwitch(startUs, false), config, startUs);
  tick(controller, makeMeasurements(), liveSwitch(startUs + 1000, true), config, startUs + 1000);
  assert(tick(controller, makeMeasurements(), liveSwitch(startUs + 2000, true), config, startUs + 2000).state ==
         State::Run);
}

Demand shape(Demand previous, float speedMps, float yawRadPerSec, float demandScale, float dtSec,
             const Config &config) {
  return BalanceController::shapeDemand(previous, Demand{speedMps, yawRadPerSec}, demandScale, dtSec, config);
}

Request liveRequest(uint32_t nowUs, float speedMps, float yawRadPerSec) {
  Request r = makeRequest(true);
  r.receivedTimeUs = nowUs;
  r.speedMps = speedMps;
  r.yawRateRadPerSec = yawRadPerSec;
  return r;
}

// A new IMU sample at nowUs with the given sequence; the other sensors are fresh.
Output liveSample(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence,
                  const Request &request) {
  Measurements s = atTime(makeMeasurements(), nowUs);
  s.imuSequence = imuSequence;
  return controller.update(s, request, config, nowUs);
}

// A new IMU sample at nowUs with the given wheel speeds and a zero request; the other sensors are fresh.
Output wheelSample(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence,
                   float leftRadPerSec, float rightRadPerSec) {
  Measurements s = atTime(makeMeasurements(), nowUs);
  s.imuSequence = imuSequence;
  s.leftWheelRadPerSec = leftRadPerSec;
  s.rightWheelRadPerSec = rightRadPerSec;
  return controller.update(s, makeRequest(true), config, nowUs);
}

// Enters Run at baseUs, then samples a 1 m/s and 2 rad/s request every 50 ms. The first sample only
// records its timestamp. Each of the next five raises the shaped speed by 0.1 m/s and clamps yaw to 0.5,
// so the last sample (at baseUs + 360000 us, IMU sequence 7) shows 0.5 m/s.
Output rampToHalfMps(BalanceController &controller, const Config &config, uint32_t baseUs) {
  enterRun(controller, config, baseUs);
  liveSample(controller, config, baseUs + 110000u, 2, liveRequest(baseUs + 110000u, 1.0f, 2.0f));
  Output last;
  for (uint32_t k = 1; k <= 5; ++k) {
    const uint32_t t = baseUs + 110000u + 50000u * k;
    last = liveSample(controller, config, t, 2 + k, liveRequest(t, 1.0f, 2.0f));
  }
  return last;
}

// Step 6 config. With R 1 ohm and a 20 A limit, the 8.4 V voltage limit alone bounds each wheel at standstill
// to [-8.4, 8.4] A. The speed loop gain is zero, so the tilt reference stays at the trim unless a test sets it.
Config makeBalanceConfig() {
  Config config = makeConfig();
  config.phaseResistanceOhm = 1.0f;
  config.currentLimitA = 20.0f;
  config.tiltPAPerRad = 2.0f;
  config.gyroDampingAPerRadPerSec = 0.5f;
  config.yawPAPerRadPerSec = 1.0f;
  config.reserveLowA = 1.0f;
  config.authorityDurationUs = 100000;
  config.speedKpRadPerMps = 0.0f;
  return config;
}

struct ImuValues {
  float tiltErrorRad = 0.0f;  // relative to the trim, which is the tilt reference at zero speed
  float tiltRateRadPerSec = 0.0f;
  float yawRateRadPerSec = 0.0f;
  float leftRadPerSec = 0.0f;
  float rightRadPerSec = 0.0f;
};

// A new IMU sample at nowUs; the bus is fresh.
Output imuSample(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence,
                 const ImuValues &values, const Request &request) {
  Measurements m = atTime(makeMeasurements(), nowUs);
  m.imuSequence = imuSequence;
  m.tiltRad = kTrimTiltRad + values.tiltErrorRad;
  m.tiltRateRadPerSec = values.tiltRateRadPerSec;
  m.yawRateRadPerSec = values.yawRateRadPerSec;
  m.leftWheelRadPerSec = values.leftRadPerSec;
  m.rightWheelRadPerSec = values.rightRadPerSec;
  return controller.update(m, request, config, nowUs);
}

// The same IMU sequence again at nowUs: no new sample, so only freshness and the fault timers advance.
Output heldTick(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence) {
  Measurements m = atTime(makeMeasurements(), nowUs);
  m.imuSequence = imuSequence;
  return controller.update(m, makeRequest(true), config, nowUs);
}

// A sample (newSample) or a held tick at nowUs with the wheels at wheelRadPerSec on both sides, a live zero request
// and the given effort-path clip flags.
Output wheelTick(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence,
                 float wheelRadPerSec, bool voltageClipped = false, bool currentClipped = false) {
  Measurements m = atTime(makeMeasurements(), nowUs);
  m.imuSequence = imuSequence;
  m.leftWheelRadPerSec = wheelRadPerSec;
  m.rightWheelRadPerSec = wheelRadPerSec;
  m.voltageClipped = voltageClipped;
  m.currentClipped = currentClipped;
  return controller.update(m, liveRequest(nowUs, 0.0f, 0.0f), config, nowUs);
}

// Step 7. Degree and radian conversions for the test values, written out here rather than taken from the module.
constexpr float kDegToRad = 0.0174532925f;
constexpr float kRadToDeg = 57.2957795f;

// Step 7 config: the balance config with the speed loop on (Kp 0.5 rad per m/s, Ki 2 rad per m/s per s).
Config makeTrimConfig() {
  Config config = makeBalanceConfig();
  config.speedKpRadPerMps = 0.5f;
  config.speedKiRadPerMpsS = 2.0f;
  config.speedIntegralEnabled = true;
  return config;
}

// A live zero-motion request received at requestTimeUs, with the commissioning switch and knob as given.
Request trimRequest(uint32_t requestTimeUs, bool commissioningSwitch, bool knobValid = false, float knobDeg = 0.0f) {
  Request r = liveRequest(requestTimeUs, 0.0f, 0.0f);
  r.commissioningSwitch = commissioningSwitch;
  r.knobValid = knobValid;
  r.knobTrimDeg = knobDeg;
  return r;
}

// A live zero-motion request received at requestTimeUs, with the save switch as given and the commissioning switch on.
Request saveRequest(uint32_t requestTimeUs, bool saveSwitch) {
  Request r = trimRequest(requestTimeUs, true);
  r.saveSwitch = saveSwitch;
  return r;
}

// A live request received at nowUs with the given speed and commissioning switch.
Request motionRequest(uint32_t nowUs, float speedMps, bool commissioningSwitch) {
  Request r = liveRequest(nowUs, speedMps, 0.0f);
  r.commissioningSwitch = commissioningSwitch;
  return r;
}

// A new IMU sample at nowUs with the knob given; the wheels are at rest and the request is fresh.
Output knobSample(BalanceController &controller, const Config &config, uint32_t nowUs, uint32_t imuSequence,
                  bool knobValid, float knobDeg) {
  return imuSample(controller, config, nowUs, imuSequence, ImuValues{}, trimRequest(nowUs, true, knobValid, knobDeg));
}

// Run at baseUs, then Commissioning at baseUs + 22000 us. The 22000 us sample is the first loop run with the switch
// on. With speed P and speed I at zero there, the transfer keeps the trim and the reference unchanged.
void enterCommissioning(BalanceController &controller, const Config &config, uint32_t baseUs) {
  enterRun(controller, config, baseUs);
  imuSample(controller, config, baseUs + 12000u, 2, ImuValues{}, trimRequest(baseUs + 12000u, false));
  assert(imuSample(controller, config, baseUs + 22000u, 3, ImuValues{}, trimRequest(baseUs + 22000u, true)).state ==
         State::Commissioning);
}

// Run with a standstill speed error of -0.008 m/s, so speed I integrates to about 0.0074 rad. One transfer request
// follows at 922000 us, on a loop run. Returns that tick.
Output integralThenTransfer(const Config &config) {
  BalanceController controller;
  enterRun(controller, config, 0);
  const ImuValues drift{0.0f, 0.0f, 0.0f, -0.08f, -0.08f};
  for (uint32_t k = 0; k < 90; ++k) {
    const uint32_t t = 12000u + 10000u * k;
    imuSample(controller, config, t, 2 + k, drift, trimRequest(t, false));
  }
  return imuSample(controller, config, 922000u, 92, drift, trimRequest(922000u, true));
}

enum class FlagSource : uint8_t { None, Voltage, Current, Effort, Reserve };

// Speed I after the Run ticks up to endUs, at 1 ms ticks from 3000 us. An IMU sample comes every 10 ms (12000, 22000,
// ...), and the loop runs on each 20 ms step (22000, 42000, ...). The wheels turn at -1 rad/s, so the speed error is
// +0.1 m/s. The source flag is set on every tick except the one before each loop run (21000, 41000, ...), or with
// dipOnly on the single tick 15000. Effort and reserve clear only with a wide envelope, so their other ticks run the
// wheels at -50 rad/s, which the loop never samples (it reads the wheels on its own tick).
float speedIAt(const Config &config, FlagSource source, bool dipOnly, uint32_t endUs) {
  BalanceController bc;
  enterRun(bc, config, 0);
  const bool effortOrReserve = source == FlagSource::Effort || source == FlagSource::Reserve;
  uint32_t imuSequence = 1;
  Output last;
  for (uint32_t t = 3000; t <= endUs; t += 1000) {
    if (t % 10000 == 2000) {
      ++imuSequence;
    }
    const bool flagged = dipOnly ? t == 15000 : t % 20000 != 1000;
    Measurements m = atTime(makeMeasurements(), t);
    m.imuSequence = imuSequence;
    m.tiltRateRadPerSec = source == FlagSource::Effort ? 20.0f : source == FlagSource::Reserve ? 15.8f : 0.0f;
    const float wheel = !flagged && effortOrReserve ? -50.0f : -1.0f;
    m.leftWheelRadPerSec = wheel;
    m.rightWheelRadPerSec = wheel;
    m.voltageClipped = flagged && source == FlagSource::Voltage;
    m.currentClipped = flagged && source == FlagSource::Current;
    last = bc.update(m, liveRequest(t, 0.0f, 0.0f), config, t);
  }
  return last.diag.speedI;
}
}  // namespace

int main() {
  const Config config = makeConfig();
  const Measurements good = makeMeasurements();

  // Disabled stays Disabled while the switch is off.
  {
    BalanceController bc;
    for (uint32_t t = 1000; t <= 4000; t += 1000) {
      const Output out = tick(bc, good, liveSwitch(t, false), config, t);
      assert(out.state == State::Disabled && !out.driveEnabled);
    }
  }

  // A switch that is already on at the first update does not arm.
  {
    BalanceController bc;
    assert(tick(bc, good, liveSwitch(1000, true), config, 1000).state == State::Disabled);
    assert(tick(bc, good, liveSwitch(2000, true), config, 2000).state == State::Disabled);
    const Output out = tick(bc, good, liveSwitch(3000, true), config, 3000);
    assert(out.state == State::Disabled && !out.driveEnabled);
  }

  // An off-to-on edge enters Arming, and drivers stay off there.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    const Output out = tick(bc, good, liveSwitch(2000, true), config, 2000);
    assert(out.state == State::Arming && !out.driveEnabled);
  }

  // Without a validated trim, the edge is lost and a later level does not arm.
  {
    Config noTrim = config;
    noTrim.trimValidated = false;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), noTrim, 1000);
    assert(tick(bc, good, liveSwitch(2000, true), noTrim, 2000).state == State::Disabled);
    assert(tick(bc, good, liveSwitch(3000, true), config, 3000).state == State::Disabled);  // no edge
    tick(bc, good, liveSwitch(4000, false), config, 4000);
    assert(tick(bc, good, liveSwitch(5000, true), config, 5000).state == State::Arming);
  }

  // Without a reachable fixed pose, the edge is lost and a later level does not arm.
  {
    Config noPose = config;
    noPose.fixedPoseReachable = false;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), noPose, 1000);
    assert(tick(bc, good, liveSwitch(2000, true), noPose, 2000).state == State::Disabled);
    assert(tick(bc, good, liveSwitch(3000, true), config, 3000).state == State::Disabled);  // no edge
    tick(bc, good, liveSwitch(4000, false), config, 4000);
    assert(tick(bc, good, liveSwitch(5000, true), config, 5000).state == State::Arming);
  }

  // Arming moves to Run once every condition holds, with drivers enabled and zero phase current.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    assert(tick(bc, good, liveSwitch(2000, true), config, 2000).state == State::Arming);
    const Output out = tick(bc, good, liveSwitch(3000, true), config, 3000);
    assert(out.state == State::Run && out.driveEnabled);
    assert(out.leftA == 0.0f && out.rightA == 0.0f);
    assert(!out.saveTrimRequested);
    assert(out.fault == FaultReason::None);
  }

  // Each arming condition alone keeps the controller in Arming.
  {
    struct Case {
      void (*apply)(Measurements &, Request &);
    };
    const Case cases[] = {
        [](Measurements &m, Request &) { m.imuValid = false; },
        [](Measurements &m, Request &) { m.wheelsValid = false; },
        [](Measurements &m, Request &) { m.tiltRad = kTrimTiltRad + 0.06f; },
        [](Measurements &m, Request &) { m.tiltRad = kTrimTiltRad - 0.06f; },
        [](Measurements &m, Request &) { m.tiltRateRadPerSec = 0.2f; },
        [](Measurements &, Request &r) { r.speedMps = 0.1f; },
        [](Measurements &, Request &r) { r.yawRateRadPerSec = -0.1f; },
    };
    for (const Case &c : cases) {
      Measurements m = good;
      Request r = makeRequest(true);
      c.apply(m, r);
      BalanceController bc;
      tick(bc, good, liveSwitch(1000, false), config, 1000);
      tick(bc, m, r, config, 2000);
      assert(tick(bc, m, r, config, 3000).state == State::Arming);
    }
  }

  // The tilt window is around the stored trim converted to radians: a level chassis (0 deg) is
  // outside the window of a 10 deg trim, and the trim angle itself is inside it.
  {
    Measurements level = good;
    level.tiltRad = 0.0f;
    BalanceController bc;
    tick(bc, level, liveSwitch(1000, false), config, 1000);
    assert(tick(bc, level, liveSwitch(2000, true), config, 2000).state == State::Arming);
    assert(tick(bc, level, liveSwitch(3000, true), config, 3000).state == State::Arming);
    assert(tick(bc, good, liveSwitch(4000, true), config, 4000).state == State::Run);
  }

  // Arming times out after the configured duration, measured with elapsed time across micros() wrap.
  {
    const uint32_t start = 0xFFFFFFF0u;
    Measurements unstable = good;
    unstable.tiltRateRadPerSec = 1.0f;
    BalanceController bc;
    tick(bc, unstable, liveSwitch(start - 1000, false), config, start - 1000);
    assert(tick(bc, unstable, liveSwitch(start, true), config, start).state == State::Arming);
    assert(tick(bc, unstable, liveSwitch(start + kTimeoutUs, true), config, start + kTimeoutUs).state == State::Arming);
    assert(tick(bc, unstable, liveSwitch(start + kTimeoutUs + 1, true), config, start + kTimeoutUs + 1).state == State::Disabled);
  }

  // After a timeout, the switch held on cannot re-arm; a fresh edge is needed.
  {
    Measurements unstable = good;
    unstable.tiltRateRadPerSec = 1.0f;
    BalanceController bc;
    tick(bc, unstable, liveSwitch(1000, false), config, 1000);
    tick(bc, unstable, liveSwitch(2000, true), config, 2000);
    assert(tick(bc, unstable, liveSwitch(2000 + kTimeoutUs + 1, true), config, 2000 + kTimeoutUs + 1).state == State::Disabled);
    assert(tick(bc, good, liveSwitch(2000 + kTimeoutUs + 2, true), config, 2000 + kTimeoutUs + 2).state == State::Disabled);
    tick(bc, good, liveSwitch(2000 + kTimeoutUs + 3, false), config, 2000 + kTimeoutUs + 3);
    assert(tick(bc, good, liveSwitch(2000 + kTimeoutUs + 4, true), config, 2000 + kTimeoutUs + 4).state == State::Arming);
  }

  // Switch off during Arming goes to Disabled.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    assert(tick(bc, good, liveSwitch(2000, true), config, 2000).state == State::Arming);
    const Output out = tick(bc, good, liveSwitch(2500, false), config, 2500);
    assert(out.state == State::Disabled && !out.driveEnabled);
  }

  // Switch off during Run goes to Disabled with drivers disabled.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    tick(bc, good, liveSwitch(2000, true), config, 2000);
    assert(tick(bc, good, liveSwitch(3000, true), config, 3000).state == State::Run);
    const Output out = tick(bc, good, liveSwitch(4000, false), config, 4000);
    assert(out.state == State::Disabled && !out.driveEnabled);
    assert(out.leftA == 0.0f && out.rightA == 0.0f);
  }

  // Run leaves only through the switch: a tilt outside the arming window does not stop it.
  {
    Measurements tilted = good;
    tilted.tiltRad = 0.5f;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    tick(bc, good, liveSwitch(2000, true), config, 2000);
    assert(tick(bc, good, liveSwitch(3000, true), config, 3000).state == State::Run);
    const Output out = tick(bc, tilted, liveSwitch(4000, true), config, 4000);
    assert(out.state == State::Run && out.driveEnabled);
  }

  // After Run is left by switch off, arming again needs a new edge.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), config, 1000);
    tick(bc, good, liveSwitch(2000, true), config, 2000);
    tick(bc, good, liveSwitch(3000, true), config, 3000);
    tick(bc, good, liveSwitch(4000, false), config, 4000);
    assert(tick(bc, good, liveSwitch(5000, true), config, 5000).state == State::Arming);
  }

  // Step 2: freshness. In Run, an age equal to the limit is fresh and one microsecond more faults.
  {
    const Measurements fresh = good;
    struct Case {
      uint32_t Measurements::*readTime;
      uint32_t Config::*maxAge;
      FaultReason reason;
    };
    const Case cases[] = {
        {&Measurements::imuReadTimeUs, &Config::imuMaxAgeUs, FaultReason::StaleImu},
        {&Measurements::wheelReadTimeUs, &Config::wheelMaxAgeUs, FaultReason::StaleWheels},
        {&Measurements::busReadTimeUs, &Config::busMaxAgeUs, FaultReason::StaleBus},
    };
    for (const Case &c : cases) {
      BalanceController bc;
      enterRun(bc, config, 100000);
      const uint32_t now = 112000;
      Measurements m = atTime(fresh, now);
      m.*(c.readTime) = now - config.*(c.maxAge);
      assert(bc.update(m, makeRequest(true), config, now).state == State::Run);
      m.*(c.readTime) = now - config.*(c.maxAge) - 1;
      const Output out = bc.update(m, makeRequest(true), config, now);
      assert(out.state == State::Fault && out.fault == c.reason && !out.driveEnabled);
    }
  }

  // A timestamp ahead of nowUs is not fresh.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements m = atTime(good, 112000);
    m.imuReadTimeUs = 112005;
    const Output out = bc.update(m, makeRequest(true), config, 112000);
    assert(out.state == State::Fault && out.fault == FaultReason::StaleImu);
  }

  // Invalid IMU or wheel data in Run latches InvalidInput.
  {
    Measurements noImu = good;
    noImu.imuValid = false;
    BalanceController bc;
    enterRun(bc, config, 100000);
    const Output out = tick(bc, noImu, liveSwitch(112000, true), config, 112000);
    assert(out.state == State::Fault && out.fault == FaultReason::InvalidInput);

    Measurements noWheels = good;
    noWheels.wheelsValid = false;
    BalanceController bc2;
    enterRun(bc2, config, 100000);
    const Output out2 = tick(bc2, noWheels, liveSwitch(112000, true), config, 112000);
    assert(out2.state == State::Fault && out2.fault == FaultReason::InvalidInput);
  }

  // Freshness across micros() wraparound: the sensor time is before the wrap, now is after it.
  {
    BalanceController bc;
    enterRun(bc, config, 0xFFFFFFF0u - 10000u);
    const uint32_t now = 0xFFFFFFF0u + 5000u;  // wraps to 4984
    Measurements m = atTime(good, now);
    m.imuReadTimeUs = now - config.imuMaxAgeUs;  // wraps backwards, age is exactly the limit
    assert(bc.update(m, makeRequest(true), config, now).state == State::Run);
    m.imuReadTimeUs = now - config.imuMaxAgeUs - 1;
    assert(bc.update(m, makeRequest(true), config, now).fault == FaultReason::StaleImu);
  }

  // Arming needs fresh IMU, wheels and bus. The request is live (received now), so each stale input alone is what
  // holds Arming; a request received at 0 would be lost and hold Arming by itself. Run follows once it is fresh.
  for (int which = 0; which < 3; ++which) {
    BalanceController bc;
    tick(bc, good, liveSwitch(200000, false), config, 200000);
    assert(tick(bc, good, liveSwitch(201000, true), config, 201000).state == State::Arming);

    Measurements late = atTime(good, 202000);
    if (which == 0) late.imuReadTimeUs = 202000 - config.imuMaxAgeUs - 1;
    if (which == 1) late.wheelReadTimeUs = 202000 - config.wheelMaxAgeUs - 1;
    if (which == 2) late.busReadTimeUs = 202000 - config.busMaxAgeUs - 1;
    assert(bc.update(late, liveSwitch(202000, true), config, 202000).state == State::Arming);

    assert(tick(bc, good, liveSwitch(203000, true), config, 203000).state == State::Run);
  }

  // Arming needs the wheels inside the arming band (5 rad/s here). Spinning wheels keep Arming; the same
  // measurements at rest arm.
  {
    Measurements spinning = good;
    spinning.leftWheelRadPerSec = 95.0f;
    spinning.rightWheelRadPerSec = 95.0f;
    BalanceController bc;
    tick(bc, spinning, liveSwitch(1000, false), config, 1000);
    tick(bc, spinning, liveSwitch(2000, true), config, 2000);
    assert(tick(bc, spinning, liveSwitch(3000, true), config, 3000).state == State::Arming);
    assert(tick(bc, good, liveSwitch(4000, true), config, 4000).state == State::Run);
  }

  // Arming needs an envelope that contains 0 A on both wheels. With KV 1 the back-EMF at 4 rad/s is about 22 V, far
  // above the 8.4 V limit, so the wheels inside the band still give envelopes excluding zero: about [-1.38, -0.62] A.
  // Arming holds, without a fault, until the timeout. At rest the same config arms.
  {
    Config lowKv = config;
    lowKv.kvRating = 1.0f;
    Measurements spinning = good;
    spinning.leftWheelRadPerSec = 4.0f;
    spinning.rightWheelRadPerSec = 4.0f;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), lowKv, 1000);
    tick(bc, spinning, liveSwitch(2000, true), lowKv, 2000);
    const Output held = tick(bc, spinning, liveSwitch(3000, true), lowKv, 3000);
    assert(held.state == State::Arming && !held.driveEnabled && held.fault == FaultReason::None);
    assert(tick(bc, spinning, liveSwitch(4000, true), lowKv, 4000).state == State::Arming);
    assert(tick(bc, spinning, liveSwitch(2000 + kTimeoutUs + 1, true), lowKv, 2000 + kTimeoutUs + 1).state ==
           State::Disabled);

    BalanceController atRest;
    tick(atRest, good, liveSwitch(1000, false), lowKv, 1000);
    tick(atRest, good, liveSwitch(2000, true), lowKv, 2000);
    assert(tick(atRest, good, liveSwitch(3000, true), lowKv, 3000).state == State::Run);
  }

  // An unchanged IMU sequence is not a new sample: it computes no dt and does not refresh the sample.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    assert(sample(bc, good, config, 103000, 2).state == State::Run);

    Measurements repeated = atTime(good, 103000);
    repeated.imuSequence = 2;
    // Same timestamp with an unchanged sequence: a dt of zero would fault if it were computed.
    assert(bc.update(repeated, makeRequest(true), config, 104000).state == State::Run);

    // Stalled IMU: the sequence stays at 2, so the sample goes stale by age alone.
    repeated.imuReadTimeUs = 103000;
    assert(bc.update(repeated, makeRequest(true), config, 103000 + config.imuMaxAgeUs + 1).fault ==
           FaultReason::StaleImu);
  }

  // dt window: a gap equal to the limit passes; one microsecond more faults.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    assert(sample(bc, good, config, 103000, 2).state == State::Run);
    assert(sample(bc, good, config, 103000 + config.dtMaxUs, 3).state == State::Run);
    const Output out = sample(bc, good, config, 103000 + 2 * config.dtMaxUs + 1, 4);
    assert(out.state == State::Fault && out.fault == FaultReason::BadDt && !out.driveEnabled);
  }

  // A dt of zero (a new sequence with the same timestamp) faults.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    assert(sample(bc, good, config, 103000, 2).state == State::Run);
    assert(sample(bc, good, config, 103000, 3).fault == FaultReason::BadDt);
  }

  // dt across micros() wraparound is the elapsed 1000 us and passes.
  {
    const uint32_t first = 0xFFFFFFF0u;
    BalanceController bc;
    enterRun(bc, config, first - 10000u);
    assert(sample(bc, good, config, first, 2).state == State::Run);
    assert(sample(bc, good, config, first + 1000u, 3).state == State::Run);
  }

  // The first processed sample after re-entering Run only records its timestamp. The gap from the
  // previous session is far beyond dtMaxUs and must not fault.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    assert(sample(bc, good, config, 103000, 2).state == State::Run);
    tick(bc, good, liveSwitch(104000, false), config, 104000);
    enterRun(bc, config, 200000);
    assert(sample(bc, good, config, 302000, 3).state == State::Run);
    assert(sample(bc, good, config, 303000, 4).state == State::Run);
  }

  // Fall: tilt beyond the fall angle for longer than the fall duration. Samples come every 10 ms, the
  // first sample past the angle starts the clock, and the fault is latched at 510 ms (500 ms elapsed).
  for (uint32_t base : {110000u, 0xFFFFFFF0u}) {
    for (float tilt : {0.7f, -0.7f}) {
      BalanceController bc;
      enterRun(bc, config, base - 8000u);
      Measurements tilted = good;
      tilted.tiltRad = tilt;
      uint32_t seq = 2;
      for (uint32_t j = 0; j <= 50; ++j) {
        assert(sample(bc, tilted, config, base + 10000u * j, seq++).state == State::Run);
      }
      const Output out = sample(bc, tilted, config, base + 10000u * 51, seq++);
      assert(out.state == State::Fault && out.fault == FaultReason::Fall && !out.driveEnabled);
    }
  }

  // Fall: a sample back inside the angle restarts the clock.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements beyond = good;
    beyond.tiltRad = 0.7f;
    Measurements level = good;
    level.tiltRad = 0.0f;
    const uint32_t base = 110000;
    uint32_t seq = 2;
    for (uint32_t j = 0; j <= 40; ++j) assert(sample(bc, beyond, config, base + 10000u * j, seq++).state == State::Run);
    assert(sample(bc, level, config, base + 10000u * 41, seq++).state == State::Run);
    // Without the restart the fault would come at j = 51; with it, only at j = 93.
    for (uint32_t j = 42; j <= 92; ++j) assert(sample(bc, beyond, config, base + 10000u * j, seq++).state == State::Run);
    const Output out = sample(bc, beyond, config, base + 10000u * 93, seq++);
    assert(out.state == State::Fault && out.fault == FaultReason::Fall);
  }

  // Overspeed: a wheel beyond the limit for longer than the duration faults.
  for (uint32_t base : {100000u, 0xFFFFFFF0u}) {
    BalanceController bc;
    enterRun(bc, config, base - 8000u);
    Measurements fast = good;
    fast.leftWheelRadPerSec = 150.0f;
    // Overspeed starts at base; 200 ms elapsed is still allowed, 210 ms latches.
    for (uint32_t j = 0; j <= 20; ++j) {
      assert(tick(bc, fast, liveSwitch(base + 10000u * j, true), config, base + 10000u * j).state == State::Run);
    }
    const Output out = tick(bc, fast, liveSwitch(base + 210000u, true), config, base + 210000u);
    assert(out.state == State::Fault && out.fault == FaultReason::Overspeed && !out.driveEnabled);
  }

  // Overspeed on the right wheel in reverse counts too, and a dip below the limit restarts the clock.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements fast = good;
    fast.rightWheelRadPerSec = -150.0f;
    const Measurements still = good;
    const uint32_t base = 110000;
    for (uint32_t j = 0; j <= 15; ++j) assert(tick(bc, fast, liveSwitch(base + 10000u * j, true), config, base + 10000u * j).state == State::Run);
    assert(tick(bc, still, liveSwitch(base + 10000u * 16, true), config, base + 10000u * 16).state == State::Run);
    for (uint32_t j = 17; j <= 37; ++j) assert(tick(bc, fast, liveSwitch(base + 10000u * j, true), config, base + 10000u * j).state == State::Run);
    const Output out = tick(bc, fast, liveSwitch(base + 10000u * 38, true), config, base + 10000u * 38);
    assert(out.state == State::Fault && out.fault == FaultReason::Overspeed);
  }

  // A NaN wheel speed makes its envelope non-finite, so it latches InvalidEnvelope on the first tick
  // rather than waiting for the overspeed duration (step 3 rule).
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements bad = good;
    bad.leftWheelRadPerSec = std::numeric_limits<float>::quiet_NaN();
    const Output out = tick(bc, bad, liveSwitch(110000u, true), config, 110000u);
    assert(out.state == State::Fault && out.fault == FaultReason::InvalidEnvelope && !out.driveEnabled);
  }

  // A tilt exactly at the fall angle is not a fall, however long it lasts.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements atAngle = good;
    atAngle.tiltRad = 0.6f;
    uint32_t seq = 2;
    for (uint32_t j = 0; j <= 100; ++j) {
      assert(sample(bc, atAngle, config, 110000u + 10000u * j, seq++).state == State::Run);
    }
  }

  // A wheel exactly at the limit is not overspeed, however long it lasts.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements atLimit = good;
    atLimit.leftWheelRadPerSec = 100.0f;
    atLimit.rightWheelRadPerSec = -100.0f;
    for (uint32_t j = 0; j <= 100; ++j) {
      assert(tick(bc, atLimit, liveSwitch(110000u + 10000u * j, true), config, 110000u + 10000u * j).state == State::Run);
    }
  }

  // A latched fault does not clear into Run while the switch stays on; the switch off returns to Disabled.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements noImu = good;
    noImu.imuValid = false;
    const Output faulted = tick(bc, noImu, liveSwitch(103000, true), config, 103000);
    assert(faulted.state == State::Fault && faulted.fault == FaultReason::InvalidInput);
    assert(!faulted.driveEnabled && faulted.leftA == 0.0f && faulted.rightA == 0.0f);
    assert(tick(bc, good, liveSwitch(104000, true), config, 104000).state == State::Fault);
    assert(tick(bc, good, liveSwitch(300000, true), config, 300000).state == State::Fault);

    const Output off = tick(bc, good, liveSwitch(301000, false), config, 301000);
    assert(off.state == State::Disabled && off.fault == FaultReason::None && !off.driveEnabled);

    // A new edge rearms through Arming, never straight into Run.
    assert(tick(bc, good, liveSwitch(302000, true), config, 302000).state == State::Arming);
    assert(tick(bc, good, liveSwitch(303000, true), config, 303000).state == State::Run);
  }

  // Step 3: per-wheel envelope. At rest with bus 8.4 V: Vmax = min(8.4, (8.4 - 0.4) * 1.0) = 8.0 V,
  // Imax = min(5, 8.0 / 22) = 0.363636 A and Imin = max(-5, -8.0 / 22) = -0.363636 A.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    const Output out = bc.update(m, makeRequest(true), config, 110000);
    assert(out.state == State::Run && out.fault == FaultReason::None);
    assert(closeTo(out.diag.leftMinA, -0.363636f) && closeTo(out.diag.leftMaxA, 0.363636f));
    assert(closeTo(out.diag.rightMinA, -0.363636f) && closeTo(out.diag.rightMaxA, 0.363636f));
  }

  // Vmax limited by voltage_limit: bus 24 V gives 23.6 V usable, so Vmax = 8.4 V and Imax = 8.4 / 22 = 0.381818 A.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    const Output out = tick(bc, good, liveSwitch(110000, true), config, 110000);
    assert(closeTo(out.diag.leftMaxA, 0.381818f) && closeTo(out.diag.leftMinA, -0.381818f));
  }

  // Vmax limited by the modulation factor: Vmax = min(8.4, 8.0 * 0.9) = 7.2 V, so Imax = 7.2 / 22 = 0.327273 A.
  {
    Config modulated = config;
    modulated.modulationFactor = 0.9f;
    BalanceController bc;
    enterRun(bc, modulated, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    const Output out = bc.update(m, makeRequest(true), modulated, 110000);
    assert(closeTo(out.diag.leftMaxA, 0.327273f) && closeTo(out.diag.leftMinA, -0.327273f));
  }

  // The current cap binds: with currentLimitA 0.2 A, both ends of the interval are capped at 0.2 A.
  {
    Config capped = config;
    capped.currentLimitA = 0.2f;
    BalanceController bc;
    enterRun(bc, capped, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    const Output out = bc.update(m, makeRequest(true), capped, 110000);
    assert(closeTo(out.diag.leftMaxA, 0.2f) && closeTo(out.diag.leftMinA, -0.2f));
  }

  // Left wheel forward at 100 rad/s, bus 8.4 V (Vmax 8.0 V): bemf = 100 / (100 * sqrt(3)) / 0.10472 = 5.513 V.
  // Imax = (8.0 - 5.513) / 22 = 0.113032 A, Imin = (-8.0 - 5.513) / 22 = -0.614240 A. Back-EMF shifts the
  // interval towards negative current. The right wheel at rest keeps the symmetric interval.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    m.leftWheelRadPerSec = 100.0f;
    const Output out = bc.update(m, makeRequest(true), config, 110000);
    assert(out.state == State::Run);
    assert(closeTo(out.diag.leftMinA, -0.614240f) && closeTo(out.diag.leftMaxA, 0.113032f));
    assert(closeTo(out.diag.rightMinA, -0.363636f) && closeTo(out.diag.rightMaxA, 0.363636f));
  }

  // Left and right with opposite speeds: the right interval is the mirror of the left one.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    m.leftWheelRadPerSec = 100.0f;
    m.rightWheelRadPerSec = -100.0f;
    const Output out = bc.update(m, makeRequest(true), config, 110000);
    assert(closeTo(out.diag.leftMinA, -0.614240f) && closeTo(out.diag.leftMaxA, 0.113032f));
    assert(closeTo(out.diag.rightMinA, -0.113032f) && closeTo(out.diag.rightMaxA, 0.614240f));
  }

  // Sign -1 left wheel. At forward -100 rad/s the motor sees +100 rad/s: motor interval [-0.614240, 0.113032]
  // A, mirrored to forward [-0.113032, 0.614240] A, the mirror of the sign +1 interval at +100 rad/s.
  // At forward +100 rad/s the motor sees -100 rad/s, so the forward interval equals the sign +1 one above.
  {
    Config mirrored = config;
    mirrored.leftWheelSign = -1;
    BalanceController bc;
    enterRun(bc, mirrored, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    m.leftWheelRadPerSec = -100.0f;
    const Output out = bc.update(m, makeRequest(true), mirrored, 110000);
    assert(closeTo(out.diag.leftMinA, -0.113032f) && closeTo(out.diag.leftMaxA, 0.614240f));

    m.leftWheelRadPerSec = 100.0f;
    const Output same = bc.update(m, makeRequest(true), mirrored, 110000);
    assert(closeTo(same.diag.leftMinA, -0.614240f) && closeTo(same.diag.leftMaxA, 0.113032f));
  }

  // Braking beyond back-EMF: left forward at 150 rad/s gives bemf 8.26993 V > Vmax 8.0 V, so the forward
  // interval lies entirely below zero: Imax = (8.0 - 8.26993) / 22 = -0.012270 A, Imin = (-8.0 - 8.26993) / 22
  // = -0.739542 A. It is a valid envelope, so the controller stays in Run.
  {
    Config fastWheel = config;
    fastWheel.wheelOverspeedRadPerSec = 200.0f;
    BalanceController bc;
    enterRun(bc, fastWheel, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    m.leftWheelRadPerSec = 150.0f;
    const Output out = bc.update(m, makeRequest(true), fastWheel, 110000);
    assert(out.state == State::Run && out.fault == FaultReason::None);
    assert(closeTo(out.diag.leftMinA, -0.739542f) && closeTo(out.diag.leftMaxA, -0.012270f));
  }

  // Demand shaping, negative side: braking from reverse, reversal to forward, exact landing when accelerating.
  {
    Config c = config;
    assert(closeTo(shape(Demand{-0.5f, 0.0f}, 0.0f, 0.0f, 1.0f, 0.01f, c).speedMps, -0.46f));
    // -0.01 brakes to zero in 0.0025 s, then accelerates 0.0075 s at 2 m/s^2 -> +0.015.
    assert(closeTo(shape(Demand{-0.01f, 0.0f}, 1.0f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.015f));
    // 0.49 is 0.01 below the target 0.5; a 0.02 step lands exactly on it.
    assert(closeTo(shape(Demand{0.49f, 0.0f}, 0.5f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.5f));
  }

  // Unusable demand caps fault instead of holding speed or moving it the wrong way.
  {
    for (int which = 0; which < 3; ++which) {
      Config broken = config;
      if (which == 0) broken.brakeCapMps2 = 0.0f;
      if (which == 1) broken.accelCapMps2 = -2.0f;
      if (which == 2) broken.speedCapMps = std::numeric_limits<float>::quiet_NaN();
      BalanceController bc;
      tick(bc, good, liveSwitch(1000, false), broken, 1000);
      tick(bc, good, liveSwitch(2000, true), broken, 2000);
      const Output armed = tick(bc, good, liveSwitch(3000, true), broken, 3000);
      assert(armed.state == State::Fault && armed.fault == FaultReason::InvalidConfig && !armed.driveEnabled);
    }
  }

  // A sum exactly at the tilt limit is not clipped and still integrates; the next step past it is refused.
  {
    Config edge = config;
    edge.speedIntegralEnabled = true;
    edge.speedKpRadPerMps = 0.0f;
    edge.speedKiRadPerMpsS = 1.0f;
    edge.tiltRefLimitRad = 0.5f;
    // candidate = 0.25 + 1.0 * 0.5 * 0.5 = 0.5, sum = 0.5 == limit.
    const auto atLimit = BalanceController::speedLoopStep(0.25f, 0.5f, 0.0f, false, 0.5f, edge);
    assert(atLimit.speedIRad == 0.5f && !atLimit.tiltClipped);
    // candidate would be 0.75 > limit and moves further out: refused.
    const auto past = BalanceController::speedLoopStep(0.5f, 0.5f, 0.0f, false, 0.5f, edge);
    assert(past.speedIRad == 0.5f);
  }

  // A stored trim beyond the tilt limit is rejected instead of being silently clamped.
  {
    Config bigTrim = config;
    bigTrim.storedTrimDeg = 20.0f;  // 0.349 rad > tiltRefLimitRad 0.3
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), bigTrim, 1000);
    tick(bc, good, liveSwitch(2000, true), bigTrim, 2000);
    Measurements tilted = good;
    tilted.tiltRad = 20.0f * 3.14159265f / 180.0f;
    const Output armed = tick(bc, tilted, liveSwitch(3000, true), bigTrim, 3000);
    assert(armed.state == State::Fault && armed.fault == FaultReason::InvalidConfig);
  }

  // An invalid envelope model faults on the arming tick itself, so drivers are never enabled on it.
  {
    Config broken = config;
    broken.kvRating = 0.0f;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), broken, 1000);
    tick(bc, good, liveSwitch(2000, true), broken, 2000);
    const Output armed = tick(bc, good, liveSwitch(3000, true), broken, 3000);
    assert(armed.state == State::Fault && armed.fault == FaultReason::InvalidEnvelope && !armed.driveEnabled);
  }

  // Each invalid envelope input latches InvalidEnvelope with drivers off.
  {
    struct Case {
      void (*apply)(Config &, Measurements &);
    };
    const Case cases[] = {
        [](Config &, Measurements &m) { m.busVoltageV = 5.0f; },                 // below busMinV 6 V
        [](Config &, Measurements &m) { m.busVoltageV = 31.0f; },                // above busMaxV 30 V
        [](Config &, Measurements &m) { m.busVoltageV = std::numeric_limits<float>::quiet_NaN(); },
        [](Config &c, Measurements &m) {  // usable 0.4 - 0.4 = 0 V, so Vmax = 0 V
          c.busMinV = 0.0f;
          m.busVoltageV = 0.4f;
        },
        [](Config &c, Measurements &) { c.kvRating = 0.0f; },
        [](Config &c, Measurements &) { c.phaseResistanceOhm = 0.0f; },
        [](Config &c, Measurements &) { c.leftWheelSign = 0; },
        [](Config &, Measurements &m) { m.rightWheelRadPerSec = std::numeric_limits<float>::quiet_NaN(); },
    };
    for (const Case &c : cases) {
      Config bad = config;
      Measurements m = good;
      c.apply(bad, m);
      BalanceController bc;
      enterRun(bc, config, 100000);
      const Output out = tick(bc, m, liveSwitch(110000, true), bad, 110000);
      assert(out.state == State::Fault && out.fault == FaultReason::InvalidEnvelope && !out.driveEnabled);
      assert(out.leftA == 0.0f && out.rightA == 0.0f);
    }
  }

  // Step 4: shapeDemand with caps 1 m/s, accel 2 m/s^2, brake 4 m/s^2, yaw 0.5 rad/s and dt 0.01 s.
  {
    const Config &c = config;
    const Demand zero{};

    // Ramp up at the accel cap: 2 m/s^2 over 0.01 s is 0.02 m/s per step.
    Demand d = zero;
    for (int k = 1; k <= 3; ++k) {
      d = shape(d, 1.0f, 0.0f, 1.0f, 0.01f, c);
      assert(closeTo(d.speedMps, 0.02f * k));
    }

    // The speed cap clamps the target; the ramp is limited on the way there.
    assert(closeTo(shape(Demand{0.5f, 0.0f}, 5.0f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.52f));
    Demand capped{0.5f, 0.0f};
    for (int k = 0; k < 100; ++k) capped = shape(capped, 5.0f, 0.0f, 1.0f, 0.01f, c);
    assert(closeTo(capped.speedMps, 1.0f));
    Demand reversedCap{-0.5f, 0.0f};
    for (int k = 0; k < 100; ++k) reversedCap = shape(reversedCap, -5.0f, 0.0f, 1.0f, 0.01f, c);
    assert(closeTo(reversedCap.speedMps, -1.0f));

    // Braking shrinks the magnitude at 4 m/s^2, twice the accel rate: 0.04 m/s per step.
    const Demand start{0.5f, 0.0f};
    assert(closeTo(shape(start, 0.0f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.46f));
    assert(closeTo(shape(start, 0.49f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.49f));  // target reached in one step

    // Reversal from +0.01 m/s to -1 m/s: braking to zero takes 0.0025 s at 4 m/s^2, then the remaining
    // 0.0075 s accelerates at 2 m/s^2 to -0.015 m/s within the same step.
    assert(closeTo(shape(Demand{0.01f, 0.0f}, -1.0f, 0.0f, 1.0f, 0.01f, c).speedMps, -0.015f));

    // Demand scaling with demandScaleCapStart 0.5. Accel and yaw factor: clamp((s - 0.5) / 0.5). Cap factor:
    // clamp(s / 0.5). The brake ramp is never scaled.
    // s = 0.75: accel and yaw factor 0.5, cap factor 1. Accel 1 m/s^2 gives 0.01 m/s in one 0.01 s step, and the
    // cruise cap stays at 1 m/s.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 0.75f, 0.01f, c).speedMps, 0.01f));
    Demand reach = zero;
    for (int k = 0; k < 200; ++k) reach = shape(reach, 5.0f, 0.0f, 0.75f, 0.01f, c);
    assert(closeTo(reach.speedMps, 1.0f));
    assert(closeTo(shape(zero, 0.0f, 2.0f, 0.75f, 0.01f, c).yawRateRadPerSec, 0.25f));

    // s = 0.5: accel and yaw factor is 0 at demandScaleCapStart, so there is no gain from rest.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 0.5f, 0.01f, c).speedMps, 0.0f));

    // s = 0.25: accel and yaw factor 0, cap factor 0.5. From rest nothing moves. From 0.9 m/s the speed brakes to
    // the 0.5 m/s cruise cap at 4 m/s^2 (0.04 m/s per step) and stays there.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 0.25f, 0.01f, c).speedMps, 0.0f));
    Demand cruise{0.9f, 0.0f};
    for (int k = 0; k < 100; ++k) cruise = shape(cruise, 0.9f, 0.0f, 0.25f, 0.01f, c);
    assert(closeTo(cruise.speedMps, 0.5f));
    assert(closeTo(shape(zero, 0.0f, 2.0f, 0.25f, 0.01f, c).yawRateRadPerSec, 0.0f));

    // The brake ramp is not scaled: from 0.5 m/s with a zero request it falls 0.04 m/s per step at s 0.25 and 0.
    assert(closeTo(shape(start, 0.0f, 0.0f, 0.25f, 0.01f, c).speedMps, 0.46f));
    assert(closeTo(shape(start, 0.0f, 0.0f, 0.0f, 0.01f, c).speedMps, 0.46f));

    // s = 0: accel, yaw and cruise cap are all zero.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 0.0f, 0.01f, c).speedMps, 0.0f));
    assert(closeTo(shape(zero, 0.0f, 2.0f, 0.0f, 0.01f, c).yawRateRadPerSec, 0.0f));

    // s = 1: all factors 1, so the full 0.02 m/s step and the full 0.5 rad/s yaw cap apply.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 1.0f, 0.01f, c).speedMps, 0.02f));

    // demandScale above 1 counts as 1, below 0 as 0.
    assert(closeTo(shape(zero, 5.0f, 0.0f, 3.0f, 0.01f, c).speedMps, 0.02f));
    assert(closeTo(shape(zero, 5.0f, 0.0f, -1.0f, 0.01f, c).speedMps, 0.0f));
    assert(closeTo(shape(zero, 0.0f, 2.0f, -1.0f, 0.01f, c).yawRateRadPerSec, 0.0f));

    // Yaw is clamped symmetrically and has no slew: a request inside the cap passes at once.
    assert(closeTo(shape(zero, 0.0f, 2.0f, 1.0f, 0.01f, c).yawRateRadPerSec, 0.5f));
    assert(closeTo(shape(zero, 0.0f, -2.0f, 1.0f, 0.01f, c).yawRateRadPerSec, -0.5f));
    assert(closeTo(shape(zero, 0.0f, 0.3f, 1.0f, 0.01f, c).yawRateRadPerSec, 0.3f));

    // NaN requests count as zero, and a NaN demand scale counts as zero.
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Demand fromNan = shape(Demand{0.5f, 0.3f}, nan, nan, 1.0f, 0.01f, c);
    assert(closeTo(fromNan.speedMps, 0.46f) && closeTo(fromNan.yawRateRadPerSec, 0.0f));
    assert(closeTo(shape(zero, 5.0f, 0.0f, nan, 0.01f, c).speedMps, 0.0f));
  }

  // Step 4: update() shapes a valid request over IMU samples. The last sample of the ramp shows 0.5 m/s
  // and yaw clamped to 0.5 rad/s.
  {
    BalanceController bc;
    const Output last = rampToHalfMps(bc, config, 100000u);
    assert(last.state == State::Run);
    assert(closeTo(last.diag.shapedSpeedMps, 0.5f) && closeTo(last.diag.shapedYawRateRadPerSec, 0.5f));
  }

  // Step 4: motion request age across micros() wraparound. Age equal to requestMaxAgeUs is fresh; one
  // microsecond more counts as a zero request, which ramps the speed down at 4 m/s^2 (0.2 m/s per 50 ms).
  {
    const uint32_t base = 0xFFFFFFF0u - 200000u;
    const uint32_t now = base + 410000u;  // wraps past micros() overflow

    BalanceController fresh;
    rampToHalfMps(fresh, config, base);
    const Output ok = liveSample(fresh, config, now, 8, liveRequest(now - 50000u, 1.0f, 0.0f));
    assert(ok.state == State::Run && closeTo(ok.diag.shapedSpeedMps, 0.6f));

    BalanceController stale;
    rampToHalfMps(stale, config, base);
    const Output old = liveSample(stale, config, now, 8, liveRequest(now - 50001u, 1.0f, 2.0f));
    assert(old.state == State::Run && closeTo(old.diag.shapedSpeedMps, 0.3f));
    assert(closeTo(old.diag.shapedYawRateRadPerSec, 0.0f));

    // A request with sourceValid false ramps to zero at the brake cap: 0.3, 0.1, then 0 with no overshoot.
    BalanceController invalid;
    rampToHalfMps(invalid, config, base);
    Request invalidRequest = liveRequest(now, 1.0f, 2.0f);
    invalidRequest.sourceValid = false;
    assert(closeTo(liveSample(invalid, config, now, 8, invalidRequest).diag.shapedSpeedMps, 0.3f));
    assert(closeTo(liveSample(invalid, config, now + 50000u, 9, invalidRequest).diag.shapedSpeedMps, 0.1f));
    const Output stopped = liveSample(invalid, config, now + 100000u, 10, invalidRequest);
    assert(closeTo(stopped.diag.shapedSpeedMps, 0.0f) && closeTo(stopped.diag.shapedYawRateRadPerSec, 0.0f));
  }

  // Step 4: an unchanged IMU sequence is not a new sample, so the ramp does not advance.
  {
    BalanceController bc;
    rampToHalfMps(bc, config, 100000u);  // last processed sequence is 7
    const Output repeated = liveSample(bc, config, 480000u, 7, liveRequest(480000u, 1.0f, 0.0f));
    assert(repeated.state == State::Run && closeTo(repeated.diag.shapedSpeedMps, 0.5f));
    const Output next = liveSample(bc, config, 510000u, 8, liveRequest(510000u, 1.0f, 0.0f));
    assert(closeTo(next.diag.shapedSpeedMps, 0.6f));
  }

  // Step 4: the shaped state resets on re-entering Run. The first sample after re-entry only records its
  // timestamp and shows zero, then the ramp starts again from zero.
  {
    BalanceController bc;
    rampToHalfMps(bc, config, 100000u);
    enterRun(bc, config, 600000u);
    const Output first = liveSample(bc, config, 610000u, 8, liveRequest(610000u, 1.0f, 0.0f));
    assert(first.state == State::Run && closeTo(first.diag.shapedSpeedMps, 0.0f));
    const Output second = liveSample(bc, config, 660000u, 9, liveRequest(660000u, 1.0f, 0.0f));
    assert(closeTo(second.diag.shapedSpeedMps, 0.1f));
  }

  // Step 5: P-only correction. Kp 0.5 * 0.2 m/s = 0.1 rad; the integral is zero even when a previous value
  // is passed in, because it is disabled.
  {
    const BalanceController::SpeedLoopResult r =
        BalanceController::speedLoopStep(0.05f, 0.2f, 0.0f, false, 0.02f, config);
    assert(nearlyTo(r.speedPRad, 0.1f, 1e-6f));
    assert(nearlyTo(r.speedIRad, 0.0f, 1e-6f));
    assert(nearlyTo(r.desiredTiltRad, 0.1f, 1e-6f) && !r.tiltClipped);
  }

  // Step 5: the integral accumulates Ki * e * dt. With Ki 0.2, e 0.2 m/s and dt 0.02 s it adds 0.0008 per
  // step, so ten steps give 0.008.
  {
    Config integ = config;
    integ.speedIntegralEnabled = true;
    integ.speedKpRadPerMps = 0.0f;
    float speedI = 0.0f;
    for (int k = 0; k < 10; ++k) {
      speedI = BalanceController::speedLoopStep(speedI, 0.2f, 0.0f, false, 0.02f, integ).speedIRad;
      if (k == 0) {
        assert(nearlyTo(speedI, 0.0008f, 1e-7f));
      }
    }
    assert(nearlyTo(speedI, 0.008f, 1e-6f));
  }

  // Step 5: the tilt reference is clamped with the trim offset. Trim 0.1 + P 0.3 = 0.4 clamps to the 0.15
  // limit, so the speed correction is 0.05 and the clip flag is set.
  {
    Config limited = config;
    limited.tiltRefLimitRad = 0.15f;
    const BalanceController::SpeedLoopResult r =
        BalanceController::speedLoopStep(0.0f, 0.6f, 0.1f, false, 0.02f, limited);
    assert(nearlyTo(r.desiredTiltRad, 0.15f, 1e-6f) && r.tiltClipped);
    assert(nearlyTo(r.desiredTiltRad - 0.1f, 0.05f, 1e-6f));
  }

  // Step 5: downstream limits freeze the integral for this update; otherwise it integrates normally.
  {
    Config integ = config;
    integ.speedIntegralEnabled = true;
    integ.speedKpRadPerMps = 0.0f;
    const BalanceController::SpeedLoopResult frozen =
        BalanceController::speedLoopStep(0.02f, 0.1f, 0.0f, true, 0.02f, integ);
    assert(nearlyTo(frozen.speedIRad, 0.02f, 1e-6f));
    const BalanceController::SpeedLoopResult free =
        BalanceController::speedLoopStep(0.02f, 0.1f, 0.0f, false, 0.02f, integ);
    assert(nearlyTo(free.speedIRad, 0.0204f, 1e-6f));
  }

  // Step 5: tilt-clipped anti-windup, both signs. Trim 0.1 with integral 0.06 is already above the 0.15
  // limit: a further rise is refused, while a fall back is accepted. The mirror case behaves the same way.
  {
    Config integ = config;
    integ.speedIntegralEnabled = true;
    integ.speedKpRadPerMps = 0.0f;
    integ.tiltRefLimitRad = 0.15f;
    const BalanceController::SpeedLoopResult up =
        BalanceController::speedLoopStep(0.06f, 0.5f, 0.1f, false, 0.02f, integ);
    assert(nearlyTo(up.speedIRad, 0.06f, 1e-6f) && up.tiltClipped);
    const BalanceController::SpeedLoopResult upBack =
        BalanceController::speedLoopStep(0.06f, -0.5f, 0.1f, false, 0.02f, integ);
    assert(nearlyTo(upBack.speedIRad, 0.058f, 1e-6f));

    const BalanceController::SpeedLoopResult low =
        BalanceController::speedLoopStep(-0.06f, -0.5f, -0.1f, false, 0.02f, integ);
    assert(nearlyTo(low.speedIRad, -0.06f, 1e-6f) && low.tiltClipped);
    const BalanceController::SpeedLoopResult lowBack =
        BalanceController::speedLoopStep(-0.06f, 0.5f, -0.1f, false, 0.02f, integ);
    assert(nearlyTo(lowBack.speedIRad, -0.058f, 1e-6f));
  }

  // Step 5: recovery after saturation, both signs. An integral of 0.2 wound up against the 0.15 limit with
  // trim 0.1 unwinds by 0.002 per step. After 80 steps it is 0.04, the sum 0.14 is back inside the band,
  // and the next step integrates normally.
  {
    Config integ = config;
    integ.speedIntegralEnabled = true;
    integ.speedKpRadPerMps = 0.0f;
    integ.tiltRefLimitRad = 0.15f;

    float speedI = 0.2f;
    for (int k = 0; k < 80; ++k) {
      speedI = BalanceController::speedLoopStep(speedI, -0.5f, 0.1f, false, 0.02f, integ).speedIRad;
    }
    assert(nearlyTo(speedI, 0.04f, 1e-5f));
    const BalanceController::SpeedLoopResult inside =
        BalanceController::speedLoopStep(speedI, -0.5f, 0.1f, false, 0.02f, integ);
    assert(!inside.tiltClipped && nearlyTo(inside.speedIRad, 0.038f, 1e-5f));

    float speedNegI = -0.2f;
    for (int k = 0; k < 80; ++k) {
      speedNegI = BalanceController::speedLoopStep(speedNegI, 0.5f, -0.1f, false, 0.02f, integ).speedIRad;
    }
    assert(nearlyTo(speedNegI, -0.04f, 1e-5f));
    const BalanceController::SpeedLoopResult insideNeg =
        BalanceController::speedLoopStep(speedNegI, 0.5f, -0.1f, false, 0.02f, integ);
    assert(!insideNeg.tiltClipped && nearlyTo(insideNeg.speedIRad, -0.038f, 1e-5f));
  }

  // Step 5: the slew moves 1 rad/s * 0.25 s per step and lands exactly on the desired value.
  {
    float ref = 0.0f;
    ref = BalanceController::slewTiltRef(ref, 1.0f, 0.25f, config);
    assert(nearlyTo(ref, 0.25f, 1e-6f));
    ref = BalanceController::slewTiltRef(ref, 1.0f, 0.25f, config);
    ref = BalanceController::slewTiltRef(ref, 1.0f, 0.25f, config);
    assert(nearlyTo(ref, 0.75f, 1e-6f));
    ref = BalanceController::slewTiltRef(ref, 1.0f, 0.25f, config);
    assert(ref == 1.0f);
    assert(nearlyTo(BalanceController::slewTiltRef(0.0f, -0.5f, 0.25f, config), -0.25f, 1e-6f));
    assert(BalanceController::slewTiltRef(0.7f, 0.8f, 0.25f, config) == 0.8f);
  }

  // Step 5: a speed loop period, wheel radius, slew rate or tilt limit that is unusable is an invalid
  // configuration, and the controller faults before drivers can be enabled.
  {
    const auto armedWith = [&](const Config &bad) {
      BalanceController bc;
      tick(bc, good, liveSwitch(1000, false), config, 1000);
      tick(bc, good, liveSwitch(2000, true), config, 2000);
      return tick(bc, good, liveSwitch(3000, true), bad, 3000);
    };
    Config noPeriod = config;
    noPeriod.speedLoopPeriodUs = 0;
    Config noRadius = config;
    noRadius.wheelRadiusM = 0.0f;
    Config noSlew = config;
    noSlew.tiltRefSlewRadPerSec = 0.0f;
    Config noTiltClip = config;
    noTiltClip.tiltClipDurationUs = 0;
    for (const Config *bad : {&noPeriod, &noRadius, &noSlew, &noTiltClip}) {
      const Output out = armedWith(*bad);
      assert(out.state == State::Fault && out.fault == FaultReason::InvalidConfig && !out.driveEnabled);
    }
  }

  // Step 5, update(): the loop runs every 20 ms (50 Hz) on processed IMU samples and integrates with the
  // elapsed time. The measured speed is the mean of both wheels times the radius: (4 + 0) / 2 * 0.1 = 0.2 m/s.
  // With a zero request, the error is -0.2 m/s, so P = 0.5 * -0.2 = -0.1 rad and each run adds
  // 0.2 * -0.2 * 0.02 = -0.0008 to the integral.
  {
    Config integ = config;
    integ.speedIntegralEnabled = true;
    BalanceController bc;
    tick(bc, good, liveSwitch(0, false), integ, 0);
    tick(bc, good, liveSwitch(1000, true), integ, 1000);
    const Output entry = tick(bc, good, liveSwitch(2000, true), integ, 2000);
    assert(entry.state == State::Run);
    // Tilt reference starts at the stored trim of 10 degrees.
    assert(nearlyTo(entry.diag.tiltRefRad, kTrimTiltRad, 1e-6f) && nearlyTo(entry.diag.trimDeg, 10.0f, 1e-3f));

    // First processed sample after entry only records its timestamp, so the loop does not run yet.
    const Output first = wheelSample(bc, integ, 12000, 2, 4.0f, 0.0f);
    assert(nearlyTo(first.diag.speedP, 0.0f, 1e-6f) && nearlyTo(first.diag.speedI, 0.0f, 1e-6f));

    // 20 ms after entry the loop runs: the reference slews by 0.01 rad (1 rad/s * 10 ms) toward its target.
    const Output run = wheelSample(bc, integ, 22000, 3, 4.0f, 0.0f);
    assert(nearlyTo(run.diag.speedP, -0.1f, 1e-6f));
    assert(nearlyTo(run.diag.speedI, -0.0008f, 1e-7f));
    assert(nearlyTo(run.diag.tiltRefRad, kTrimTiltRad - 0.01f, 1e-6f));

    // 10 ms later the loop is not due: the integral is unchanged and the reference slews one more step.
    const Output between = wheelSample(bc, integ, 32000, 4, 4.0f, 0.0f);
    assert(nearlyTo(between.diag.speedI, -0.0008f, 1e-7f));
    assert(nearlyTo(between.diag.tiltRefRad, kTrimTiltRad - 0.02f, 1e-6f));

    // Next due sample: a second run adds the same -0.0008 over its 20 ms.
    const Output next = wheelSample(bc, integ, 42000, 5, 4.0f, 0.0f);
    assert(nearlyTo(next.diag.speedI, -0.0016f, 1e-7f));
    assert(nearlyTo(next.diag.speedP, -0.1f, 1e-6f));

    // Switching the enable off and on again re-enters Run. The speed loop state starts from zero and the
    // reference restarts at the trim.
    tick(bc, good, liveSwitch(50000, false), integ, 50000);
    tick(bc, good, liveSwitch(51000, true), integ, 51000);
    const Output reentry = tick(bc, good, liveSwitch(52000, true), integ, 52000);
    assert(reentry.state == State::Run);
    assert(nearlyTo(reentry.diag.speedI, 0.0f, 1e-6f) && nearlyTo(reentry.diag.speedP, 0.0f, 1e-6f));
    assert(nearlyTo(reentry.diag.tiltRefRad, kTrimTiltRad, 1e-6f));
  }

  // Step 6, allocateEffort: symmetric envelopes [-2, 2] on both wheels, balance 1.0 and yaw 0.5. cMin -2, cMax 2,
  // c 1. dMin = max(1-2, -2-1) = -1 and dMax = min(1+2, 2-1) = 1, so d = 0.5, left = 0.5, right = 1.5,
  // reserve = min(2-1, 1+2) = 1. Nothing is clipped.
  {
    const Allocation a = BalanceController::allocateEffort(1.0f, 0.5f, -2.0f, 2.0f, -2.0f, 2.0f);
    assert(nearlyTo(a.cMin, -2.0f, 1e-6f) && nearlyTo(a.cMax, 2.0f, 1e-6f));
    assert(nearlyTo(a.c, 1.0f, 1e-6f) && nearlyTo(a.dMin, -1.0f, 1e-6f) && nearlyTo(a.dMax, 1.0f, 1e-6f));
    assert(nearlyTo(a.d, 0.5f, 1e-6f));
    assert(nearlyTo(a.leftA, 0.5f, 1e-6f) && nearlyTo(a.rightA, 1.5f, 1e-6f));
    assert(nearlyTo(a.reserve, 1.0f, 1e-6f));
    assert(!a.effortClipped && !a.differentialClipped && !a.forcedDifferential);
  }

  // Step 6, allocateEffort: asymmetric envelopes L [-0.6, 0.1] and R [-0.2, 0.3]. cMin = (-0.6 - 0.2) / 2 = -0.4
  // and cMax = (0.1 + 0.3) / 2 = 0.2.
  {
    // Balance 0 and yaw 0 fit: dMin = max(0 - 0.1, -0.2 - 0) = -0.1, dMax = min(0 + 0.6, 0.3 - 0) = 0.3,
    // d = 0, reserve = min(0.2, 0.4) = 0.2.
    const Allocation fits = BalanceController::allocateEffort(0.0f, 0.0f, -0.6f, 0.1f, -0.2f, 0.3f);
    assert(nearlyTo(fits.cMin, -0.4f, 1e-6f) && nearlyTo(fits.cMax, 0.2f, 1e-6f));
    assert(nearlyTo(fits.dMin, -0.1f, 1e-6f) && nearlyTo(fits.dMax, 0.3f, 1e-6f));
    assert(nearlyTo(fits.leftA, 0.0f, 1e-6f) && nearlyTo(fits.rightA, 0.0f, 1e-6f));
    assert(nearlyTo(fits.reserve, 0.2f, 1e-6f));
    assert(!fits.effortClipped && !fits.differentialClipped && !fits.forcedDifferential);

    // Yaw 0.5 is clamped at dMax = 0.3: left = -0.3 and right = 0.3. The envelope, not balance, limits d here,
    // and the clip is flagged the same way.
    const Allocation yawHigh = BalanceController::allocateEffort(0.0f, 0.5f, -0.6f, 0.1f, -0.2f, 0.3f);
    assert(nearlyTo(yawHigh.d, 0.3f, 1e-6f) && nearlyTo(yawHigh.leftA, -0.3f, 1e-6f) &&
           nearlyTo(yawHigh.rightA, 0.3f, 1e-6f));
    assert(yawHigh.differentialClipped && yawHigh.forcedDifferential && !yawHigh.effortClipped);

    // Yaw -0.5 is clamped at dMin = -0.1: left = 0.1 (the left maximum) and right = -0.1.
    const Allocation yawLow = BalanceController::allocateEffort(0.0f, -0.5f, -0.6f, 0.1f, -0.2f, 0.3f);
    assert(nearlyTo(yawLow.d, -0.1f, 1e-6f) && nearlyTo(yawLow.leftA, 0.1f, 1e-6f) &&
           nearlyTo(yawLow.rightA, -0.1f, 1e-6f));
    assert(yawLow.differentialClipped && yawLow.forcedDifferential);

    // Balance -1.0 clips c to cMin = -0.4 (effortClipped). dMin = max(-0.4 - 0.1, -0.2 + 0.4) = 0.2 and
    // dMax = min(-0.4 + 0.6, 0.3 + 0.4) = 0.2, so d = 0.2 is forced although the yaw request is 0.
    // left = -0.6 (the left minimum), right = -0.2 (the right minimum), reserve = 0.
    const Allocation low = BalanceController::allocateEffort(-1.0f, 0.0f, -0.6f, 0.1f, -0.2f, 0.3f);
    assert(nearlyTo(low.c, -0.4f, 1e-6f) && low.effortClipped);
    assert(nearlyTo(low.dMin, 0.2f, 1e-6f) && nearlyTo(low.dMax, 0.2f, 1e-6f));
    assert(nearlyTo(low.d, 0.2f, 1e-6f) && low.forcedDifferential);
    assert(nearlyTo(low.leftA, -0.6f, 1e-6f) && nearlyTo(low.rightA, -0.2f, 1e-6f));
    assert(nearlyTo(low.reserve, 0.0f, 1e-6f));

    // Balance +1.0 clips c to cMax = 0.2. dMin = max(0.2 - 0.1, -0.2 - 0.2) = 0.1 and
    // dMax = min(0.2 + 0.6, 0.3 - 0.2) = 0.1, so d = 0.1: left = 0.1 and right = 0.3, both at their maxima.
    const Allocation high = BalanceController::allocateEffort(1.0f, 0.0f, -0.6f, 0.1f, -0.2f, 0.3f);
    assert(nearlyTo(high.c, 0.2f, 1e-6f) && high.effortClipped);
    assert(nearlyTo(high.d, 0.1f, 1e-6f) && high.forcedDifferential);
    assert(nearlyTo(high.leftA, 0.1f, 1e-6f) && nearlyTo(high.rightA, 0.3f, 1e-6f));
    assert(nearlyTo(high.reserve, 0.0f, 1e-6f));
  }

  // Step 6, allocateEffort: symmetric envelopes [-2, 2]. A balance request beyond either end clips c to +/-2 and
  // collapses the differential to zero, so both wheels carry the same current.
  {
    const Allocation top = BalanceController::allocateEffort(3.0f, 0.0f, -2.0f, 2.0f, -2.0f, 2.0f);
    assert(nearlyTo(top.c, 2.0f, 1e-6f) && top.effortClipped);
    assert(nearlyTo(top.d, 0.0f, 1e-6f) && !top.differentialClipped);
    assert(nearlyTo(top.leftA, 2.0f, 1e-6f) && nearlyTo(top.rightA, 2.0f, 1e-6f));
    assert(nearlyTo(top.reserve, 0.0f, 1e-6f));

    const Allocation bottom = BalanceController::allocateEffort(-3.0f, 0.0f, -2.0f, 2.0f, -2.0f, 2.0f);
    assert(nearlyTo(bottom.c, -2.0f, 1e-6f) && bottom.effortClipped);
    assert(nearlyTo(bottom.leftA, -2.0f, 1e-6f) && nearlyTo(bottom.rightA, -2.0f, 1e-6f));
    assert(nearlyTo(bottom.reserve, 0.0f, 1e-6f));
  }

  // Step 6, allocateEffort: a yaw request of 5 exceeds dMax = min(0 + 2, 2 - 0) = 2, so d clamps to the envelope
  // edge: left = -2 and right = 2. The differential is forced by the envelope, not by balance.
  {
    const Allocation a = BalanceController::allocateEffort(0.0f, 5.0f, -2.0f, 2.0f, -2.0f, 2.0f);
    assert(nearlyTo(a.d, 2.0f, 1e-6f) && nearlyTo(a.leftA, -2.0f, 1e-6f) && nearlyTo(a.rightA, 2.0f, 1e-6f));
    assert(a.differentialClipped && a.forcedDifferential && !a.effortClipped);
  }

  // Step 6, configValid: reserveLowA must be positive, and the balance and yaw gains must be finite.
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();
    Config noReserve = makeBalanceConfig();
    noReserve.reserveLowA = 0.0f;
    Config nanTilt = makeBalanceConfig();
    nanTilt.tiltPAPerRad = nan;
    Config infYaw = makeBalanceConfig();
    infYaw.yawPAPerRadPerSec = inf;
    for (const Config *bad : {&noReserve, &nanTilt, &infYaw}) {
      BalanceController bc;
      tick(bc, good, makeRequest(false), *bad, 1000);
      tick(bc, good, makeRequest(true), *bad, 2000);
      const Output out = tick(bc, good, makeRequest(true), *bad, 3000);
      assert(out.state == State::Fault && out.fault == FaultReason::InvalidConfig && !out.driveEnabled);
    }
  }

  // currentLimitA must be finite and positive. Zero or negative leaves an empty envelope, and NaN is refused, all on the
  // arming tick, so no driver is enabled.
  {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    for (float badLimit : {0.0f, -5.0f, nan}) {
      Config noLimit = config;
      noLimit.currentLimitA = badLimit;
      BalanceController bc;
      tick(bc, good, liveSwitch(1000, false), noLimit, 1000);
      assert(tick(bc, good, liveSwitch(2000, true), noLimit, 2000).state == State::Arming);
      const Output out = tick(bc, good, liveSwitch(3000, true), noLimit, 3000);
      assert(out.state == State::Fault && out.fault == FaultReason::InvalidConfig && !out.driveEnabled);
    }
  }

  // Step 6, update(): balance law and yaw law with exact currents. Envelopes are +/-8.4 A at standstill.
  // Tilt error 0.01 rad, tilt rate 0.2 rad/s: balance = 2 * 0.01 + 0.5 * 0.2 = 0.12 A. Measured yaw 0 and request 0,
  // so the yaw differential is 0 and both wheels carry 0.12 A. The first processed sample only records its timestamp,
  // but the balance law still runs on it.
  const Config balance = makeBalanceConfig();
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    const Output first = imuSample(bc, balance, 12000, 2, ImuValues{0.01f, 0.2f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    assert(first.state == State::Run && first.driveEnabled && first.fault == FaultReason::None);
    assert(nearlyTo(first.leftA, 0.12f, 1e-4f) && nearlyTo(first.rightA, 0.12f, 1e-4f));
    assert(nearlyTo(first.diag.c, 0.12f, 1e-4f) && nearlyTo(first.diag.d, 0.0f, 1e-6f));
    assert(nearlyTo(first.diag.reserve, 8.28f, 1e-4f) && nearlyTo(first.diag.demandScale, 1.0f, 1e-6f));
    assert(!first.diag.effortClipped && !first.diag.forcedDifferential);
    assert(!first.diag.voltageClipped && !first.diag.currentClipped);

    // Yaw law: the request is 0.3 rad/s (below the 0.5 cap, scale 1), the measured yaw is 0.1 rad/s, and the
    // gain is 1.0, so the yaw differential is 0.2 A. Balance is 0 here, so left = -0.2 and right = 0.2.
    const Output yaw = imuSample(bc, balance, 22000, 3, ImuValues{0.0f, 0.0f, 0.1f, 0.0f, 0.0f},
                                 liveRequest(22000, 0.0f, 0.3f));
    assert(nearlyTo(yaw.diag.d, 0.2f, 1e-6f) && nearlyTo(yaw.diag.c, 0.0f, 1e-6f));
    assert(nearlyTo(yaw.leftA, -0.2f, 1e-4f) && nearlyTo(yaw.rightA, 0.2f, 1e-4f));
    assert(nearlyTo(yaw.diag.shapedYawRateRadPerSec, 0.3f, 1e-6f));

    // A repeated sequence is not a new sample, so the held outputs stay at -0.2 A and 0.2 A even though the
    // tilt error and rate in the held measurement would otherwise change the balance.
    Measurements repeat = atTime(makeMeasurements(), 25000);
    repeat.imuSequence = 3;
    repeat.tiltRad = kTrimTiltRad + 0.5f;
    repeat.tiltRateRadPerSec = 5.0f;
    repeat.yawRateRadPerSec = -2.0f;
    const Output held = bc.update(repeat, makeRequest(true), balance, 25000);
    assert(held.state == State::Run);
    assert(nearlyTo(held.leftA, -0.2f, 1e-4f) && nearlyTo(held.rightA, 0.2f, 1e-4f));
    assert(nearlyTo(held.diag.c, 0.0f, 1e-6f) && nearlyTo(held.diag.d, 0.2f, 1e-6f));
  }

  // Held demands are reallocated every tick: when the wheels speed up between IMU samples, the held 0.3 A
  // is clamped to the new envelope. At 100 rad/s and Vmax 8.4 V the limit is [-0.6324, 0.1312] A (bemf 5.5133 V, R 22).
  {
    Config pOnly = config;
    pOnly.tiltPAPerRad = 30.0f;
    pOnly.gyroDampingAPerRadPerSec = 0.0f;
    BalanceController bc;
    enterRun(bc, pOnly, 0);
    Measurements m = good;
    m.tiltRad = kTrimTiltRad + 0.01f;  // 30 * 0.01 = 0.3 A, inside the at-rest limit of 0.3636 A
    m.imuSequence = 2;
    const Output first = bc.update(atTime(m, 12000), makeRequest(true), pOnly, 12000);
    assert(first.state == State::Run && nearlyTo(first.leftA, 0.3f, 1e-4f));
    m.leftWheelRadPerSec = 100.0f;
    m.rightWheelRadPerSec = 100.0f;
    const Output held = bc.update(atTime(m, 15000), makeRequest(true), pOnly, 15000);
    assert(held.state == State::Run);
    assert(nearlyTo(held.leftA, 0.131214f, 1e-4f) && nearlyTo(held.rightA, 0.131214f, 1e-4f));
  }

  // Step 6, outputs: zero in Disabled, Arming, and Fault. The output is also zero for a fault latched by a stale IMU.
  {
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), balance, 1000);
    // Tilt error 0.5 is outside the 0.05 rad arming window, so the controller stays in Arming.
    const Output arming = imuSample(bc, balance, 2000, 1, ImuValues{0.5f, 0.0f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    assert(arming.state == State::Arming && !arming.driveEnabled);
    assert(arming.leftA == 0.0f && arming.rightA == 0.0f);

    BalanceController running;
    enterRun(running, balance, 0);
    const Output live = imuSample(running, balance, 12000, 2, ImuValues{0.01f, 0.2f, 0.0f, 0.0f, 0.0f},
                                  makeRequest(true));
    assert(live.driveEnabled && live.leftA > 0.0f);
    const Output off = tick(running, good, liveSwitch(30000, false), balance, 30000);
    assert(off.state == State::Disabled && !off.driveEnabled);
    assert(off.leftA == 0.0f && off.rightA == 0.0f);

    BalanceController stale;
    enterRun(stale, balance, 0);
    imuSample(stale, balance, 12000, 2, ImuValues{0.01f, 0.2f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    // IMU read at 0 us, checked at 30000 us: older than the 20 ms limit, so StaleImu latches.
    Measurements old = atTime(makeMeasurements(), 30000);
    old.imuSequence = 3;
    old.imuReadTimeUs = 0;
    const Output fault = stale.update(old, makeRequest(true), balance, 30000);
    assert(fault.state == State::Fault && fault.fault == FaultReason::StaleImu && !fault.driveEnabled);
    assert(fault.leftA == 0.0f && fault.rightA == 0.0f);
  }

  // Step 6, update(): a NaN tilt latches InvalidInput on its sample and the outputs are zero. The balance law and
  // the allocation would otherwise pass NaN to the motor targets.
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const Output out = imuSample(bc, balance, 12000, 2, ImuValues{nan, 0.0f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    assert(out.state == State::Fault && out.fault == FaultReason::InvalidInput && !out.driveEnabled);
    assert(out.leftA == 0.0f && out.rightA == 0.0f);
  }

  // Step 6, demandScale: reserve = min(cMax - c, c - cMin). A rate of 15.8 rad/s with gyro damping 0.5 gives
  // balance 7.9 A, so reserve = 8.4 - 7.9 = 0.5 A = reserveLowA / 2 and demandScale = 0.5. The next sample shapes
  // demand with that scale. demandScaleCapStart is 0.5, so the accel factor is 0 and the speed stays at zero, even
  // though the 1 m/s cruise cap is whole at this scale. (The earlier version halved accel and cap together and
  // expected 0.01 m/s.)
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    const Output first = imuSample(bc, balance, 12000, 2, ImuValues{0.0f, 15.8f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    assert(nearlyTo(first.diag.c, 7.9f, 1e-4f) && !first.diag.effortClipped);
    assert(nearlyTo(first.diag.reserve, 0.5f, 1e-4f) && nearlyTo(first.diag.demandScale, 0.5f, 1e-4f));

    const Output next = imuSample(bc, balance, 22000, 3, ImuValues{}, liveRequest(22000, 1.0f, 0.0f));
    assert(nearlyTo(next.diag.shapedSpeedMps, 0.0f, 1e-6f));
    // With zero effort the reserve is back at 8.4 A and the scale returns to 1.
    assert(nearlyTo(next.diag.demandScale, 1.0f, 1e-6f));
  }

  // Step 6, demandScale: a reserve of 0.25 A gives 0.25 (linear below reserveLowA).
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    // Balance 8.15 A = 0.5 * 16.3 rad/s, so reserve 0.25 A.
    const Output out = imuSample(bc, balance, 12000, 2, ImuValues{0.0f, 16.3f, 0.0f, 0.0f, 0.0f}, makeRequest(true));
    assert(nearlyTo(out.diag.reserve, 0.25f, 1e-4f) && nearlyTo(out.diag.demandScale, 0.25f, 1e-4f));
  }

  // Step 6, speed integral freeze. Wheels at 1 rad/s give a measured speed of 0.1 m/s; the shaped speed is 0 with
  // zero request, so the loop error is -0.1 m/s. Each 20 ms run then adds 0.2 * -0.1 * 0.02 = -0.0004 unless frozen.
  // Control first: no clipping, so the integral runs.
  {
    Config integ = balance;
    integ.speedIntegralEnabled = true;
    BalanceController bc;
    enterRun(bc, integ, 0);
    ImuValues wheels;
    wheels.leftRadPerSec = 1.0f;
    wheels.rightRadPerSec = 1.0f;
    imuSample(bc, integ, 12000, 2, wheels, makeRequest(true));
    const Output run = imuSample(bc, integ, 22000, 3, wheels, makeRequest(true));
    assert(nearlyTo(run.diag.speedI, -0.0004f, 1e-6f));
  }
  // Clipped: a 10 A balance request (rate 20 rad/s) clips c to about 8.345 A, so the speed integral holds through
  // the next loop run. Clipped c leaves zero reserve, so the reserve input freezes it too; this checks the freeze.
  {
    Config integ = balance;
    integ.speedIntegralEnabled = true;
    BalanceController bc;
    enterRun(bc, integ, 0);
    ImuValues clipped;
    clipped.tiltRateRadPerSec = 20.0f;
    clipped.leftRadPerSec = 1.0f;
    clipped.rightRadPerSec = 1.0f;
    const Output first = imuSample(bc, integ, 12000, 2, clipped, makeRequest(true));
    assert(first.diag.effortClipped);
    const Output frozen = imuSample(bc, integ, 22000, 3, clipped, makeRequest(true));
    assert(frozen.diag.effortClipped);
    assert(nearlyTo(frozen.diag.speedI, 0.0f, 1e-6f));
  }

  // Step 6, anti-windup hold: a flag set on every tick since the last loop run freezes the integral, not only a flag
  // on the tick just before the loop run. Control first: with no flag the integral moves.
  {
    const Config cfg = makeTrimConfig();
    assert(speedIAt(cfg, FlagSource::None, false, 1002000) > 0.05f);
    assert(speedIAt(cfg, FlagSource::Voltage, false, 1002000) == 0.0f);
    assert(speedIAt(cfg, FlagSource::Current, false, 1002000) == 0.0f);
    assert(speedIAt(cfg, FlagSource::Effort, false, 1002000) == 0.0f);
    assert(speedIAt(cfg, FlagSource::Reserve, false, 1002000) == 0.0f);
  }
  // A flag on a single tick (15000 us) freezes the first loop run after it (22000 us). The next run (42000 us)
  // integrates for the sensor flags. Effort and reserve are not checked there: the wide-envelope ticks make the
  // loop's 22000 us run see a 5 m/s error, and the tilt response then pushes the reserve below its limit by itself.
  {
    const Config cfg = makeTrimConfig();
    for (FlagSource source : {FlagSource::Voltage, FlagSource::Current, FlagSource::Effort, FlagSource::Reserve}) {
      assert(speedIAt(cfg, source, true, 22000) == 0.0f);
    }
    assert(speedIAt(cfg, FlagSource::Voltage, true, 42000) > 0.0f);
    assert(speedIAt(cfg, FlagSource::Current, true, 42000) > 0.0f);
  }

  // Step 6, authority fault timing: clipped effort at 12000 us starts the timer. The limit is 100000 us, so 112000
  // is not yet over and 112001 faults.
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    ImuValues clipped;
    clipped.tiltRateRadPerSec = 20.0f;
    const Output start = imuSample(bc, balance, 12000, 2, clipped, makeRequest(true));
    assert(start.state == State::Run && start.diag.effortClipped);
    assert(heldTick(bc, balance, 112000, 2).state == State::Run);
    const Output over = heldTick(bc, balance, 112001, 2);
    assert(over.state == State::Fault && over.fault == FaultReason::AuthorityExhausted && !over.driveEnabled);
    assert(over.leftA == 0.0f && over.rightA == 0.0f);
  }

  // Step 6, authority timer restart: an unclipped sample at 60000 us clears the timer. The next clipped sample
  // at 70000 us starts a new run, so the fault comes at 170001 us, not 112001 us.
  {
    BalanceController bc;
    enterRun(bc, balance, 0);
    ImuValues clipped;
    clipped.tiltRateRadPerSec = 20.0f;
    imuSample(bc, balance, 12000, 2, clipped, makeRequest(true));
    const Output unclipped = imuSample(bc, balance, 60000, 3, ImuValues{}, makeRequest(true));
    assert(!unclipped.diag.effortClipped);
    imuSample(bc, balance, 70000, 4, clipped, makeRequest(true));
    assert(heldTick(bc, balance, 112001, 4).state == State::Run);
    assert(heldTick(bc, balance, 170000, 4).state == State::Run);
    assert(heldTick(bc, balance, 170001, 4).state == State::Fault);
  }

  // Step 6, authority timer across micros() wraparound. The clip starts at 0xFFFFFFFF - 8000 us. Adding 100000 us
  // wraps to 91999 us, which is exactly the limit (not over); 92000 us is over.
  {
    const uint32_t base = 0xFFFFFFFFu - 20000u;
    BalanceController bc;
    enterRun(bc, balance, base);
    ImuValues clipped;
    clipped.tiltRateRadPerSec = 20.0f;
    const uint32_t clipTimeUs = base + 12000u;
    assert(imuSample(bc, balance, clipTimeUs, 2, clipped, makeRequest(true)).diag.effortClipped);
    assert(heldTick(bc, balance, 91999u, 2).state == State::Run);
    assert(heldTick(bc, balance, 92000u, 2).state == State::Fault);
  }

  // Step 6, configuration: a NaN and an infinite gain are refused at entry. A zero reserveLowA is refused too.
  // Covered in the block above.

  // Step 7, transfer: speed I of about -0.006 rad is moved into the trim at one tick. Speed P is -0.0025 rad and the
  // tilt error is 0.0085 rad, inside the 0.01 rad tolerance. The stored and applied references do not change.
  {
    const Config cfg = makeTrimConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    const ImuValues creeping{0.0f, 0.0f, 0.0f, 0.05f, 0.05f};  // measured 0.005 m/s
    Output last;
    // Loop runs fall on 22000 + 20000 j us; the last of these samples is at 602000 us.
    for (uint32_t k = 0; k < 60; ++k) {
      const uint32_t t = 12000u + 10000u * k;
      last = imuSample(bc, cfg, t, 2 + k, creeping, trimRequest(t, false));
    }
    assert(last.state == State::Run && last.diag.speedI < -0.001f);
    // 612000 us is not a loop run, so speed I and speed P are the values of the last loop.
    const Output moved = imuSample(bc, cfg, 612000u, 62, creeping, trimRequest(612000u, true));
    assert(moved.state == State::Commissioning && moved.driveEnabled && !moved.diag.commissioningRefused);
    assert(moved.diag.speedI == 0.0f);
    assert(nearlyTo(moved.diag.trimDeg, 10.0f + last.diag.speedI * kRadToDeg, 1e-3f));
    assert(nearlyTo(moved.diag.tiltRefRad, last.diag.tiltRefRad, 1e-4f));
    // Speed I stays zero in Commissioning: the 622000 us loop run does not integrate.
    const Output later = imuSample(bc, cfg, 622000u, 63, creeping, trimRequest(622000u, true));
    assert(later.state == State::Commissioning && later.diag.speedI == 0.0f);
    assert(nearlyTo(later.diag.tiltRefRad, last.diag.tiltRefRad, 1e-4f));
  }

  // Step 7, Commissioning keeps the balance law: a 0.02 rad tilt error and 0.1 rad/s tilt rate give
  // 2 * 0.02 + 0.5 * 0.1 = 0.09 A on each wheel, and the drivers stay enabled.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output out = imuSample(bc, cfg, 32000, 4, ImuValues{0.02f, 0.1f, 0.0f, 0.0f, 0.0f}, trimRequest(32000, true));
    assert(out.state == State::Commissioning && out.driveEnabled);
    assert(nearlyTo(out.leftA, 0.09f, 1e-4f) && nearlyTo(out.rightA, 0.09f, 1e-4f));
  }

  // Step 7, Commissioning -> Run: the switch off and a validated trim are both needed. The knob first moves the trim
  // to 10.5 deg, so a return to the stored 10 deg would show up.
  {
    const Config cfg = makeBalanceConfig();
    Config unvalidated = cfg;
    unvalidated.trimValidated = false;
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Output out;
    for (uint32_t i = 0; i < 9; ++i) {
      out = knobSample(bc, cfg, 32000u + 10000u * i, 4 + i, true, 10.5f);
    }
    assert(nearlyTo(out.diag.trimDeg, 10.5f, 1e-3f));
    // Switch off with an unvalidated trim: stays in Commissioning, drivers on.
    const Output stays = imuSample(bc, unvalidated, 122000u, 13, ImuValues{}, trimRequest(122000u, false));
    assert(stays.state == State::Commissioning && stays.driveEnabled && nearlyTo(stays.diag.trimDeg, 10.5f, 1e-3f));
    // Switch off with a validated trim, but the working 10.5 deg differs from the stored 10 deg: stays in
    // Commissioning, so an unsaved change cannot enter Run.
    const Output unsaved = imuSample(bc, cfg, 132000u, 14, ImuValues{}, trimRequest(132000u, false));
    assert(unsaved.state == State::Commissioning && unsaved.driveEnabled && !unsaved.diag.knobEngaged);
    assert(nearlyTo(unsaved.diag.trimDeg, 10.5f, 1e-3f));
  }
  // Working trim equal to the stored trim, validated, switch off: Run.
  {
    Config matched = makeBalanceConfig();
    matched.storedTrimDeg = 10.2f;
    BalanceController bc;
    enterCommissioning(bc, matched, 0);
    const Output back = imuSample(bc, matched, 32000u, 4, ImuValues{}, trimRequest(32000u, false));
    assert(back.state == State::Run && back.driveEnabled && !back.diag.knobEngaged);
    assert(nearlyTo(back.diag.trimDeg, 10.2f, 1e-3f));
  }

  // Step 7 refusals. Each block changes one condition and checks that Run stays, with the refusal flagged. The
  // balance and yaw gains are zero unless a block sets them, so the tilt reference stays at the trim.

  // Transfer request band: the raw request must lie within 0.02 m/s and 0.03 rad/s (the save bands, not the 0.05
  // arming bands). 0.04 m/s is inside the arming band, so it is refused here only by the save band.
  {
    const Config cfg = makeBalanceConfig();
    const auto transferWith = [&cfg](const Request &request) {
      BalanceController bc;
      enterRun(bc, cfg, 0);
      imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
      return imuSample(bc, cfg, 22000, 3, ImuValues{}, request);
    };
    const Output speedInside = transferWith(motionRequest(22000, 0.019f, true));
    assert(speedInside.state == State::Commissioning && !speedInside.diag.commissioningRefused);
    const Output speedOutside = transferWith(motionRequest(22000, 0.021f, true));
    assert(speedOutside.state == State::Run && speedOutside.diag.commissioningRefused);
    const Output speedBetween = transferWith(motionRequest(22000, 0.04f, true));
    assert(speedBetween.state == State::Run && speedBetween.diag.commissioningRefused);

    Request yawInside = trimRequest(22000, true);
    yawInside.yawRateRadPerSec = 0.029f;
    const Output yawIn = transferWith(yawInside);
    assert(yawIn.state == State::Commissioning && !yawIn.diag.commissioningRefused);
    Request yawOutside = trimRequest(22000, true);
    yawOutside.yawRateRadPerSec = 0.031f;
    const Output yawOut = transferWith(yawOutside);
    assert(yawOut.state == State::Run && yawOut.diag.commissioningRefused);
    Request yawBetween = trimRequest(22000, true);
    yawBetween.yawRateRadPerSec = 0.04f;
    const Output yawMid = transferWith(yawBetween);
    assert(yawMid.state == State::Run && yawMid.diag.commissioningRefused);
  }

  // Step 7, transfer: a voltage or current clip on the transfer tick refuses it. With neither flag it transfers.
  {
    const Config cfg = makeBalanceConfig();
    const auto transferWithFlags = [&cfg](bool voltageClipped, bool currentClipped) {
      BalanceController bc;
      enterRun(bc, cfg, 0);
      imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
      Measurements m = atTime(makeMeasurements(), 22000);
      m.imuSequence = 3;
      m.voltageClipped = voltageClipped;
      m.currentClipped = currentClipped;
      return bc.update(m, trimRequest(22000, true), cfg, 22000);
    };
    const Output clear = transferWithFlags(false, false);
    assert(clear.state == State::Commissioning && !clear.diag.commissioningRefused);
    const Output voltage = transferWithFlags(true, false);
    assert(voltage.state == State::Run && voltage.diag.commissioningRefused);
    const Output current = transferWithFlags(false, true);
    assert(current.state == State::Run && current.diag.commissioningRefused);
  }

  // Refused: nonzero shaped demand. The request ramps to 0.2 m/s; then the switch comes on and the shaped speed
  // ramps down at 4 m/s^2, 0.04 m/s per sample. The transfer waits for it to reach zero.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    uint32_t seq = 2;
    for (uint32_t k = 0; k <= 10; ++k) {
      const uint32_t t = 12000u + 10000u * k;
      imuSample(bc, cfg, t, seq++, ImuValues{}, motionRequest(t, 0.2f, false));
    }
    Output out;
    uint32_t t = 122000u;
    // Samples at 122000, 132000, 142000, 152000 hold 0.16, 0.12, 0.08, 0.04 m/s: refused each time.
    for (uint32_t k = 0; k < 4; ++k) {
      out = imuSample(bc, cfg, t, seq++, ImuValues{}, motionRequest(t, 0.0f, true));
      assert(out.state == State::Run && out.diag.commissioningRefused && out.diag.shapedSpeedMps > 0.0f);
      t += 10000u;
    }
    assert(nearlyTo(out.diag.shapedSpeedMps, 0.04f, 1e-4f));
    // The next sample reaches zero and the transfer goes through on that sample.
    out = imuSample(bc, cfg, t, seq++, ImuValues{}, motionRequest(t, 0.0f, true));
    assert(out.state == State::Commissioning && !out.diag.commissioningRefused);
    assert(nearlyTo(out.diag.shapedSpeedMps, 0.0f, 1e-6f));
  }

  // Refused: measured wheel speed beyond 0.01 m/s. Both wheels at 0.2 rad/s give 0.02 m/s.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    const Output out = imuSample(bc, cfg, 22000, 3, ImuValues{0.0f, 0.0f, 0.0f, 0.2f, 0.2f}, trimRequest(22000, true));
    assert(out.state == State::Run && out.diag.commissioningRefused);
  }

  // Refused: tilt error 0.02 rad, beyond the 0.01 rad tolerance.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    const Output out = imuSample(bc, cfg, 22000, 3, ImuValues{0.02f, 0.0f, 0.0f, 0.0f, 0.0f}, trimRequest(22000, true));
    assert(out.state == State::Run && out.diag.commissioningRefused);
  }

  // Refused: the speed correction clips the tilt reference at +0.3 rad. Speed P is 20 * 0.0075 = 0.15 rad, so the
  // reference reaches 0.3 rad and the tilt error stays at zero once the slew has landed on it.
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 20.0f;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    const ImuValues clipping{0.3f - kTrimTiltRad, 0.0f, 0.0f, -0.075f, -0.075f};  // measured -0.0075 m/s
    uint32_t seq = 2;
    for (uint32_t t = 12000u; t <= 212000u; t += 10000u) {
      imuSample(bc, cfg, t, seq++, clipping, trimRequest(t, false));
    }
    const Output out = imuSample(bc, cfg, 222000u, seq, clipping, trimRequest(222000u, true));
    assert(out.state == State::Run && out.diag.commissioningRefused);
    assert(out.diag.tiltClipped && nearlyTo(out.diag.tiltRefRad, 0.3f, 1e-4f));
  }

  // Refused: the stored reference is clipped at 0.3 rad, but the fresh speed P now puts trim + P at 0.29995 rad, just
  // inside the limit. The applied reference is within the transfer tolerance of that, so only the stored clip check
  // refuses the transfer.
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 20.0f;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    const ImuValues clipping{0.3f - kTrimTiltRad, 0.0f, 0.0f, -0.075f, -0.075f};  // measured -0.0075 m/s
    uint32_t seq = 2;
    for (uint32_t t = 12000u; t <= 202000u; t += 10000u) {
      imuSample(bc, cfg, t, seq++, clipping, trimRequest(t, false));
    }
    // 212000 us is not a loop run. Measured -0.00627 m/s gives P = 0.12542 rad.
    const ImuValues nearLimit{0.3f - kTrimTiltRad, 0.0f, 0.0f, -0.0627086f, -0.0627086f};
    const Output out = imuSample(bc, cfg, 212000u, seq, nearLimit, trimRequest(212000u, true));
    assert(out.state == State::Run && out.diag.commissioningRefused && out.diag.tiltClipped);
  }

  // Refused: effort clip. Tilt rate 20 rad/s gives 0.5 * 20 = 10 A, beyond the 8.4 A envelope, so c clips on that
  // sample. The next sample has no balance demand and the switch on; the refusal reads the previous tick's clip.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    const Output clipped =
        imuSample(bc, cfg, 22000, 3, ImuValues{0.0f, 20.0f, 0.0f, 0.0f, 0.0f}, trimRequest(22000, false));
    assert(clipped.state == State::Run && clipped.diag.effortClipped);
    const Output out = imuSample(bc, cfg, 32000, 4, ImuValues{}, trimRequest(32000, true));
    assert(out.state == State::Run && out.diag.commissioningRefused);
  }

  // Refused: low reserve, without an effort clip. Tilt rate 15.8 rad/s gives 0.5 * 15.8 = 7.9 A; the reserve is
  // 8.4 - 7.9 = 0.5 A, below reserveLowA 1.0 A.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    const Output low =
        imuSample(bc, cfg, 22000, 3, ImuValues{0.0f, 15.8f, 0.0f, 0.0f, 0.0f}, trimRequest(22000, false));
    assert(!low.diag.effortClipped && nearlyTo(low.diag.reserve, 0.5f, 1e-3f));
    const Output out = imuSample(bc, cfg, 32000, 4, ImuValues{}, trimRequest(32000, true));
    assert(out.state == State::Run && out.diag.commissioningRefused);
  }

  // Refused while the reference slews. Speed P becomes 2 * 0.00503 = 0.01006 rad at the 42000 us loop run. That
  // exceeds the 0.01 rad slew step by 0.00006 rad, which is inside the 1e-4 rad transfer tolerance, so only the
  // slew check refuses. The reference lands on the next sample, and the transfer goes through there.
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 2.0f;
    cfg.standstillTiltErrorTolRad = 0.05f;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    imuSample(bc, cfg, 22000, 3, ImuValues{}, trimRequest(22000, false));  // loop run: P 0
    const ImuValues moving{0.0f, 0.0f, 0.0f, -0.0503f, -0.0503f};        // measured -0.00503 m/s
    imuSample(bc, cfg, 32000, 4, moving, trimRequest(32000, false));       // no loop run
    const Output slewing = imuSample(bc, cfg, 42000, 5, moving, trimRequest(42000, true));
    assert(slewing.state == State::Run && slewing.diag.commissioningRefused);
    const Output landed = imuSample(bc, cfg, 52000, 6, moving, trimRequest(52000, true));
    assert(landed.state == State::Commissioning && !landed.diag.commissioningRefused);
    assert(nearlyTo(landed.diag.trimDeg, 10.0f, 1e-3f));
  }

  // Refused: reference jump. Speed P is 0 in the stored value, but a fresh P at 32000 us (no loop run) is
  // 0.5 * 0.008 = 0.004 rad. The 42000 us loop run stores that P, and the transfer then goes through.
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 0.5f;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    imuSample(bc, cfg, 22000, 3, ImuValues{}, trimRequest(22000, false));
    const ImuValues moving{0.0f, 0.0f, 0.0f, -0.08f, -0.08f};
    const Output jump = imuSample(bc, cfg, 32000, 4, moving, trimRequest(32000, true));
    assert(jump.state == State::Run && jump.diag.commissioningRefused);
    const Output transferred = imuSample(bc, cfg, 42000, 5, moving, trimRequest(42000, true));
    assert(transferred.state == State::Commissioning && !transferred.diag.commissioningRefused);
  }

  // Refused: the fresh speed P takes trim + P past the limit. The stored reference is 0.2999829 rad, just inside the
  // 0.3 rad limit, so the jump check passes (1.7e-5 rad) and only the new clamp check refuses. The loop runs with
  // measured -0.0062725 m/s (P 0.12545 rad); the transition tick measures -0.006275 m/s (P 0.1255 rad).
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 20.0f;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    const ImuValues stored{0.12545f, 0.0f, 0.0f, -0.062725f, -0.062725f};  // tilt tracks the reference
    uint32_t seq = 2;
    for (uint32_t t = 12000u; t <= 202000u; t += 10000u) {
      imuSample(bc, cfg, t, seq++, stored, trimRequest(t, false));
    }
    const ImuValues fresh{0.12545f, 0.0f, 0.0f, -0.06275f, -0.06275f};
    const Output out = imuSample(bc, cfg, 212000u, seq, fresh, trimRequest(212000u, true));
    assert(out.state == State::Run && out.diag.commissioningRefused && !out.diag.tiltClipped);
    assert(nearlyTo(out.diag.tiltRefRad, 0.2999829f, 1e-5f));
  }

  // Refused: trim plus speed I beyond the envelope. Both cases integrate the same speed error. With a 0.18 rad
  // envelope the sum 0.1745 + 0.0074 is refused; with 0.2 rad the same state transfers.
  {
    Config tight = makeBalanceConfig();
    tight.speedKiRadPerMpsS = 1.0f;
    tight.speedIntegralEnabled = true;
    tight.standstillTiltErrorTolRad = 0.05f;
    tight.trimEnvelopeRad = 0.18f;
    const Output refused = integralThenTransfer(tight);
    assert(refused.state == State::Run && refused.diag.commissioningRefused);
    assert(refused.diag.speedI > 0.0055f);

    Config wide = tight;
    wide.trimEnvelopeRad = 0.2f;
    const Output moved = integralThenTransfer(wide);
    assert(moved.state == State::Commissioning && !moved.diag.commissioningRefused);
    assert(nearlyTo(moved.diag.trimDeg, 10.0f + refused.diag.speedI * kRadToDeg, 1e-3f));
  }

  // Step 7, knob. Commissioning from 22000 us; the trim starts at 10 deg (0.174533 rad).
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    // 12 deg is 0.0349 rad from the trim, outside the 0.02 rad band: no pickup, no movement.
    Output out;
    for (uint32_t i = 0; i < 3; ++i) {
      out = knobSample(bc, cfg, 32000u + 10000u * i, 4 + i, true, 12.0f);
    }
    assert(out.state == State::Commissioning && !out.diag.knobEngaged && nearlyTo(out.diag.trimDeg, 10.0f, 1e-4f));

    // 10.5 deg is 0.00873 rad from the trim, inside the band: the pickup and the first 0.001 rad step come together.
    const float targetRad = 10.5f * kDegToRad;
    const Output first = knobSample(bc, cfg, 62000u, 7, true, 10.5f);
    assert(first.diag.knobEngaged && nearlyTo(first.diag.trimDeg * kDegToRad, kTrimTiltRad + 0.001f, 1e-5f));
    float previousRad = first.diag.trimDeg * kDegToRad;
    // Slew at 0.1 rad/s over 0.01 s: 0.001 rad per sample. The eighth step after pickup (142000 us) would be the
    // ninth sample; the remaining 0.000727 rad is less than a step, so the trim lands exactly on the knob there.
    for (uint32_t i = 1; i <= 7; ++i) {
      const Output step = knobSample(bc, cfg, 62000u + 10000u * i, 7 + i, true, 10.5f);
      const float rad = step.diag.trimDeg * kDegToRad;
      assert(nearlyTo(rad - previousRad, 0.001f, 1e-5f));
      previousRad = rad;
    }
    const Output landed = knobSample(bc, cfg, 132000u + 10000u, 15, true, 10.5f);
    assert(landed.diag.knobEngaged && nearlyTo(landed.diag.trimDeg * kDegToRad - previousRad, 0.000727f, 1e-5f));
    assert(nearlyTo(landed.diag.trimDeg * kDegToRad, targetRad, 1e-6f));

    // Clamp at the envelope: a 20 deg knob is engaged, so no pickup is needed. The target is clamped to 0.2 rad, and
    // the trim slews there and stops, 0.01674 rad away, in 17 steps.
    for (uint32_t i = 0; i < 20; ++i) {
      out = knobSample(bc, cfg, 152000u + 10000u * i, 16 + i, true, 20.0f);
    }
    assert(out.diag.knobEngaged && nearlyTo(out.diag.trimDeg * kDegToRad, 0.2f, 1e-5f));
    assert(nearlyTo(out.diag.trimDeg, 0.2f * kRadToDeg, 1e-3f));

    // Invalid knob disengages and holds the trim. Re-engaging needs a knob inside the band; 20 deg is not.
    const Output invalid = knobSample(bc, cfg, 352000u, 36, false, 20.0f);
    assert(!invalid.diag.knobEngaged && nearlyTo(invalid.diag.trimDeg, 0.2f * kRadToDeg, 1e-3f));
    const Output still = knobSample(bc, cfg, 362000u, 37, true, 20.0f);
    assert(!still.diag.knobEngaged && nearlyTo(still.diag.trimDeg, 0.2f * kRadToDeg, 1e-3f));
    // A knob at 0.2 rad (11.459 deg) is inside the band, so it engages again. Then a NaN knob disengages it.
    const Output reengaged = knobSample(bc, cfg, 372000u, 38, true, 0.2f * kRadToDeg);
    assert(reengaged.diag.knobEngaged);
    const Output nanKnob = knobSample(bc, cfg, 382000u, 39, true, std::numeric_limits<float>::quiet_NaN());
    assert(!nanKnob.diag.knobEngaged && nearlyTo(nanKnob.diag.trimDeg, 0.2f * kRadToDeg, 1e-3f));
  }

  // Step 7, RC loss with a source that is invalid: the knob disengages and the trim is held. After the source
  // returns, a knob outside the band of the held trim does not move it, and one inside the band engages again.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Output out;
    for (uint32_t i = 0; i < 9; ++i) {
      out = knobSample(bc, cfg, 32000u + 10000u * i, 4 + i, true, 10.5f);
    }
    assert(out.diag.knobEngaged && nearlyTo(out.diag.trimDeg, 10.5f, 1e-3f));

    Request lost = trimRequest(122000u, true, true, 10.5f);
    lost.sourceValid = false;
    const Output held = imuSample(bc, cfg, 122000u, 13, ImuValues{}, lost);
    assert(!held.diag.knobEngaged && held.state == State::Commissioning && held.driveEnabled);
    assert(nearlyTo(held.diag.trimDeg, 10.5f, 1e-3f));

    Request lostFar = trimRequest(132000u, true, true, 20.0f);
    lostFar.sourceValid = false;
    const Output stillHeld = imuSample(bc, cfg, 132000u, 14, ImuValues{}, lostFar);
    assert(!stillHeld.diag.knobEngaged && nearlyTo(stillHeld.diag.trimDeg, 10.5f, 1e-3f));

    const Output returnedFar = imuSample(bc, cfg, 142000u, 15, ImuValues{}, trimRequest(142000u, true, true, 20.0f));
    assert(!returnedFar.diag.knobEngaged && nearlyTo(returnedFar.diag.trimDeg, 10.5f, 1e-3f));
    const Output returnedNear = imuSample(bc, cfg, 152000u, 16, ImuValues{}, trimRequest(152000u, true, true, 10.5f));
    assert(returnedNear.diag.knobEngaged && nearlyTo(returnedNear.diag.trimDeg, 10.5f, 1e-3f));
  }

  // Step 7, RC loss by a request that goes stale across the micros() wrap. The request is received at base + 32000 us,
  // 8000 us before the wrap. It is 50000 us old at base + 82000 us, still fresh, and 60000 us old at base + 92000 us,
  // after the wrap, so it is stale there. The knob at 20 deg would move the trim if the request were used.
  {
    const Config cfg = makeBalanceConfig();
    const uint32_t base = 0xFFFFFFFFu - 40000u;
    const uint32_t received = base + 32000u;
    BalanceController bc;
    enterCommissioning(bc, cfg, base);
    Output out = knobSample(bc, cfg, received, 4, true, 10.5f);
    assert(out.diag.knobEngaged);
    uint32_t seq = 5;
    for (uint32_t t = received + 10000u; t <= received + 50000u; t += 10000u) {
      Request fresh = trimRequest(t, true, true, 10.5f);
      fresh.receivedTimeUs = received;
      out = imuSample(bc, cfg, t, seq++, ImuValues{}, fresh);
    }
    assert(out.diag.knobEngaged);
    const float heldDeg = out.diag.trimDeg;

    Request stale = trimRequest(received + 60000u, true, true, 20.0f);
    stale.receivedTimeUs = received;
    out = imuSample(bc, cfg, received + 60000u, seq, ImuValues{}, stale);
    assert(!out.diag.knobEngaged && nearlyTo(out.diag.trimDeg, heldDeg, 1e-4f));
    assert(out.state == State::Commissioning && out.driveEnabled);
  }

  // Step 7, the knob is ignored in Run. A knob 0.00873 rad from the trim (inside the band) does not engage.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false, true, 10.5f));
    const Output out = imuSample(bc, cfg, 22000, 3, ImuValues{}, trimRequest(22000, false, true, 10.5f));
    assert(out.state == State::Run && !out.diag.knobEngaged && nearlyTo(out.diag.trimDeg, 10.0f, 1e-3f));
  }

  // Step 7, faults latch from Commissioning, and the enable switch off returns to Disabled.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Measurements old = atTime(makeMeasurements(), 32000u);
    old.imuSequence = 4;
    old.imuReadTimeUs = 0;  // 32000 us old: stale
    const Output fault = bc.update(old, trimRequest(32000u, true), cfg, 32000u);
    assert(fault.state == State::Fault && fault.fault == FaultReason::StaleImu && !fault.driveEnabled);
    assert(fault.leftA == 0.0f && fault.rightA == 0.0f);
  }
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output off = tick(bc, good, liveSwitch(32000u, false), cfg, 32000u);
    assert(off.state == State::Disabled && !off.driveEnabled && off.diag.trimDeg == 0.0f);
  }

  // Step 7, configuration: the trim envelope must lie within the tilt limit, the stored trim within the envelope,
  // and each new value must be positive.
  {
    auto armedWith = [&good](const Config &c) {
      BalanceController bc;
      tick(bc, good, liveSwitch(1000u, false), c, 1000u);
      tick(bc, good, liveSwitch(2000u, true), c, 2000u);
      return tick(bc, good, liveSwitch(3000u, true), c, 3000u);
    };
    Config beyondLimit = makeBalanceConfig();
    beyondLimit.trimEnvelopeRad = 0.31f;
    const Output a = armedWith(beyondLimit);
    assert(a.state == State::Fault && a.fault == FaultReason::InvalidConfig && !a.driveEnabled);

    Config noBand = makeBalanceConfig();
    noBand.knobPickupBandRad = 0.0f;
    const Output b = armedWith(noBand);
    assert(b.state == State::Fault && b.fault == FaultReason::InvalidConfig);

    Config storedOutside = makeBalanceConfig();
    storedOutside.trimEnvelopeRad = 0.17f;  // the stored 10 deg (0.1745 rad) lies outside
    const Output c = armedWith(storedOutside);
    assert(c.state == State::Fault && c.fault == FaultReason::InvalidConfig);

    // Save band: finite and not negative. Demand scale cap start: strictly between 0 and 1.
    Config negativeSaveBand = makeBalanceConfig();
    negativeSaveBand.saveRequestYawTolRadPerSec = -0.01f;
    const Output d = armedWith(negativeSaveBand);
    assert(d.state == State::Fault && d.fault == FaultReason::InvalidConfig);

    Config capStartAtOne = makeBalanceConfig();
    capStartAtOne.demandScaleCapStart = 1.0f;
    const Output e = armedWith(capStartAtOne);
    assert(e.state == State::Fault && e.fault == FaultReason::InvalidConfig);

    Config capStartAtZero = makeBalanceConfig();
    capStartAtZero.demandScaleCapStart = 0.0f;
    const Output f = armedWith(capStartAtZero);
    assert(f.state == State::Fault && f.fault == FaultReason::InvalidConfig);
  }

  // Step 7b, granted save: one pulse with the working trim in degrees, then the switch must go off and on again.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);  // Commissioning at 22000 us, save switch off
    const Output saved = imuSample(bc, cfg, 32000, 4, ImuValues{}, saveRequest(32000, true));
    assert(saved.state == State::Commissioning && saved.saveTrimRequested && !saved.diag.saveRefused);
    assert(nearlyTo(saved.trimToSaveDeg, 10.0f, 1e-3f));
    // The switch held on gives no second save, and the pulse is gone on the next tick.
    const Output held = imuSample(bc, cfg, 42000, 5, ImuValues{}, saveRequest(42000, true));
    assert(!held.saveTrimRequested && held.trimToSaveDeg == 0.0f && !held.diag.saveRefused);
    imuSample(bc, cfg, 52000, 6, ImuValues{}, saveRequest(52000, false));
    const Output again = imuSample(bc, cfg, 62000, 7, ImuValues{}, saveRequest(62000, true));
    assert(again.saveTrimRequested && nearlyTo(again.trimToSaveDeg, 10.0f, 1e-3f));
    const Output after = imuSample(bc, cfg, 72000, 8, ImuValues{}, saveRequest(72000, true));
    assert(!after.saveTrimRequested && after.trimToSaveDeg == 0.0f);
  }

  // Step 7b, working trim: a knob at 10.5 deg lands in nine 0.001 rad steps from 32000 us. The save edge at 122000 us
  // is granted with 10.5 deg. Before that, the save edge at 32000 us is refused while the trim is still slewing.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request firstEdge = trimRequest(32000u, true, true, 10.5f);
    firstEdge.saveSwitch = true;
    const Output slewing = imuSample(bc, cfg, 32000u, 4, ImuValues{}, firstEdge);
    assert(slewing.state == State::Commissioning && slewing.diag.saveRefused && !slewing.saveTrimRequested);
    assert(slewing.trimToSaveDeg == 0.0f);
    Output out;
    for (uint32_t i = 1; i < 9; ++i) {
      Request step = trimRequest(32000u + 10000u * i, true, true, 10.5f);
      step.saveSwitch = true;
      out = imuSample(bc, cfg, 32000u + 10000u * i, 4 + i, ImuValues{}, step);
      assert(!out.saveTrimRequested && !out.diag.saveRefused);
    }
    assert(nearlyTo(out.diag.trimDeg, 10.5f, 1e-3f));
    imuSample(bc, cfg, 122000u, 13, ImuValues{}, trimRequest(122000u, true, true, 10.5f));
    Request edge = trimRequest(132000u, true, true, 10.5f);
    edge.saveSwitch = true;
    const Output saved = imuSample(bc, cfg, 132000u, 14, ImuValues{}, edge);
    assert(saved.saveTrimRequested && !saved.diag.saveRefused);
    assert(nearlyTo(saved.trimToSaveDeg, 10.5f, 1e-3f));
  }

  // Step 7b, the save level already on at Commissioning entry does not save. Only an off-to-on edge does.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    Request entry = trimRequest(22000, true);
    entry.saveSwitch = true;
    const Output in = imuSample(bc, cfg, 22000, 3, ImuValues{}, entry);
    assert(in.state == State::Commissioning && !in.saveTrimRequested && !in.diag.saveRefused);
    const Output still = imuSample(bc, cfg, 32000, 4, ImuValues{}, saveRequest(32000, true));
    assert(!still.saveTrimRequested && !still.diag.saveRefused);
    imuSample(bc, cfg, 42000, 5, ImuValues{}, saveRequest(42000, false));
    const Output on = imuSample(bc, cfg, 52000, 6, ImuValues{}, saveRequest(52000, true));
    assert(on.saveTrimRequested && nearlyTo(on.trimToSaveDeg, 10.0f, 1e-3f));
  }

  // Step 7b, the save switch is ignored in Run: an edge there is neither a save nor a refusal.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    Request saveOn = trimRequest(22000, false);
    saveOn.saveSwitch = true;
    const Output on = imuSample(bc, cfg, 22000, 3, ImuValues{}, saveOn);
    assert(on.state == State::Run && !on.saveTrimRequested && !on.diag.saveRefused);
    const Output off = imuSample(bc, cfg, 32000, 4, ImuValues{}, trimRequest(32000, false));
    assert(off.state == State::Run && !off.saveTrimRequested && !off.diag.saveRefused);
    Request saveAgain = trimRequest(42000, false);
    saveAgain.saveSwitch = true;
    const Output again = imuSample(bc, cfg, 42000, 5, ImuValues{}, saveAgain);
    assert(again.state == State::Run && !again.saveTrimRequested && !again.diag.saveRefused);
  }

  // Step 7b, each refusal: one condition is violated on the save edge. The output shows saveRefused, no pulse, and
  // stays in Commissioning. Commissioning is entered at 22000 us, so the edge is on the next sample at 32000 us.
  {
    const Config cfg = makeBalanceConfig();
    // Demand: the request speed beyond the 0.02 m/s save band. Commissioning ignores it for motion.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request fast = saveRequest(32000, true);
    fast.speedMps = 0.2f;
    const Output demand = imuSample(bc, cfg, 32000, 4, ImuValues{}, fast);
    assert(demand.state == State::Commissioning && demand.diag.saveRefused && !demand.saveTrimRequested);
    assert(demand.trimToSaveDeg == 0.0f);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Demand: the request yaw beyond the 0.03 rad/s save band.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request turning = saveRequest(32000, true);
    turning.yawRateRadPerSec = 0.2f;
    const Output demand = imuSample(bc, cfg, 32000, 4, ImuValues{}, turning);
    assert(demand.state == State::Commissioning && demand.diag.saveRefused && !demand.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Wheel speed: both wheels at 0.2 rad/s give 0.02 m/s, beyond the 0.01 m/s tolerance.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output moving =
        imuSample(bc, cfg, 32000, 4, ImuValues{0.0f, 0.0f, 0.0f, 0.2f, 0.2f}, saveRequest(32000, true));
    assert(moving.state == State::Commissioning && moving.diag.saveRefused && !moving.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Tilt error: 0.02 rad beyond the 0.01 rad tolerance.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output tilted =
        imuSample(bc, cfg, 32000, 4, ImuValues{0.02f, 0.0f, 0.0f, 0.0f, 0.0f}, saveRequest(32000, true));
    assert(tilted.state == State::Commissioning && tilted.diag.saveRefused && !tilted.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Effort clip: a tilt rate of 20 rad/s gives 10 A, beyond the 8.4 A envelope.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output clipped =
        imuSample(bc, cfg, 32000, 4, ImuValues{0.0f, 20.0f, 0.0f, 0.0f, 0.0f}, saveRequest(32000, true));
    assert(clipped.state == State::Commissioning && clipped.diag.effortClipped);
    assert(clipped.diag.saveRefused && !clipped.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Low reserve without an effort clip: 7.9 A on the balance leaves 0.5 A of reserve, below 1.0 A.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output low =
        imuSample(bc, cfg, 32000, 4, ImuValues{0.0f, 15.8f, 0.0f, 0.0f, 0.0f}, saveRequest(32000, true));
    assert(!low.diag.effortClipped && nearlyTo(low.diag.reserve, 0.5f, 1e-3f));
    assert(low.diag.saveRefused && !low.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Knob pending: the knob at 10.5 deg engages on this sample, so the trim is still slewing toward it.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request pending = trimRequest(32000u, true, true, 10.5f);
    pending.saveSwitch = true;
    const Output out = imuSample(bc, cfg, 32000u, 4, ImuValues{}, pending);
    assert(out.diag.knobEngaged && out.diag.saveRefused && !out.saveTrimRequested);
  }
  {
    const Config cfg = makeBalanceConfig();
    // Request lost: a switch level on a lost frame is not an edge, so it is neither a save nor a refusal.
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request lost = saveRequest(32000, true);
    lost.sourceValid = false;
    const Output out = imuSample(bc, cfg, 32000, 4, ImuValues{}, lost);
    assert(out.state == State::Commissioning && !out.diag.saveRefused && !out.saveTrimRequested);
  }

  // Step 7b, a refused edge does not wait for the conditions: they hold on the next tick, but the switch level has not
  // changed, so there is no save until it goes off and on.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output refused =
        imuSample(bc, cfg, 32000, 4, ImuValues{0.02f, 0.0f, 0.0f, 0.0f, 0.0f}, saveRequest(32000, true));
    assert(refused.diag.saveRefused);
    const Output settled = imuSample(bc, cfg, 42000, 5, ImuValues{}, saveRequest(42000, true));
    assert(!settled.saveTrimRequested && !settled.diag.saveRefused);
    imuSample(bc, cfg, 52000, 6, ImuValues{}, saveRequest(52000, false));
    const Output retried = imuSample(bc, cfg, 62000, 7, ImuValues{}, saveRequest(62000, true));
    assert(retried.saveTrimRequested && nearlyTo(retried.trimToSaveDeg, 10.0f, 1e-3f));
  }

  // Step 7b, a lost request refuses entry from Run with the switch on. Run stays, and the refusal is flagged.
  // A valid request with the switch on then enters Commissioning.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    Request lost = trimRequest(22000, true);
    lost.sourceValid = false;
    const Output refused = imuSample(bc, cfg, 22000, 3, ImuValues{}, lost);
    assert(refused.state == State::Run && refused.driveEnabled && refused.diag.commissioningRefused);
    const Output entered = imuSample(bc, cfg, 32000, 4, ImuValues{}, trimRequest(32000, true));
    assert(entered.state == State::Commissioning && !entered.diag.commissioningRefused);
  }
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    // Stale rather than a bad source: received at 0 us, 60000 us old at 60000 us, beyond the 50000 us age.
    Request stale = trimRequest(60000u, true);
    stale.receivedTimeUs = 0;
    const Output refused = imuSample(bc, cfg, 60000u, 3, ImuValues{}, stale);
    assert(refused.state == State::Run && refused.diag.commissioningRefused);
  }

  // Step 7b, a lost request does not leave Commissioning, even with the switch off. A valid request with the switch off
  // leaves it.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    Request lost = trimRequest(32000u, false);
    lost.sourceValid = false;
    const Output held = imuSample(bc, cfg, 32000u, 4, ImuValues{}, lost);
    assert(held.state == State::Commissioning && held.driveEnabled);
    Request stillLost = trimRequest(42000u, false);
    stillLost.sourceValid = false;
    const Output stillHeld = imuSample(bc, cfg, 42000u, 5, ImuValues{}, stillLost);
    assert(stillHeld.state == State::Commissioning && stillHeld.driveEnabled);
    const Output back = imuSample(bc, cfg, 52000u, 6, ImuValues{}, trimRequest(52000u, false));
    assert(back.state == State::Run && back.driveEnabled);
  }

  // Review fix 2: a lost request blocks arming. The switch edge on a lost frame is consumed, so holding it on does not
  // arm later; a live off-to-on edge does.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    tick(bc, good, liveSwitch(1000u, false), cfg, 1000u);
    Request lostOn = liveSwitch(2000u, true);
    lostOn.sourceValid = false;
    assert(tick(bc, good, lostOn, cfg, 2000u).state == State::Disabled);
    assert(tick(bc, good, liveSwitch(3000u, true), cfg, 3000u).state == State::Disabled);
    tick(bc, good, liveSwitch(4000u, false), cfg, 4000u);
    assert(tick(bc, good, liveSwitch(5000u, true), cfg, 5000u).state == State::Arming);
  }
  // Review fix 2: a lost request holds Arming, and the arming timeout still applies while it is lost.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    tick(bc, good, liveSwitch(1000u, false), cfg, 1000u);
    assert(tick(bc, good, liveSwitch(2000u, true), cfg, 2000u).state == State::Arming);
    Request lost = liveSwitch(3000u, true);
    lost.sourceValid = false;
    assert(tick(bc, good, lost, cfg, 3000u).state == State::Arming);
    assert(tick(bc, good, lost, cfg, 4000u).state == State::Arming);
    assert(tick(bc, good, liveSwitch(5000u, true), cfg, 5000u).state == State::Run);
  }
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    tick(bc, good, liveSwitch(1000u, false), cfg, 1000u);
    assert(tick(bc, good, liveSwitch(2000u, true), cfg, 2000u).state == State::Arming);
    // Past the 1000000 us arming timeout from 2000 us, with the request still lost: Disabled.
    Request lost = liveSwitch(1002001u, true);
    lost.sourceValid = false;
    assert(tick(bc, good, lost, cfg, 1002001u).state == State::Disabled);
  }

  // Review fix 3: lost frames end save tracking. The switch high when RC returns is a level, not an edge: no pulse.
  // A fresh off-to-on edge afterwards saves.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);  // Commissioning at 22000 us, save switch off
    Request lostLow = trimRequest(32000u, true);
    lostLow.sourceValid = false;
    const Output lostTick = imuSample(bc, cfg, 32000u, 4, ImuValues{}, lostLow);
    assert(lostTick.state == State::Commissioning && !lostTick.saveTrimRequested && !lostTick.diag.saveRefused);
    const Output returned = imuSample(bc, cfg, 42000u, 5, ImuValues{}, saveRequest(42000u, true));
    assert(returned.state == State::Commissioning && !returned.saveTrimRequested && !returned.diag.saveRefused);
    imuSample(bc, cfg, 52000u, 6, ImuValues{}, saveRequest(52000u, false));
    const Output saved = imuSample(bc, cfg, 62000u, 7, ImuValues{}, saveRequest(62000u, true));
    assert(saved.saveTrimRequested && nearlyTo(saved.trimToSaveDeg, 10.0f, 1e-3f));
  }

  // Review fix 4: a deflected stick blocks the Run -> Commissioning transfer, even when shaping has zeroed the demand
  // under the switch. Neutral afterwards transfers. Yaw is checked the same way.
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    const Output deflected = imuSample(bc, cfg, 22000, 3, ImuValues{}, motionRequest(22000, 0.2f, true));
    assert(deflected.state == State::Run && deflected.diag.commissioningRefused);
    const Output neutral = imuSample(bc, cfg, 32000, 4, ImuValues{}, trimRequest(32000, true));
    assert(neutral.state == State::Commissioning && !neutral.diag.commissioningRefused);
  }
  {
    const Config cfg = makeBalanceConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    imuSample(bc, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
    Request turning = trimRequest(22000u, true);
    turning.yawRateRadPerSec = 0.2f;
    const Output deflected = imuSample(bc, cfg, 22000, 3, ImuValues{}, turning);
    assert(deflected.state == State::Run && deflected.diag.commissioningRefused);
  }

  // Save request band: 0.02 m/s and 0.03 rad/s, not the 0.05 arming bands. 0.04 m/s is refused by the save band alone.
  {
    const Config cfg = makeBalanceConfig();
    const auto saveWith = [&cfg](const Request &request) {
      BalanceController bc;
      enterCommissioning(bc, cfg, 0);
      return imuSample(bc, cfg, 32000, 4, ImuValues{}, request);
    };
    Request speedInside = saveRequest(32000, true);
    speedInside.speedMps = 0.019f;
    const Output speedIn = saveWith(speedInside);
    assert(speedIn.saveTrimRequested && !speedIn.diag.saveRefused);
    Request speedOutside = saveRequest(32000, true);
    speedOutside.speedMps = 0.021f;
    const Output speedOut = saveWith(speedOutside);
    assert(!speedOut.saveTrimRequested && speedOut.diag.saveRefused);
    Request speedBetween = saveRequest(32000, true);
    speedBetween.speedMps = 0.04f;
    const Output speedMid = saveWith(speedBetween);
    assert(!speedMid.saveTrimRequested && speedMid.diag.saveRefused);

    Request yawInside = saveRequest(32000, true);
    yawInside.yawRateRadPerSec = 0.029f;
    const Output yawIn = saveWith(yawInside);
    assert(yawIn.saveTrimRequested && !yawIn.diag.saveRefused);
    Request yawOutside = saveRequest(32000, true);
    yawOutside.yawRateRadPerSec = 0.031f;
    const Output yawOut = saveWith(yawOutside);
    assert(!yawOut.saveTrimRequested && yawOut.diag.saveRefused);
    Request yawBetween = saveRequest(32000, true);
    yawBetween.yawRateRadPerSec = 0.04f;
    const Output yawMid = saveWith(yawBetween);
    assert(!yawMid.saveTrimRequested && yawMid.diag.saveRefused);
  }

  // Step 7b, a save edge with a voltage or current clip on that tick is refused. With neither flag it is granted.
  {
    const Config cfg = makeBalanceConfig();
    const auto saveWithFlags = [&cfg](bool voltageClipped, bool currentClipped) {
      BalanceController bc;
      enterCommissioning(bc, cfg, 0);
      Measurements m = atTime(makeMeasurements(), 32000);
      m.imuSequence = 4;
      m.voltageClipped = voltageClipped;
      m.currentClipped = currentClipped;
      return bc.update(m, saveRequest(32000, true), cfg, 32000);
    };
    const Output clear = saveWithFlags(false, false);
    assert(clear.saveTrimRequested && !clear.diag.saveRefused);
    const Output voltage = saveWithFlags(true, false);
    assert(!voltage.saveTrimRequested && voltage.diag.saveRefused && voltage.state == State::Commissioning);
    const Output current = saveWithFlags(false, true);
    assert(!current.saveTrimRequested && current.diag.saveRefused && current.state == State::Commissioning);
  }

  // Clip flags: a voltage clip or a current clip on the sample at 32000 us freezes the speed integral on the next
  // loop run at 42000 us. Wheels at -1 rad/s give a 0.004 rad step per loop run, so the integral reads 0.004 when
  // frozen and 0.008 when it moves. Each flag is tested alone, and neither effort nor reserve is clipped.
  {
    const auto speedIAfterFlags = [](bool voltageClipped, bool currentClipped) {
      const Config cfg = makeTrimConfig();
      BalanceController bc;
      enterRun(bc, cfg, 0);
      wheelTick(bc, cfg, 12000, 2, -1.0f);
      assert(nearlyTo(wheelTick(bc, cfg, 22000, 3, -1.0f).diag.speedI, 0.004f, 1e-6f));
      wheelTick(bc, cfg, 32000, 4, -1.0f, voltageClipped, currentClipped);
      return wheelTick(bc, cfg, 42000, 5, -1.0f);
    };
    const Output none = speedIAfterFlags(false, false);
    assert(nearlyTo(none.diag.speedI, 0.008f, 1e-6f));
    const Output voltage = speedIAfterFlags(true, false);
    assert(nearlyTo(voltage.diag.speedI, 0.004f, 1e-6f) && !voltage.diag.effortClipped);
    assert(voltage.diag.reserve >= makeTrimConfig().reserveLowA);
    const Output current = speedIAfterFlags(false, true);
    assert(nearlyTo(current.diag.speedI, 0.004f, 1e-6f) && !current.diag.effortClipped);
    assert(current.diag.reserve >= makeTrimConfig().reserveLowA);
  }

  // Clip flags are reported in diagnostics in Run, as measured, and are false once the controller is Disabled.
  {
    const Config cfg = makeTrimConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    wheelTick(bc, cfg, 12000, 2, 0.0f);
    const Output voltage = wheelTick(bc, cfg, 22000, 3, 0.0f, true, false);
    assert(voltage.state == State::Run && voltage.diag.voltageClipped && !voltage.diag.currentClipped);
    const Output both = wheelTick(bc, cfg, 32000, 4, 0.0f, true, true);
    assert(both.diag.voltageClipped && both.diag.currentClipped);
    Measurements off = atTime(makeMeasurements(), 42000);
    off.imuSequence = 5;
    off.voltageClipped = true;
    off.currentClipped = true;
    const Output disabled = bc.update(off, liveSwitch(42000, false), cfg, 42000);
    assert(disabled.state == State::Disabled && !disabled.diag.voltageClipped && !disabled.diag.currentClipped);
  }

  // A clip flag held on the last Run tick does not freeze the integral after re-entering Run: the first loop run
  // after entry integrates as usual, so the integral reads 0.004 at 62000 us.
  {
    const Config cfg = makeTrimConfig();
    BalanceController bc;
    enterRun(bc, cfg, 0);
    wheelTick(bc, cfg, 12000, 2, -1.0f);
    wheelTick(bc, cfg, 22000, 3, -1.0f);
    wheelTick(bc, cfg, 32000, 4, -1.0f, true, false);
    assert(bc.update(atTime(makeMeasurements(), 33000), liveSwitch(33000, false), cfg, 33000).state ==
           State::Disabled);
    enterRun(bc, cfg, 40000);
    wheelTick(bc, cfg, 52000, 5, -1.0f);
    const Output rerun = wheelTick(bc, cfg, 62000, 6, -1.0f);
    assert(rerun.state == State::Run && nearlyTo(rerun.diag.speedI, 0.004f, 1e-6f));
  }

  // Tilt clip timer: the wheels at -12 rad/s give a speed P of +0.6 rad and the reference clips at +0.3 rad; +12 rad/s
  // clips at -0.3 rad. The clip starts on the loop run at 22000 us, with a 100000 us duration: 122000 us is not over,
  // 122001 us faults. Held ticks in between keep the clip.
  {
    for (const float wheel : {-12.0f, 12.0f}) {
      Config cfg = makeBalanceConfig();
      cfg.speedKpRadPerMps = 0.5f;
      cfg.tiltClipDurationUs = 100000;
      BalanceController bc;
      enterRun(bc, cfg, 0);
      wheelTick(bc, cfg, 12000, 2, wheel);
      assert(wheelTick(bc, cfg, 22000, 3, wheel).diag.tiltClipped);
      assert(wheelTick(bc, cfg, 121999, 3, wheel).state == State::Run);
      assert(wheelTick(bc, cfg, 122000, 3, wheel).state == State::Run);
      const Output over = wheelTick(bc, cfg, 122001, 3, wheel);
      assert(over.state == State::Fault && over.fault == FaultReason::TiltSaturated && !over.driveEnabled);
    }
  }

  // Tilt clip timer restarts when the clip recovers, for both signs. The first clip runs from 22000 to 62000 us
  // (40000 us). It is clipped again from 82000 us, so 122001 us is still Run: the first clip's start would have
  // faulted there. The second clip faults at 182001 us.
  {
    for (const float wheel : {-12.0f, 12.0f}) {
      Config cfg = makeBalanceConfig();
      cfg.speedKpRadPerMps = 0.5f;
      cfg.tiltClipDurationUs = 100000;
      BalanceController bc;
      enterRun(bc, cfg, 0);
      wheelTick(bc, cfg, 12000, 2, wheel);
      assert(wheelTick(bc, cfg, 22000, 3, wheel).diag.tiltClipped);
      const Output recovered = wheelTick(bc, cfg, 62000, 4, 0.0f);
      assert(!recovered.diag.tiltClipped && recovered.state == State::Run);
      wheelTick(bc, cfg, 72000, 5, wheel);
      assert(wheelTick(bc, cfg, 82000, 6, wheel).diag.tiltClipped);
      assert(wheelTick(bc, cfg, 122001, 6, wheel).state == State::Run);
      assert(wheelTick(bc, cfg, 182000, 6, wheel).state == State::Run);
      const Output over = wheelTick(bc, cfg, 182001, 6, wheel);
      assert(over.state == State::Fault && over.fault == FaultReason::TiltSaturated);
    }
  }

  // Tilt clip timer across micros() wraparound. The clip starts at base + 22000 us, which is 28000 us before the
  // wrap; 122000 us after the start is not over and 122001 us is.
  {
    const uint32_t base = 0xFFFFFFFFu - 50000u;
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 0.5f;
    cfg.tiltClipDurationUs = 100000;
    BalanceController bc;
    enterRun(bc, cfg, base);
    wheelTick(bc, cfg, base + 12000u, 2, -12.0f);
    assert(wheelTick(bc, cfg, base + 22000u, 3, -12.0f).diag.tiltClipped);
    // 8000 us after the start and before the wrap: the end time has already wrapped to a small value, but this is Run.
    assert(wheelTick(bc, cfg, base + 30000u, 3, -12.0f).state == State::Run);
    assert(wheelTick(bc, cfg, base + 122000u, 3, -12.0f).state == State::Run);
    const Output over = wheelTick(bc, cfg, base + 122001u, 3, -12.0f);
    assert(over.state == State::Fault && over.fault == FaultReason::TiltSaturated);
  }

  // Tilt clip timer resets on re-entering Run. The clip starts at 22000 us, Run is left at 82000 us with the clip
  // 60000 us old, and after re-entry the clip starts at 122000 us. It faults at 222001 us, not at 122001 us.
  {
    Config cfg = makeBalanceConfig();
    cfg.speedKpRadPerMps = 0.5f;
    cfg.tiltClipDurationUs = 100000;
    BalanceController bc;
    enterRun(bc, cfg, 0);
    wheelTick(bc, cfg, 12000, 2, -12.0f);
    assert(wheelTick(bc, cfg, 22000, 3, -12.0f).diag.tiltClipped);
    assert(bc.update(atTime(makeMeasurements(), 82000), liveSwitch(82000, false), cfg, 82000).state ==
           State::Disabled);
    enterRun(bc, cfg, 100000);
    wheelTick(bc, cfg, 112000, 4, -12.0f);
    assert(wheelTick(bc, cfg, 122000, 5, -12.0f).diag.tiltClipped);
    assert(wheelTick(bc, cfg, 222000, 5, -12.0f).state == State::Run);
    const Output over = wheelTick(bc, cfg, 222001, 5, -12.0f);
    assert(over.state == State::Fault && over.fault == FaultReason::TiltSaturated);
  }

  // Refused transfer changes nothing. Two controllers share a standstill drift, so speed I builds at the 22000 us loop
  // run. At 32000 us one controller gets the commissioning switch with a disturbance that refuses the transfer; the twin
  // has the switch off, so any difference from it is a side effect of the refusal. A transfer at 42000 us, a loop run
  // with the drift back, then gives trim + speed I of the twin's values.
  {
    const Config cfg = makeTrimConfig();
    const ImuValues drift{0.0f, 0.0f, 0.0f, -0.08f, -0.08f};
    const ImuValues disturbances[] = {
        ImuValues{0.02f, 0.0f, 0.0f, -0.08f, -0.08f},   // tilt error 0.02 rad, beyond 0.01 rad
        ImuValues{0.0f, 0.0f, 0.0f, -0.2f, -0.2f},      // wheel speed 0.02 m/s, beyond 0.01 m/s
        ImuValues{0.0f, 0.0f, 0.0f, -0.099f, -0.099f},  // speed P 0.00495 rad against the stored 0.004 rad: a jump
    };
    for (const ImuValues &disturbance : disturbances) {
      BalanceController ctl;
      BalanceController twin;
      enterRun(ctl, cfg, 0);
      enterRun(twin, cfg, 0);
      imuSample(ctl, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
      imuSample(twin, cfg, 12000, 2, ImuValues{}, trimRequest(12000, false));
      imuSample(ctl, cfg, 22000, 3, drift, trimRequest(22000, false));
      imuSample(twin, cfg, 22000, 3, drift, trimRequest(22000, false));
      const Output twinAt32 = imuSample(twin, cfg, 32000, 4, disturbance, trimRequest(32000, false));
      const Output refused = imuSample(ctl, cfg, 32000, 4, disturbance, trimRequest(32000, true));
      assert(refused.state == State::Run && refused.diag.commissioningRefused);
      assert(refused.diag.speedI > 0.0f);
      assert(refused.diag.speedI == twinAt32.diag.speedI);
      assert(refused.diag.trimDeg == twinAt32.diag.trimDeg);
      assert(refused.diag.tiltRefRad == twinAt32.diag.tiltRefRad);

      const Output twinAt42 = imuSample(twin, cfg, 42000, 5, drift, trimRequest(42000, false));
      const Output moved = imuSample(ctl, cfg, 42000, 5, drift, trimRequest(42000, true));
      assert(moved.state == State::Commissioning && !moved.diag.commissioningRefused && moved.driveEnabled);
      assert(moved.diag.speedI == 0.0f);
      assert(nearlyTo(moved.diag.trimDeg, twinAt42.diag.trimDeg + twinAt42.diag.speedI * kRadToDeg, 1e-3f));
    }
  }

  // Commissioning -> Run needs a validated trim on its own. The working trim equals the stored 10 deg and the switch is
  // off, so only trimValidated decides.
  {
    const Config cfg = makeBalanceConfig();
    Config unvalidated = cfg;
    unvalidated.trimValidated = false;
    BalanceController bc;
    enterCommissioning(bc, cfg, 0);
    const Output stays = imuSample(bc, unvalidated, 32000u, 4, ImuValues{}, trimRequest(32000u, false));
    assert(stays.state == State::Commissioning && stays.driveEnabled);
    assert(nearlyTo(stays.diag.trimDeg, 10.0f, 1e-3f));
    const Output back = imuSample(bc, cfg, 42000u, 5, ImuValues{}, trimRequest(42000u, false));
    assert(back.state == State::Run && back.driveEnabled);
  }

  // Right wheel sign -1 with the left at +1. The envelope is in forward terms, so a right wheel at forward +100 rad/s
  // (its motor sees -100 rad/s) gets the same interval as the left wheel at +100 rad/s. At -100 rad/s the right
  // interval is the left one at -100 rad/s. Removing the mirror would swap the interval ends and fail both cases.
  {
    Config mirrored = config;
    mirrored.rightWheelSign = -1;
    BalanceController bc;
    enterRun(bc, mirrored, 100000);
    Measurements m = atTime(good, 110000);
    m.busVoltageV = 8.4f;
    m.leftWheelRadPerSec = 100.0f;
    m.rightWheelRadPerSec = 100.0f;
    const Output forward = bc.update(m, makeRequest(true), mirrored, 110000);
    assert(forward.state == State::Run && forward.fault == FaultReason::None);
    assert(closeTo(forward.diag.leftMinA, -0.614240f) && closeTo(forward.diag.leftMaxA, 0.113032f));
    assert(closeTo(forward.diag.rightMinA, -0.614240f) && closeTo(forward.diag.rightMaxA, 0.113032f));

    m.rightWheelRadPerSec = -100.0f;
    const Output opposite = bc.update(m, makeRequest(true), mirrored, 110000);
    assert(closeTo(opposite.diag.leftMinA, -0.614240f) && closeTo(opposite.diag.leftMaxA, 0.113032f));
    assert(closeTo(opposite.diag.rightMinA, -0.113032f) && closeTo(opposite.diag.rightMaxA, 0.614240f));
  }

  // A right wheel sign of 0 is not a direction: the arming tick faults InvalidEnvelope with drivers off.
  {
    Config broken = config;
    broken.rightWheelSign = 0;
    BalanceController bc;
    tick(bc, good, liveSwitch(1000, false), broken, 1000);
    tick(bc, good, liveSwitch(2000, true), broken, 2000);
    const Output armed = tick(bc, good, liveSwitch(3000, true), broken, 3000);
    assert(armed.state == State::Fault && armed.fault == FaultReason::InvalidEnvelope && !armed.driveEnabled);
  }

  // Speed integral freezes on low reserve without an effort clip. A balance of 7.9 A (tilt rate 15.8 rad/s, gain 0.5)
  // leaves about 0.55 A of reserve below reserveLowA 1.0 A, with c not clipped, so the loop at 42000 us holds speed I
  // at 0.004 rad. A balance of 6.4 A (tilt rate 12.8 rad/s) leaves about 2 A of reserve and speed I moves to 0.008 rad.
  {
    const auto tiltRateTick = [](BalanceController &bc, const Config &cfg, uint32_t nowUs, uint32_t seq,
                                 float tiltRate) {
      Measurements m = atTime(makeMeasurements(), nowUs);
      m.imuSequence = seq;
      m.tiltRateRadPerSec = tiltRate;
      m.leftWheelRadPerSec = -1.0f;
      m.rightWheelRadPerSec = -1.0f;
      return bc.update(m, liveRequest(nowUs, 0.0f, 0.0f), cfg, nowUs);
    };
    const Config cfg = makeTrimConfig();
    BalanceController low;
    enterRun(low, cfg, 0);
    wheelTick(low, cfg, 12000, 2, -1.0f);
    assert(nearlyTo(wheelTick(low, cfg, 22000, 3, -1.0f).diag.speedI, 0.004f, 1e-6f));
    const Output lowReserve = tiltRateTick(low, cfg, 32000, 4, 15.8f);
    assert(!lowReserve.diag.effortClipped && lowReserve.diag.reserve < cfg.reserveLowA);
    assert(nearlyTo(wheelTick(low, cfg, 42000, 5, -1.0f).diag.speedI, 0.004f, 1e-6f));

    BalanceController high;
    enterRun(high, cfg, 0);
    wheelTick(high, cfg, 12000, 2, -1.0f);
    assert(nearlyTo(wheelTick(high, cfg, 22000, 3, -1.0f).diag.speedI, 0.004f, 1e-6f));
    const Output highReserve = tiltRateTick(high, cfg, 32000, 4, 12.8f);
    assert(!highReserve.diag.effortClipped && highReserve.diag.reserve >= cfg.reserveLowA);
    assert(nearlyTo(wheelTick(high, cfg, 42000, 5, -1.0f).diag.speedI, 0.008f, 1e-6f));
  }

  // Knob pickup boundary. The band is the float difference of the knob (11 deg) and trim (10 deg) angles, so a knob
  // exactly one band from the trim is on the boundary and must not engage. A knob 0.001 deg inside does.
  {
    const float trimRad = 10.0f * kDegToRad;
    const float knobRad = 11.0f * kDegToRad;
    Config cfg = makeBalanceConfig();
    cfg.knobPickupBandRad = knobRad - trimRad;
    BalanceController onBoundary;
    enterCommissioning(onBoundary, cfg, 0);
    const Output edge = knobSample(onBoundary, cfg, 32000u, 4, true, 11.0f);
    assert(edge.state == State::Commissioning && !edge.diag.knobEngaged && nearlyTo(edge.diag.trimDeg, 10.0f, 1e-4f));

    BalanceController inside;
    enterCommissioning(inside, cfg, 0);
    const Output near = knobSample(inside, cfg, 32000u, 4, true, 10.999f);
    assert(near.diag.knobEngaged);
  }

  // Each of the three timing and tolerance values must be finite and positive. Zero or NaN faults InvalidConfig on the
  // arming tick, with drivers off.
  {
    const auto armedWith = [&good](const Config &c) {
      BalanceController bc;
      tick(bc, good, liveSwitch(1000u, false), c, 1000u);
      tick(bc, good, liveSwitch(2000u, true), c, 2000u);
      return tick(bc, good, liveSwitch(3000u, true), c, 3000u);
    };
    for (float Config::*field : {&Config::transferToleranceRad, &Config::standstillWheelSpeedTolMps,
                                 &Config::trimSlewRadPerSec}) {
      for (const float bad : {0.0f, std::numeric_limits<float>::quiet_NaN()}) {
        Config c = makeBalanceConfig();
        c.*field = bad;
        const Output out = armedWith(c);
        assert(out.state == State::Fault && out.fault == FaultReason::InvalidConfig && !out.driveEnabled);
      }
    }
  }

  std::puts("balance controller passed");
  return 0;
}
