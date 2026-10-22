#include <cassert>
#include <cstdio>

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
  config.trimValidated = true;
  config.storedTrimDeg = 10.0f;
  config.fixedPoseReachable = true;
  return config;
}

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
  return controller.update(m, r, c, nowUs);
}
}  // namespace

int main() {
  const Config config = makeConfig();
  const Measurements good = makeMeasurements();

  // Disabled stays Disabled while the switch is off.
  {
    BalanceController bc;
    for (uint32_t t = 1000; t <= 4000; t += 1000) {
      const Output out = tick(bc, good, makeRequest(false), config, t);
      assert(out.state == State::Disabled && !out.driveEnabled);
    }
  }

  // A switch that is already on at the first update does not arm.
  {
    BalanceController bc;
    assert(tick(bc, good, makeRequest(true), config, 1000).state == State::Disabled);
    assert(tick(bc, good, makeRequest(true), config, 2000).state == State::Disabled);
    const Output out = tick(bc, good, makeRequest(true), config, 3000);
    assert(out.state == State::Disabled && !out.driveEnabled);
  }

  // An off-to-on edge enters Arming, and drivers stay off there.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    const Output out = tick(bc, good, makeRequest(true), config, 2000);
    assert(out.state == State::Arming && !out.driveEnabled);
  }

  // Without a validated trim, the edge is lost and a later level does not arm.
  {
    Config noTrim = config;
    noTrim.trimValidated = false;
    BalanceController bc;
    tick(bc, good, makeRequest(false), noTrim, 1000);
    assert(tick(bc, good, makeRequest(true), noTrim, 2000).state == State::Disabled);
    assert(tick(bc, good, makeRequest(true), config, 3000).state == State::Disabled);  // no edge
    tick(bc, good, makeRequest(false), config, 4000);
    assert(tick(bc, good, makeRequest(true), config, 5000).state == State::Arming);
  }

  // Without a reachable fixed pose, the edge is lost and a later level does not arm.
  {
    Config noPose = config;
    noPose.fixedPoseReachable = false;
    BalanceController bc;
    tick(bc, good, makeRequest(false), noPose, 1000);
    assert(tick(bc, good, makeRequest(true), noPose, 2000).state == State::Disabled);
    assert(tick(bc, good, makeRequest(true), config, 3000).state == State::Disabled);  // no edge
    tick(bc, good, makeRequest(false), config, 4000);
    assert(tick(bc, good, makeRequest(true), config, 5000).state == State::Arming);
  }

  // Arming moves to Run once every condition holds, with drivers enabled and zero phase current.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    assert(tick(bc, good, makeRequest(true), config, 2000).state == State::Arming);
    const Output out = tick(bc, good, makeRequest(true), config, 3000);
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
      tick(bc, good, makeRequest(false), config, 1000);
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
    tick(bc, level, makeRequest(false), config, 1000);
    assert(tick(bc, level, makeRequest(true), config, 2000).state == State::Arming);
    assert(tick(bc, level, makeRequest(true), config, 3000).state == State::Arming);
    assert(tick(bc, good, makeRequest(true), config, 4000).state == State::Run);
  }

  // Arming times out after the configured duration, measured with elapsed time across micros() wrap.
  {
    const uint32_t start = 0xFFFFFFF0u;
    Measurements unstable = good;
    unstable.tiltRateRadPerSec = 1.0f;
    BalanceController bc;
    tick(bc, unstable, makeRequest(false), config, start - 1000);
    assert(tick(bc, unstable, makeRequest(true), config, start).state == State::Arming);
    assert(tick(bc, unstable, makeRequest(true), config, start + kTimeoutUs).state == State::Arming);
    assert(tick(bc, unstable, makeRequest(true), config, start + kTimeoutUs + 1).state == State::Disabled);
  }

  // After a timeout, the switch held on cannot re-arm; a fresh edge is needed.
  {
    Measurements unstable = good;
    unstable.tiltRateRadPerSec = 1.0f;
    BalanceController bc;
    tick(bc, unstable, makeRequest(false), config, 1000);
    tick(bc, unstable, makeRequest(true), config, 2000);
    assert(tick(bc, unstable, makeRequest(true), config, 2000 + kTimeoutUs + 1).state == State::Disabled);
    assert(tick(bc, good, makeRequest(true), config, 2000 + kTimeoutUs + 2).state == State::Disabled);
    tick(bc, good, makeRequest(false), config, 2000 + kTimeoutUs + 3);
    assert(tick(bc, good, makeRequest(true), config, 2000 + kTimeoutUs + 4).state == State::Arming);
  }

  // Switch off during Arming goes to Disabled.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    assert(tick(bc, good, makeRequest(true), config, 2000).state == State::Arming);
    const Output out = tick(bc, good, makeRequest(false), config, 2500);
    assert(out.state == State::Disabled && !out.driveEnabled);
  }

  // Switch off during Run goes to Disabled with drivers disabled.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    tick(bc, good, makeRequest(true), config, 2000);
    assert(tick(bc, good, makeRequest(true), config, 3000).state == State::Run);
    const Output out = tick(bc, good, makeRequest(false), config, 4000);
    assert(out.state == State::Disabled && !out.driveEnabled);
    assert(out.leftA == 0.0f && out.rightA == 0.0f);
  }

  // Run leaves only through the switch: a tilt outside the arming window does not stop it.
  {
    Measurements tilted = good;
    tilted.tiltRad = 0.5f;
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    tick(bc, good, makeRequest(true), config, 2000);
    assert(tick(bc, good, makeRequest(true), config, 3000).state == State::Run);
    const Output out = tick(bc, tilted, makeRequest(true), config, 4000);
    assert(out.state == State::Run && out.driveEnabled);
  }

  // After Run is left by switch off, arming again needs a new edge.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 1000);
    tick(bc, good, makeRequest(true), config, 2000);
    tick(bc, good, makeRequest(true), config, 3000);
    tick(bc, good, makeRequest(false), config, 4000);
    assert(tick(bc, good, makeRequest(true), config, 5000).state == State::Arming);
  }

  // Not tested in step 1: entry into Fault. Faults are first produced in step 2.
  std::puts("balance controller passed");
  return 0;
}
