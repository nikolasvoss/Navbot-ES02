#include <cassert>
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
void enterRun(BalanceController &controller, const Config &config, uint32_t startUs) {
  tick(controller, makeMeasurements(), makeRequest(false), config, startUs);
  tick(controller, makeMeasurements(), makeRequest(true), config, startUs + 1000);
  assert(tick(controller, makeMeasurements(), makeRequest(true), config, startUs + 2000).state == State::Run);
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
    const Output out = tick(bc, noImu, makeRequest(true), config, 112000);
    assert(out.state == State::Fault && out.fault == FaultReason::InvalidInput);

    Measurements noWheels = good;
    noWheels.wheelsValid = false;
    BalanceController bc2;
    enterRun(bc2, config, 100000);
    const Output out2 = tick(bc2, noWheels, makeRequest(true), config, 112000);
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

  // Arming needs fresh IMU, wheels and bus; stale inputs keep Arming until fresh ones arrive.
  {
    BalanceController bc;
    tick(bc, good, makeRequest(false), config, 200000);
    assert(tick(bc, good, makeRequest(true), config, 201000).state == State::Arming);

    Measurements staleBus = atTime(good, 202000);
    staleBus.busReadTimeUs = 202000 - config.busMaxAgeUs - 1;
    assert(bc.update(staleBus, makeRequest(true), config, 202000).state == State::Arming);

    Measurements staleImu = atTime(good, 203000);
    staleImu.imuReadTimeUs = 203000 - config.imuMaxAgeUs - 1;
    assert(bc.update(staleImu, makeRequest(true), config, 203000).state == State::Arming);

    assert(tick(bc, good, makeRequest(true), config, 204000).state == State::Run);
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
    tick(bc, good, makeRequest(false), config, 104000);
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
      assert(tick(bc, fast, makeRequest(true), config, base + 10000u * j).state == State::Run);
    }
    const Output out = tick(bc, fast, makeRequest(true), config, base + 210000u);
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
    for (uint32_t j = 0; j <= 15; ++j) assert(tick(bc, fast, makeRequest(true), config, base + 10000u * j).state == State::Run);
    assert(tick(bc, still, makeRequest(true), config, base + 10000u * 16).state == State::Run);
    for (uint32_t j = 17; j <= 37; ++j) assert(tick(bc, fast, makeRequest(true), config, base + 10000u * j).state == State::Run);
    const Output out = tick(bc, fast, makeRequest(true), config, base + 10000u * 38);
    assert(out.state == State::Fault && out.fault == FaultReason::Overspeed);
  }

  // A NaN wheel speed counts as overspeed, even when the other wheel is normal.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements bad = good;
    bad.leftWheelRadPerSec = std::numeric_limits<float>::quiet_NaN();
    for (uint32_t j = 0; j <= 20; ++j) {
      assert(tick(bc, bad, makeRequest(true), config, 110000u + 10000u * j).state == State::Run);
    }
    const Output out = tick(bc, bad, makeRequest(true), config, 110000u + 210000u);
    assert(out.state == State::Fault && out.fault == FaultReason::Overspeed);
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
      assert(tick(bc, atLimit, makeRequest(true), config, 110000u + 10000u * j).state == State::Run);
    }
  }

  // A latched fault does not clear into Run while the switch stays on; the switch off returns to Disabled.
  {
    BalanceController bc;
    enterRun(bc, config, 100000);
    Measurements noImu = good;
    noImu.imuValid = false;
    const Output faulted = tick(bc, noImu, makeRequest(true), config, 103000);
    assert(faulted.state == State::Fault && faulted.fault == FaultReason::InvalidInput);
    assert(!faulted.driveEnabled && faulted.leftA == 0.0f && faulted.rightA == 0.0f);
    assert(tick(bc, good, makeRequest(true), config, 104000).state == State::Fault);
    assert(tick(bc, good, makeRequest(true), config, 300000).state == State::Fault);

    const Output off = tick(bc, good, makeRequest(false), config, 301000);
    assert(off.state == State::Disabled && off.fault == FaultReason::None && !off.driveEnabled);

    // A new edge rearms through Arming, never straight into Run.
    assert(tick(bc, good, makeRequest(true), config, 302000).state == State::Arming);
    assert(tick(bc, good, makeRequest(true), config, 303000).state == State::Run);
  }

  std::puts("balance controller passed");
  return 0;
}
