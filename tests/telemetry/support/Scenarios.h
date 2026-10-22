#pragma once

#include <stdint.h>
#include <stdio.h>
#include <string>

#include "DebugTelemetry.h"
#include "RecordingStubs.h"

namespace Scenarios {

// Hands out distinct, exactly representable values so that any swapped field changes the output.
class Filler {
 public:
  explicit Filler(int seed) : next_(1000.0f * (seed + 1)), counter_(seed * 7) {}
  float f() { next_ += 0.5f; return next_; }
  int16_t i16() { return static_cast<int16_t>(counter_ += 13); }
  uint16_t u16() { return static_cast<uint16_t>(counter_ += 17); }
  int32_t i32() { return counter_ += 19; }

 private:
  float next_;
  int counter_;
};

inline void fill(Filler& g, imu::Vector3& v) { v.x = g.f(); v.y = g.f(); v.z = g.f(); }

inline void fill(Filler& g, DebugTelemetry::PidStatus& p) {
  p.Kp = g.f(); p.Ki = g.f(); p.Kd = g.f(); p.deriv = g.f(); p.integral = g.f(); p.iLimit = g.f();
  p.outP = g.f(); p.outI = g.f(); p.outD = g.f(); p.output = g.f(); p.error = g.f();
}

// Variant 0: balance running. 1: gains on but tumbling. 2: gains off, touch state outside {0,1}.
inline DebugTelemetry::TelemetryInputs makeInputs(int variant, int selection, int tick) {
  static const uint32_t kGate[4] = {0, 3, 7, 14};
  Filler g(variant);
  DebugTelemetry::TelemetryInputs in{};
  in.selection = selection;
  in.nowMs = 5000 + 50 * tick + variant;
  in.controlGateSequence = kGate[tick];
  in.balanceActive = variant == 0;

  imu::Snapshot& s = in.imu;
  s.valid = true;
  s.sequence = 100 + tick;
  s.readTimeUs = 3000000u + 1000u * tick;
  s.dtSec = g.f();
  in.imuTiming = {4000000u + static_cast<uint32_t>(tick), 20u + static_cast<uint32_t>(variant),
                  2500u + static_cast<uint32_t>(tick)};
  fill(g, s.accelerationG);
  fill(g, s.angularVelocityRadPerSec);
  fill(g, s.filteredAccelerationG);
  fill(g, s.filteredAngularVelocityRadPerSec);
  s.temperatureC = g.f();
  s.attitudeDegrees.roll = g.f(); s.attitudeDegrees.pitch = g.f(); s.attitudeDegrees.yaw = g.f();
  s.correctedRollDegrees = g.f();
  s.correctedPitchDegrees = g.f();
  fill(g, s.complementaryAnglesDegrees);
  fill(g, s.integratedGyroAnglesDegrees);
  in.imuSettings.mahonyTwoKp = g.f();
  in.imuSettings.mahonyTwoKi = g.f();
  in.imuCalibration.gyro = {g.f(), g.f(), g.f()};
  in.imuCalibration.attitude = {g.f(), g.f()};
  in.controlTimestepSec = g.f();

  in.m1VelocityRadPerSec = g.f();
  in.m2VelocityRadPerSec = g.f();
  in.m1FilteredVelocityRadPerSec = g.f();
  in.m2FilteredVelocityRadPerSec = g.f();
  in.motor1Target = g.f();
  in.motor2Target = g.f();
  in.motor2CurrentSetpoint = g.f();
  in.motor1CurrentSetpoint = g.f();
  in.motor1VoltageQ = g.f();
  in.motor2VoltageQ = g.f();
  in.motor1ShaftVelocity = g.f();
  in.motor2ShaftVelocity = g.f();
  in.sensor1Angle = g.f();
  in.sensor1MechanicalAngle = g.f();
  in.sensor2Angle = g.f();
  in.sensor2MechanicalAngle = g.f();

  for (int i = 0; i < 18; ++i) in.sbusChannels[i] = static_cast<int16_t>(172 + 41 * i + variant);
  in.sbusFrameIntervalMs = 14 + variant;

  fill(g, in.anglePid);
  fill(g, in.speedPid);
  fill(g, in.yawPid);
  fill(g, in.rollPid);
  fill(g, in.touchXPid);
  fill(g, in.touchYPid);

  DebugTelemetry::TouchStatus& t = in.touch;
  t.XPressDat = g.u16(); t.YPressDat = g.u16();
  t.XPdat = g.i16(); t.YPdat = g.i16(); t.XLdat = g.i16(); t.YLdat = g.i16();
  t.XPdatF = g.f(); t.YPdatF = g.f();
  t.state = static_cast<uint8_t>(variant == 0 ? 1 : (variant == 1 ? 0 : 2));
  t.start = g.i32();

  in.servoTrims = {g.f(), g.f(), g.f(), g.f()};
  for (int i = 0; i < 4; ++i) in.servoTraceAngle[i] = g.i32();

  in.topBallX = g.f();
  in.topBallY = g.f();
  in.sbusTopBallXSmoothed = g.f();
  in.sbusTopBallYSmoothed = g.f();

  in.bodyRoll = g.f();
  in.bodyPitching = g.f();
  in.bodyPitchingFiltered = g.f();
  in.bodyRollFiltered = g.f();
  in.bodyPitchCorrection = g.f();
  in.bodyHeightCommandM = g.f();
  in.bodyHeightCommandFilteredM = g.f();
  in.bodyTurn = g.f();
  in.bodyX = g.f();
  in.bodyCommandFilteringEnabled = g.f();
  in.bodyCommandFilterCutoffHz = g.f();

  in.movementSpeed = g.f();
  in.driveEffectiveSpeed = g.f();
  in.driveSpeedBodyXRaw = g.f();
  in.wheelVelocityFeedbackCorrection = g.f();

  in.voltageAdc = static_cast<uint16_t>(700 + 11 * tick + variant);
  in.voltageAdcFiltered = g.f();
  in.voltage = g.f();
  in.batteryInstantRawV = g.f();
  in.batteryInstantFilteredV = g.f();
  in.controlStartUs = 5000000u + 1000u * tick;
  in.controlExecutionUs = 800u + 20u * tick;
  in.maxLoopEntryIntervalUs = 1200u + 50u * tick;

  in.pidGainsMode = variant == 2 ? 0 : (variant == 0 ? 2 : 1);
  in.postureOrMarkMode = 1 + variant;
  in.robotTumble = variant == 1 ? 2 : 0;
  in.pidParameterTuning = 1.0f + variant;
  in.targetBodyHeightOverrideM = g.f();
  return in;
}

// Every selector from 0 to 80, three input variants, due and not due, four ticks with different
// trace gates. Persistent state restarts for each run. The runner executes one control tick.
template <class Runner>
std::string recordAll(Runner run) {
  std::string out;
  char line[160];
  for (int selection = 0; selection <= 80; ++selection) {
    for (int variant = 0; variant < 3; ++variant) {
      for (int due = 1; due >= 0; --due) {
        Recording::reset();
        Recording::dueResult = due == 1;
        DebugTelemetry::TelemetryState state{static_cast<uint16_t>(650 + variant), 0, 0};
        for (int tick = 0; tick < 4; ++tick) {
          const DebugTelemetry::TelemetryInputs in = makeInputs(variant, selection, tick);
          const size_t before = Recording::events().size();
          run(in, state);
          snprintf(line, sizeof line, "== sel=%d var=%d due=%d tick=%d gate=%u now=%u\n", selection,
                   variant, due, tick, static_cast<unsigned>(in.controlGateSequence),
                   static_cast<unsigned>(in.nowMs));
          out += line;
          for (size_t i = before; i < Recording::events().size(); ++i) {
            const Recording::Event& e = Recording::events()[i];
            out += e.kind + " " + e.text + "\n";
          }
          snprintf(line, sizeof line, "state adcMin=%u traceSeq=%u\n",
                   static_cast<unsigned>(state.voltageAdcMin),
                   static_cast<unsigned>(state.traceSequence));
          out += line;
        }
      }
    }
  }
  return out;
}

}  // namespace Scenarios
