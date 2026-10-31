// The balance path of the control tick through its public functions, with literal expected values. The balance controller,
// the overspeed guard, the wheel boundary, the battery filter and the default settings are the firmware's own code.
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "ControlTick.h"
#include "Logging.h"
#include "RobotConfig.h"
#include "rc_mode.h"

// The sketch sends the log lines to the link. Here only the overspeed trip lines are counted.
namespace {
int overspeedLogLines = 0;
}  // namespace

namespace Logging {
void message(Level, const char *tag, const char *, ...) {
  if (std::strcmp(tag, "WHEEL_OVERSPEED") == 0) ++overspeedLogLines;
}
}  // namespace Logging

namespace {

constexpr float kTol = 1e-4f;
constexpr float kSupplyVoltageV = 8.4f;  // kWheelSupplyVoltageV

bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

using Frame = int16_t[ControlTick::kRcChannelCount];

// A frame with every channel at centre, then the channels the tests use. CH6 and CH7 are off.
void fillFrame(Frame &frame, int16_t ch3, int16_t ch4, int16_t ch5) {
  for (int16_t &channel : frame) channel = 992;
  frame[2] = ch3;
  frame[3] = ch4;
  frame[4] = ch5;
  frame[5] = 192;
  frame[6] = 192;
}

// The inputs of one control tick. The wheel speed is the same on both wheels.
struct Inputs {
  bool sendFrame = true;
  int16_t ch3 = 992;
  int16_t ch4 = 992;
  int16_t ch5 = 192;
  float rollDeg = 0.0f;
  float gyroXRadPerSec = 0.0f;
  float gyroZRadPerSec = 0.0f;
  float wheelRadPerSec = 0.0f;
  bool motorTest = false;
};

ControlTick::MotorFeedback idleMotor() { return {0.0f, 4.2f, 0.0f, 0.7304f}; }

// One control tick, in the sketch's order: the frame, the overspeed guard, then the balance. The battery and the wheels are
// set directly, as the battery filter and the wheel estimator are not under test here. The IMU sequence rises each tick.
ControlTick::WheelCommand step(ControlTick::State &state, uint32_t nowUs, const Inputs &in) {
  static uint32_t imuSequence = 0;
  Frame frame;
  fillFrame(frame, in.ch3, in.ch4, in.ch5);
  if (in.sendFrame) ControlTick::applyRcFrame(state, frame, nowUs);
  state.batteryVoltage = kSupplyVoltageV;
  state.busReadTimeUs = nowUs;
  state.wheelsValid = true;
  state.wheelReadTimeUs = nowUs;
  state.m1VelocityRadPerSec = in.wheelRadPerSec;
  state.m2VelocityRadPerSec = in.wheelRadPerSec;

  imu::Snapshot snapshot;
  snapshot.valid = true;
  snapshot.sequence = ++imuSequence;
  snapshot.readTimeUs = nowUs;
  snapshot.correctedRollDegrees = in.rollDeg;
  snapshot.angularVelocityRadPerSec = {in.gyroXRadPerSec, 0.0f, in.gyroZRadPerSec};

  ControlTick::serviceOverspeed(state, nowUs / 1000U);
  const Config config = defaultBalanceConfig();
  return ControlTick::runBalance(state, config, snapshot, idleMotor(), idleMotor(), true, in.motorTest, nowUs);
}

// Brings the controller to Run: CH5 off for one tick, then position 2 for two ticks, with the wheels still.
uint32_t armToRun(ControlTick::State &state, uint32_t nowUs) {
  Inputs off;
  off.ch5 = 192;
  step(state, nowUs, off);
  Inputs on;
  on.ch5 = 1792;
  step(state, nowUs + 1000, on);
  step(state, nowUs + 2000, on);
  assert(state.balanceOutput.state == State::Run);
  return nowUs + 2000;
}

void testCh5Positions() {
  const int16_t raws[4] = {191, 592, 1401, 1402};
  const uint8_t positions[4] = {0, 1, 1, 2};
  for (int i = 0; i < 4; ++i) {
    ControlTick::State state;
    Frame frame;
    fillFrame(frame, 992, 992, raws[i]);
    ControlTick::applyRcFrame(state, frame, 0);
    assert(state.ch5Position == positions[i]);
  }
}

void testStickFractions() {
  // CH3 sets the speed request (speed cap 0.3 m/s) and CH4 the yaw request (yaw cap 1.0 rad/s), each -1 at 192, 0 at 992
  // and +1 at 1792. CH4 is positive to the left, so 192 turns left.
  ControlTick::State state;
  Inputs in;
  in.ch3 = 192;
  in.ch4 = 192;
  step(state, 0, in);
  assert(near(state.movementSpeed, -0.3f, kTol));
  assert(near(state.bodyTurn, 1.0f, kTol));
  in.ch3 = 992;
  in.ch4 = 992;
  step(state, 1000, in);
  assert(near(state.movementSpeed, 0.0f, kTol));
  assert(near(state.bodyTurn, 0.0f, kTol));
  in.ch3 = 1792;
  in.ch4 = 1792;
  step(state, 2000, in);
  assert(near(state.movementSpeed, 0.3f, kTol));
  assert(near(state.bodyTurn, -1.0f, kTol));
}

void testStaleFrameIsZeroDemand() {
  ControlTick::State state;
  Inputs in;
  in.ch3 = 1792;
  in.ch5 = 1792;
  step(state, 0, in);
  // 100 ms after the frame it still counts; 1 us later it does not.
  Inputs same = in;
  same.sendFrame = false;
  step(state, 100000, same);
  assert(state.rcValid);
  assert(near(state.movementSpeed, 0.3f, kTol));
  step(state, 100001, same);
  assert(!state.rcValid);
  assert(state.movementSpeed == 0.0f);
}

void testLinkCommand() {
  ControlTick::State state;
  assert(ControlTick::linkCommandAgeMs(state, 5000) == UINT32_MAX);

  // Position 0: refused.
  Inputs in;
  in.ch5 = 191;
  step(state, 0, in);
  const char *refusal = ControlTick::acceptLinkCommand(state, 0.1f, 0.0f, 0);
  assert(std::strcmp(refusal, "CH5 not in link position (1)") == 0);

  // Position 1 for the first time: the position change is still pending until a tick has seen it.
  Frame frame;
  fillFrame(frame, 992, 992, 592);
  ControlTick::applyRcFrame(state, frame, 1000000);
  refusal = ControlTick::acceptLinkCommand(state, 0.1f, 0.0f, 1000000);
  assert(std::strcmp(refusal, "CH5 position change pending") == 0);
  in.ch5 = 592;
  step(state, 1000000, in);
  refusal = ControlTick::acceptLinkCommand(state, std::nanf(""), 0.0f, 1000000);
  assert(std::strcmp(refusal, "speed and turn must be finite numbers") == 0);
  assert(!state.linkCommand.seen);

  // Accepted in position 1. The age counts from the Command.
  refusal = ControlTick::acceptLinkCommand(state, 0.1f, 0.2f, 1000000);
  assert(refusal == nullptr);
  assert(state.linkCommand.seen);
  assert(ControlTick::linkCommandAgeMs(state, 1005000) == 5);

  // The Command drives the speed request while it is at most RcMode::kLinkCommandTimeoutUs old.
  step(state, 1000000 + RcMode::kLinkCommandTimeoutUs, in);
  assert(near(state.movementSpeed, 0.1f, kTol));
  assert(state.linkCommand.seen);

  // One microsecond later it has expired: the demand is zero and the Command is cleared.
  step(state, 1000000 + RcMode::kLinkCommandTimeoutUs + 1, in);
  assert(state.movementSpeed == 0.0f);
  assert(!state.linkCommand.seen);
}

void testSetLinkBalanceRefusals() {
  ControlTick::State state;
  const char *refusal = ControlTick::setLinkBalance(state, true, false, false);
  assert(std::strcmp(refusal, "CH5 not in link position (1)") == 0);

  Frame frame;
  fillFrame(frame, 992, 992, 592);
  ControlTick::applyRcFrame(state, frame, 0);
  refusal = ControlTick::setLinkBalance(state, true, false, false);
  assert(std::strcmp(refusal, "CH5 position change pending") == 0);
  Inputs link;
  link.ch5 = 592;
  step(state, 0, link);

  refusal = ControlTick::setLinkBalance(state, true, true, false);
  assert(std::strcmp(refusal, "motor test running") == 0);
  refusal = ControlTick::setLinkBalance(state, true, false, true);
  assert(std::strcmp(refusal, "calibration running") == 0);
  assert(!state.linkBalance);
  refusal = ControlTick::setLinkBalance(state, true, false, false);
  assert(refusal == nullptr);
  assert(state.linkBalance);
  // Off is accepted while a motor test or a calibration runs.
  refusal = ControlTick::setLinkBalance(state, false, true, true);
  assert(refusal == nullptr);
  assert(!state.linkBalance);

  // Overspeed: 40 rad/s on one wheel for 50 ms trips the guard, and balance on is then refused.
  state.m1VelocityRadPerSec = 40.0f;
  bool tripped = ControlTick::serviceOverspeed(state, 0);
  assert(!tripped);
  tripped = ControlTick::serviceOverspeed(state, 49);
  assert(!tripped);
  tripped = ControlTick::serviceOverspeed(state, 50);
  assert(tripped);
  refusal = ControlTick::setLinkBalance(state, true, false, false);
  assert(std::strcmp(refusal, "wheel overspeed latched, switch CH5 to 0") == 0);
}

void testOverspeedTripAndRearm() {
  ControlTick::State state;
  const int logsBefore = overspeedLogLines;
  const uint32_t now = armToRun(state, 0);

  // Driving: the wheels get a command.
  Inputs in;
  in.ch5 = 1792;
  ControlTick::WheelCommand command = step(state, now + 1000, in);
  assert(command.action == ControlTick::WheelAction::Drive);

  // 40 rad/s on the wheels: the guard trips after 50 ms above 30 rad/s, and the command is Disable from that tick on.
  Inputs fast = in;
  fast.wheelRadPerSec = 40.0f;
  const uint32_t fastStart = now + 2000;
  for (uint32_t k = 0; k <= 50; ++k) {
    command = step(state, fastStart + k * 1000, fast);
    assert(state.wheelOverspeed.tripped == (k == 50));
  }
  assert(command.action == ControlTick::WheelAction::Disable);
  assert(state.balanceOutput.state == State::Disabled);
  assert(overspeedLogLines == logsBefore + 1);

  // The trip holds with CH5 on: enable is off, so the controller stays Disabled.
  command = step(state, fastStart + 51000, fast);
  assert(command.action == ControlTick::WheelAction::Disable);
  assert(state.balanceOutput.state == State::Disabled);
  assert(overspeedLogLines == logsBefore + 1);

  // CH5 to 0 with the wheels within the limit rearms the guard. CH5 to 2 again starts the arming.
  Inputs off;
  off.ch5 = 192;
  step(state, fastStart + 52000, off);
  assert(!state.wheelOverspeed.tripped);
  step(state, fastStart + 53000, in);
  assert(state.balanceOutput.state == State::Arming);
}

void testMotorTestKeepsOutputs() {
  ControlTick::State state;
  const uint32_t now = armToRun(state, 0);
  Inputs in;
  in.ch5 = 1792;
  in.motorTest = true;
  const ControlTick::WheelCommand command = step(state, now + 1000, in);
  assert(command.action == ControlTick::WheelAction::Keep);
}

void testMeasurementSigns() {
  // Forward lean: correctedRollDegrees -2 is tilt +2 deg. The controller asks tilt P (8 A/rad) times the tilt, here
  // 8 * 0.0349066 rad, with the tilt reference at zero after arming.
  {
    ControlTick::State state;
    const uint32_t now = armToRun(state, 0);
    Inputs in;
    in.ch5 = 1792;
    in.rollDeg = -2.0f;
    step(state, now + 1000, in);
    assert(near(state.balanceOutput.diag.c, 0.2792527f, 1e-3f));
  }
  // Gyro x -1 is a forward tilt rate of +1 rad/s: gyro damping 0.4 A per rad/s.
  {
    ControlTick::State state;
    const uint32_t now = armToRun(state, 0);
    Inputs in;
    in.ch5 = 1792;
    in.gyroXRadPerSec = -1.0f;
    step(state, now + 1000, in);
    assert(near(state.balanceOutput.diag.c, 0.4f, 1e-3f));
  }
  // Gyro z +1 is a left yaw rate of +1 rad/s, against a yaw demand of zero: yaw P 0.6 A per rad/s, so -0.6 A.
  {
    ControlTick::State state;
    const uint32_t now = armToRun(state, 0);
    Inputs in;
    in.ch5 = 1792;
    in.gyroZRadPerSec = 1.0f;
    step(state, now + 1000, in);
    assert(near(state.balanceOutput.diag.d, -0.6f, 1e-3f));
  }
}

void testLegRamp() {
  ControlTick::State state;
  state.legPoseDegrees = 10.0f;
  for (int i = 0; i < 300; ++i) ControlTick::stepLegs(state, 0.001f);
  assert(state.legPoseDegrees < 20.0f);  // 300 ms at 30 deg/s is 9 degrees
  for (int i = 300; i < 340; ++i) ControlTick::stepLegs(state, 0.001f);
  assert(state.legPoseDegrees == 20.0f);
  for (int angle : state.servoTraceAngle) assert(angle == 20);

  // A 1 s step is capped at kLegRampMaxDtSec (0.02 s): 30 deg/s times 0.02 s is 0.6 degrees.
  ControlTick::State capped;
  capped.legPoseDegrees = 10.0f;
  ControlTick::stepLegs(capped, 1.0f);
  assert(near(capped.legPoseDegrees, 10.6f, kTol));
  assert(capped.servoTraceAngle[0] == 11);
}

void testBatteryCounts() {
  ControlTick::State state;
  ControlTick::readBattery(state, 813, 5);
  assert(state.voltageAdc == 813);
  assert(state.busReadTimeUs == 5);
  assert(near(state.batteryInstantRawV, 7.7659f, 1e-3f));  // 813 counts at 7.77 V over 813.43 counts
}

void testWheelSpeedEstimate() {
  ControlTick::State state;
  bool updated = ControlTick::updateWheels(state, 0.0, 0.0, 2000);  // the first call has nothing to difference
  assert(!updated);
  assert(!state.wheelsValid);
  // Both motor angles rise by 1 rad in one second. Forward is negative motor-local on both wheels (WheelBoundary.h).
  updated = ControlTick::updateWheels(state, 1.0, 1.0, 1002000);
  assert(updated);
  assert(state.wheelsValid);
  assert(state.wheelReadTimeUs == 1002000);
  assert(near(state.m1VelocityRadPerSec, -1.0f, kTol));
  assert(near(state.m2VelocityRadPerSec, -1.0f, kTol));
}

void testBalanceRequested() {
  ControlTick::State state;
  assert(!ControlTick::balanceRequested(state));
  Frame frame;
  fillFrame(frame, 992, 992, 592);
  ControlTick::applyRcFrame(state, frame, 0);
  assert(ControlTick::balanceRequested(state));
}

}  // namespace

int main() {
  testCh5Positions();
  testStickFractions();
  testStaleFrameIsZeroDemand();
  testLinkCommand();
  testSetLinkBalanceRefusals();
  testOverspeedTripAndRearm();
  testMotorTestKeepsOutputs();
  testMeasurementSigns();
  testLegRamp();
  testBatteryCounts();
  testWheelSpeedEstimate();
  testBalanceRequested();
  std::printf("control_tick: 12 tests passed\n");
  return 0;
}
