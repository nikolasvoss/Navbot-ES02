#include "DebugTelemetry.h"

#include "BoardConfig.h"
#include "Logging.h"
#include "RobotLogCapture.h"

namespace DebugTelemetry {
namespace {

constexpr uint32_t kTraceGatePeriod = 7;
constexpr int kTraceSelectorFirst = 55;
constexpr int kTraceSelectorLast = 58;

float whenActive(const TelemetryInputs& in, float value) {
  return in.balanceActive ? value : 0.0f;
}

int32_t maxOfFour(const int32_t values[4]) {
  int32_t largest = values[0];
  for (int i = 1; i < 4; ++i) {
    if (values[i] > largest) largest = values[i];
  }
  return largest;
}

// Returns false when the selection has no sample for the current inputs.
bool fillSelectedPayload(const TelemetryInputs& in, Logging::DebugPayload& payload) {
  const imu::Snapshot& snapshot = in.imu;
  const TouchStatus& touch = in.touch;
  switch (in.selection) {
    case 1:
      payload.timedVector3 = {in.controlTimestepSec, snapshot.attitudeDegrees.roll,
                              snapshot.attitudeDegrees.pitch, snapshot.attitudeDegrees.yaw};
      return true;
    case 2:
      payload.timedVector3 = {in.controlTimestepSec, snapshot.accelerationG.x,
                              snapshot.accelerationG.y, snapshot.accelerationG.z};
      return true;
    case 3:
      payload.timedVector3 = {in.controlTimestepSec, snapshot.angularVelocityRadPerSec.x,
                              snapshot.angularVelocityRadPerSec.y,
                              snapshot.angularVelocityRadPerSec.z};
      return true;
    case 4:
      payload.timedVector3 = {in.controlTimestepSec,
                              snapshot.attitudeDegrees.roll - in.imuCalibration.attitude.rollDegrees,
                              snapshot.attitudeDegrees.pitch - in.imuCalibration.attitude.pitchDegrees,
                              snapshot.attitudeDegrees.yaw - 0.0f};
      return true;
    case 5:
      payload.timedVector3 = {in.controlTimestepSec, in.imuCalibration.attitude.rollDegrees,
                              in.imuCalibration.attitude.pitchDegrees, 0.0f};
      return true;
    case 6:
      payload.motorVelocities = {in.m1VelocityRadPerSec, in.m2VelocityRadPerSec};
      return true;
    case 7:
      payload.pair = {in.m1VelocityRadPerSec, in.m1FilteredVelocityRadPerSec};
      return true;
    case 8:
      for (int i = 0; i < 10; ++i) payload.receiverChannels.channels[i] = in.sbusChannels[i];
      payload.receiverChannels.frameDtMs = in.sbusFrameIntervalMs;
      return true;
    case 9:
      payload.controllerGains = {{in.anglePid.Kp, in.anglePid.Ki, in.anglePid.Kd},
                                 {in.speedPid.Kp, in.speedPid.Ki, in.speedPid.Kd},
                                 {in.yawPid.Kp, in.yawPid.Ki, in.yawPid.Kd},
                                 in.controlTimestepSec};
      return true;
    case 10:
      payload.mahony = {in.imuSettings.mahonyTwoKp, in.imuSettings.mahonyTwoKi,
                        snapshot.attitudeDegrees.roll, snapshot.attitudeDegrees.pitch,
                        snapshot.dtSec};
      return true;
    case 11:
      payload.imuAxes = {snapshot.complementaryAnglesDegrees.x,
                         snapshot.complementaryAnglesDegrees.y,
                         snapshot.complementaryAnglesDegrees.z,
                         snapshot.integratedGyroAnglesDegrees.x,
                         snapshot.integratedGyroAnglesDegrees.y,
                         snapshot.integratedGyroAnglesDegrees.z, snapshot.dtSec};
      return true;
    case 12:
      payload.pair = {snapshot.complementaryAnglesDegrees.x, snapshot.attitudeDegrees.roll};
      return true;
    case 13:
      payload.timedVector3 = {in.controlTimestepSec, snapshot.filteredAngularVelocityRadPerSec.x,
                              snapshot.filteredAngularVelocityRadPerSec.y,
                              snapshot.filteredAngularVelocityRadPerSec.z};
      return true;
    case 14:
      payload.timedVector3 = {in.controlTimestepSec, snapshot.filteredAccelerationG.x,
                              snapshot.filteredAccelerationG.y, snapshot.filteredAccelerationG.z};
      return true;
    case 15:
      payload.pair = {snapshot.accelerationG.y, snapshot.filteredAccelerationG.y};
      return true;
    case 16:
      payload.pair = {snapshot.angularVelocityRadPerSec.y,
                      snapshot.filteredAngularVelocityRadPerSec.y};
      return true;
    case 17:
      payload.currentSetpoint = {in.motor2CurrentSetpoint};
      return true;
    case 18:
      payload.sensorAngles = {in.motor2Target, in.sensor1Angle, in.sensor1MechanicalAngle,
                              in.sensor2Angle, in.sensor2MechanicalAngle};
      return true;
    case 19:
      payload.servoOffsets = {{in.servoTrims.servo1Degrees, in.servoTrims.servo2Degrees,
                               in.servoTrims.servo3Degrees, in.servoTrims.servo4Degrees}};
      return true;
    case 20:
      payload.ballBalanceGeometry = {in.topBallX, in.bodyRoll, in.bodyHeightCommandM};
      return true;
    case 21:
      payload.balanceState = {in.imu.correctedRollDegrees, in.bodyPitching};
      return true;
    case 22:
      payload.pidIntegralState = {in.anglePid.iLimit, in.anglePid.integral, in.anglePid.outI,
                                  in.anglePid.output};
      return true;
    case 23:
      payload.pidIntegralOutputState = {in.speedPid.iLimit, in.speedPid.integral,
                                        in.speedPid.outI, in.bodyPitchingFiltered,
                                        in.speedPid.output};
      return true;
    case 24:
      payload.pair = {in.bodyCommandFilteringEnabled, in.bodyCommandFilterCutoffHz};
      return true;
    case 25:
      payload.bodyPitchState = {in.bodyPitchingFiltered, in.bodyPitching};
      return true;
    case 26:
      if (touch.state == 1) {
        payload.touchPoint = {touch.state, touch.XPdat, touch.YPdat};
      } else if (touch.state == 0) {
        payload.touchPoint = {touch.state, touch.XLdat, touch.YLdat};
      } else {
        return false;
      }
      return true;
    case 27:
      payload.touchFilteredPoint = {touch.XPdat, touch.YPdat, touch.XPdatF, touch.YPdatF};
      return true;
    case 28:
      payload.filteredGeometry = {in.bodyPitchingFiltered, in.bodyRollFiltered,
                                  in.bodyHeightCommandFilteredM, 0.0f, in.topBallX,
                                  in.topBallY};
      return true;
    case 29:
      payload.pidTuningState = {in.touchYPid.Kp, in.touchYPid.Ki, in.touchYPid.Kd,
                                in.touchYPid.deriv, in.touchYPid.output};
      return true;
    case 30:
      payload.pidDerivative = {in.touchYPid.deriv};
      return true;
    case 31:
      payload.pidState = {in.rollPid.error, in.rollPid.iLimit, in.rollPid.integral,
                          in.rollPid.outI, in.rollPid.output};
      return true;
    case 32:
      payload.pidCoefficients = {in.rollPid.Kp, in.rollPid.Ki, in.rollPid.Kd};
      return true;
    case 33:
      payload.pidIntegralOutputState = {in.yawPid.iLimit, in.yawPid.integral, in.yawPid.outI,
                                        in.bodyPitchingFiltered, in.yawPid.output};
      return true;
    case 34:
      payload.pidTuningState = {in.touchXPid.Kp, in.touchXPid.Ki, in.touchXPid.Kd,
                                in.touchXPid.deriv, in.touchXPid.output};
      return true;
    case 35:
      payload.pidDerivative = {in.touchXPid.deriv};
      return true;
    case 36:
      payload.touchState = {touch.state, touch.start};
      return true;
    case 37:
      payload.ballPosition = {in.topBallX, in.sbusTopBallXSmoothed, in.topBallY,
                              in.sbusTopBallYSmoothed};
      return true;
    case 38:
      payload.balanceOutputPair = {in.bodyPitching, in.touchYPid.output};
      return true;
    case 39:
      payload.pidIntegralState = {in.touchYPid.iLimit, in.touchYPid.integral,
                                  in.touchYPid.outI, in.touchYPid.output};
      return true;
    case 40:
      if (touch.state != 0 && touch.state != 1) return false;
      payload.touchPoint = {touch.state, touch.XPressDat, touch.YPressDat};
      return true;
    case 41:
      payload.rollOutput = {in.imu.correctedRollDegrees, in.bodyPitching, in.speedPid.output};
      return true;
    case 42:
      payload.rollCorrection = {in.imu.correctedRollDegrees, in.bodyPitching,
                                in.bodyPitchCorrection};
      return true;
    case 43:
      payload.voltageState = {in.voltageAdc, in.voltageAdcFiltered, in.voltage};
      return true;
    case 44:
      payload.tuningState = {static_cast<int32_t>(in.pidParameterTuning),
                             in.targetBodyHeightOverrideM};
      return true;
    case 45:
      payload.tumbleState = {in.robotTumble, in.imu.correctedRollDegrees, in.anglePid.error};
      return true;
    case 46:
      payload.gyroFft = {static_cast<uint32_t>(snapshot.readTimeUs),
                         snapshot.angularVelocityRadPerSec.x, snapshot.angularVelocityRadPerSec.y,
                         snapshot.angularVelocityRadPerSec.z,
                         snapshot.filteredAngularVelocityRadPerSec.x,
                         snapshot.filteredAngularVelocityRadPerSec.y,
                         snapshot.filteredAngularVelocityRadPerSec.z};
      return true;
    default:
      return false;
  }
}

void submitChannel(const TelemetryInputs& in) {
  Logging::DebugSample sample{};
  sample.selector = static_cast<Logging::DebugSelector>(in.selection);
  sample.payload.channelState = {
      in.nowMs * 0.001f, in.sbusChannels[in.selection - Logging::kChannelDebugFirst]};
  Logging::submit(sample);
}

void submitSelected(const TelemetryInputs& in) {
  Logging::DebugSample sample{};
  sample.selector = static_cast<Logging::DebugSelector>(in.selection);
  if (fillSelectedPayload(in, sample.payload)) Logging::submit(sample);
}

// Fields shared by the four trace profiles.
struct TraceRow {
  uint32_t sequence;
  float minimumBatteryRawV;
  int32_t servoRange[4];
  int32_t maxServoRange;
};

TraceRow beginTraceRow(TelemetryState& state) {
  TraceRow row;
  row.sequence = state.traceSequence++;
  row.minimumBatteryRawV = board::kBatteryVoltsPerAdcCount * state.voltageAdcMin;
  RobotLogCapture::copyServoRangesAndBeginNextWindow(row.servoRange);
  row.maxServoRange = maxOfFour(row.servoRange);
  return row;
}

void submitTrace(const TelemetryInputs& in, const TraceRow& row) {
  Logging::TraceSample sample{};
  sample.timestampMs = in.nowMs;
  sample.sequence = row.sequence;
  sample.gainMode = in.pidGainsMode;
  sample.postureOrMarkMode = in.postureOrMarkMode;
  sample.minimumBatteryRawV = row.minimumBatteryRawV;
  sample.batteryV = in.voltage;
  sample.rollOk = in.imu.correctedRollDegrees;
  sample.pitchOk = in.imu.correctedPitchDegrees;
  for (int i = 0; i < 4; ++i) {
    sample.servoAngle[i] = in.servoTraceAngle[i];
    sample.servoRange[i] = row.servoRange[i];
  }
  sample.leftMotorTarget = in.motor1Target;
  sample.rightMotorTarget = in.motor2Target;
  Logging::submit(sample);
}

void submitControl(const TelemetryInputs& in, const TraceRow& row) {
  Logging::ControlSample sample{};
  sample.timestampMs = in.nowMs;
  sample.sequence = row.sequence;
  sample.gainMode = in.pidGainsMode;
  sample.minimumBatteryRawV = row.minimumBatteryRawV;
  sample.batteryV = in.voltage;
  sample.rollOk = in.imu.correctedRollDegrees;
  sample.yawRateRadPerSec = in.imu.angularVelocityRadPerSec.z;
  sample.bodyTurn = in.bodyTurn;
  sample.angleError = whenActive(in, in.anglePid.error);
  sample.angleOutput = whenActive(in, in.anglePid.output);
  sample.yawError = whenActive(in, in.yawPid.error);
  sample.yawOutput = whenActive(in, in.yawPid.output);
  sample.leftMotorTarget = in.motor1Target;
  sample.rightMotorTarget = in.motor2Target;
  sample.maxServoRange = row.maxServoRange;
  Logging::submit(sample);
}

void submitBalance(const TelemetryInputs& in, const TraceRow& row) {
  Logging::BalanceSample sample{};
  sample.timestampMs = in.nowMs;
  sample.sequence = row.sequence;
  sample.gainMode = in.pidGainsMode;
  sample.minimumBatteryRawV = row.minimumBatteryRawV;
  sample.rollOk = in.imu.correctedRollDegrees;
  sample.angleError = whenActive(in, in.anglePid.error);
  sample.angleProportional = whenActive(in, in.anglePid.outP);
  sample.angleIntegral = whenActive(in, in.anglePid.outI);
  sample.angleDerivative = whenActive(in, in.anglePid.outD);
  sample.bodyX = whenActive(in, in.bodyX);
  sample.leftMotorTarget = in.motor1Target;
  sample.rightMotorTarget = in.motor2Target;
  sample.maxServoRange = row.maxServoRange;
  Logging::submit(sample);
}

void submitDrive(const TelemetryInputs& in, const TraceRow& row) {
  Logging::DriveSample sample{};
  sample.timestampMs = in.nowMs;
  sample.sequence = row.sequence;
  sample.gainMode = in.pidGainsMode;
  sample.minimumBatteryRawV = row.minimumBatteryRawV;
  sample.batteryV = in.voltage;
  sample.requestedSpeed = in.movementSpeed;
  sample.effectiveSpeed = whenActive(in, in.driveEffectiveSpeed);
  sample.leftWheelVelocity = in.m1FilteredVelocityRadPerSec;
  sample.rightWheelVelocity = in.m2FilteredVelocityRadPerSec;
  sample.controlDtSec = in.controlTimestepSec;
  sample.speedError = whenActive(in, in.speedPid.error);
  sample.speedProportional = whenActive(in, in.speedPid.outP);
  sample.speedIntegral = whenActive(in, in.speedPid.outI);
  sample.speedDerivative = whenActive(in, in.speedPid.outD);
  sample.speedOutput = whenActive(in, in.speedPid.output);
  sample.driveBodyXRaw = whenActive(in, in.driveSpeedBodyXRaw);
  sample.bodyX = whenActive(in, in.bodyX);
  sample.bodyPitchFiltered = in.bodyPitchingFiltered;
  sample.rollOk = in.imu.correctedRollDegrees;
  sample.angleOutput = whenActive(in, in.anglePid.output);
  sample.angleProportional = whenActive(in, in.anglePid.outP);
  sample.angleIntegral = whenActive(in, in.anglePid.outI);
  sample.angleDerivative = whenActive(in, in.anglePid.outD);
  sample.wheelSpeedFeedbackOutput = whenActive(in, in.wheelVelocityFeedbackCorrection);
  sample.leftMotorTarget = in.motor1Target;
  sample.rightMotorTarget = in.motor2Target;
  sample.ballX = in.topBallX;
  sample.touchXFiltered = in.touch.XPdatF;
  sample.bodyPitch = in.bodyPitching;
  sample.maxServoRange = row.maxServoRange;
  Logging::submit(sample);
}

void submitTraceProfile(const TelemetryInputs& in, TelemetryState& state) {
  const TraceRow row = beginTraceRow(state);
  switch (in.selection) {
    case 55: submitTrace(in, row); break;
    case 56: submitControl(in, row); break;
    case 57: submitBalance(in, row); break;
    case 58: submitDrive(in, row); break;
  }
  state.voltageAdcMin = in.voltageAdc;
}

bool isTraceSelector(int selection) {
  return selection >= kTraceSelectorFirst && selection <= kTraceSelectorLast;
}

}  // namespace

bool due(int selection, uint32_t nowMs, uint32_t controlGateSequence) {
  if (Logging::isChannelDebugSelector(selection) || Logging::isSelectedDebugSelector(selection))
    return RobotLogCapture::selectedDebugDue(static_cast<Logging::DebugSelector>(selection), nowMs);
  if (isTraceSelector(selection)) return controlGateSequence % kTraceGatePeriod == 0;
  return false;
}

void submit(const TelemetryInputs& in, TelemetryState& state) {
  if (Logging::isChannelDebugSelector(in.selection)) {
    submitChannel(in);
  } else if (Logging::isSelectedDebugSelector(in.selection)) {
    submitSelected(in);
  } else if (isTraceSelector(in.selection)) {
    submitTraceProfile(in, state);
  }
}

}  // namespace DebugTelemetry
