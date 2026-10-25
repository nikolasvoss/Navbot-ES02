#pragma once

#include <stdint.h>

#include "imu.h"

namespace DebugTelemetry {

struct PidStatus {
  float Kp;
  float Ki;
  float Kd;
  float deriv;
  float integral;
  float iLimit;
  float outP;
  float outI;
  float outD;
  float output;
  float error;
};

// Snapshot of the sketch state read by the debug selections, taken once per control tick.
struct TelemetryInputs {
  int selection;
  uint32_t nowMs;
  uint32_t controlGateSequence;
  bool balanceActive;  // Balance gains enabled and the robot is not tumbling.

  imu::Snapshot imu;
  imu::Settings imuSettings;
  imu::CalibrationValues imuCalibration;
  float controlTimestepSec;

  float m1VelocityRadPerSec;
  float m2VelocityRadPerSec;
  float m1FilteredVelocityRadPerSec;
  float m2FilteredVelocityRadPerSec;
  float motor1Target;
  float motor2Target;
  float motor2CurrentSetpoint;
  float sensor1Angle;
  float sensor1MechanicalAngle;
  float sensor2Angle;
  float sensor2MechanicalAngle;

  int16_t sbusChannels[18];
  int sbusFrameIntervalMs;

  PidStatus anglePid;
  PidStatus speedPid;
  PidStatus yawPid;
  PidStatus rollPid;

  ServoTrimsDegrees servoTrims;
  int servoTraceAngle[4];

  float topBallX;
  float topBallY;
  float sbusTopBallXSmoothed;
  float sbusTopBallYSmoothed;
  float ballRollOffset;

  float bodyRoll;
  float bodyPitching;
  float bodyPitchingFiltered;
  float bodyRollFiltered;
  float bodyPitchCorrection;
  float bodyHeightCommandM;
  float bodyHeightCommandFilteredM;
  float bodyTurn;
  float bodyX;
  float bodyCommandFilteringEnabled;
  float bodyCommandFilterCutoffHz;

  float movementSpeed;
  float driveEffectiveSpeed;
  float driveSpeedBodyXRaw;
  float wheelVelocityFeedbackCorrection;

  uint16_t voltageAdc;
  float voltageAdcFiltered;
  float voltage;

  int pidGainsMode;
  int postureOrMarkMode;
  int robotTumble;
  float pidParameterTuning;
  float targetBodyHeightOverrideM;
};

// Telemetry state that persists across control ticks.
struct TelemetryState {
  uint16_t voltageAdcMin;  // Lowest raw battery reading since the last trace row.
  uint32_t traceSequence;
};

// Emits the sample selected by in.selection: a selected-debug or channel sample, or a trace row.
void submit(const TelemetryInputs &in, TelemetryState &state);

}  // namespace DebugTelemetry
