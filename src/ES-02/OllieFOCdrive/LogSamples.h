#pragma once

#include <stdint.h>

namespace Logging {

enum class Level : uint8_t { Debug, Info, Warning, Error };

enum class Profile : uint8_t {
  Idle,
  Diagnostic,
  SelectedDebug,
  Trace,
  Control,
  Balance,
  Drive
};

enum class IncompleteReason : uint8_t {
  None,
  QueueFull,
  SenderStartFailed,
  FormatFailed,
  SenderWriteFailed
};

enum class Result : uint8_t {
  Accepted,
  Filtered,
  Suppressed,
  Paced,
  Truncated,
  FormattingFailed,
  NotStarted,
  Incomplete,
  QueueFull,
  Invalid
};

struct Status {
  IncompleteReason incompleteReason;
  uint32_t queuedRecords;
  uint32_t droppedRecords;
  uint32_t writeFailures;
  uint32_t formatFailures;
  uint32_t truncatedMessages;
  uint32_t rejectedRecords;
  uint32_t filteredMessages;
  uint32_t suppressedMessages;
  uint32_t profileEpoch;
  uint8_t queueDepth;
};

struct DiagnosticSample {
  uint32_t timestampUs;
  uint32_t sequence;
  float gyroX;
  float gyroY;
  float gyroZ;
  float accelX;
  float accelY;
  float accelZ;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float batteryRawV;
  int32_t receiverFrameAgeMs;
  int32_t receiverFailsafe;
  int32_t imuReady;
};

struct TraceSample {
  uint32_t timestampMs;
  uint32_t sequence;
  int32_t gainMode;
  int32_t postureOrMarkMode;
  float minimumBatteryRawV;
  float batteryV;
  float rollOk;
  float pitchOk;
  int32_t servoAngle[4];
  int32_t servoRange[4];
  float leftMotorTarget;
  float rightMotorTarget;
};

struct ControlSample {
  uint32_t timestampMs;
  uint32_t sequence;
  int32_t gainMode;
  float minimumBatteryRawV;
  float batteryV;
  float rollOk;
  float yawRateRadPerSec;
  float bodyTurn;
  float angleError;
  float angleOutput;
  float yawError;
  float yawOutput;
  float leftMotorTarget;
  float rightMotorTarget;
  int32_t maxServoRange;
};

struct BalanceSample {
  uint32_t timestampMs;
  uint32_t sequence;
  int32_t gainMode;
  float minimumBatteryRawV;
  float rollOk;
  float angleError;
  float angleProportional;
  float angleIntegral;
  float angleDerivative;
  float bodyX;
  float leftMotorTarget;
  float rightMotorTarget;
  int32_t maxServoRange;
};

struct DriveSample {
  uint32_t timestampMs;
  uint32_t sequence;
  int32_t gainMode;
  float minimumBatteryRawV;
  float batteryV;
  float requestedSpeed;
  float effectiveSpeed;
  float leftWheelVelocity;
  float rightWheelVelocity;
  float controlDtSec;
  float speedError;
  float speedProportional;
  float speedIntegral;
  float speedDerivative;
  float speedOutput;
  float driveBodyXRaw;
  float bodyX;
  float bodyPitchFiltered;
  float rollOk;
  float angleOutput;
  float angleProportional;
  float angleIntegral;
  float angleDerivative;
  float wheelSpeedFeedbackOutput;
  float leftMotorTarget;
  float rightMotorTarget;
  float ballX;
  float touchXFiltered;
  float bodyPitch;
  int32_t maxServoRange;
};

enum class DebugSelector : uint8_t {
  K1 = 1, K2, K3, K4, K5, K6, K7, K8, K9, K10,
  K11, K12, K13, K14, K15, K16, K17, K18, K19, K20,
  K21, K22, K23, K24, K25, K26, K27, K28, K29, K30,
  K31, K32, K33, K34, K35, K36, K37, K38, K39, K40,
  K41, K42, K43, K44, K45,
  K60 = 60, K61, K62, K63, K64, K65, K66, K67,
  K68, K69, K70, K71, K72, K73, K74, K75
};

struct TimedVector3 { float dtSec; float x; float y; float z; };
struct Attitude3 { float firstDeg; float secondDeg; float thirdDeg; };
struct MotorVelocities { float left; float right; };
struct ReceiverChannels { int32_t channels[10]; float frameDtMs; };
struct ControllerGains { float kp; float ki; float kd; };
struct ControllerGainsSet { ControllerGains angle; ControllerGains speed; ControllerGains yaw; float dtSec; };
struct MahonyAttitude { float twoKp; float twoKi; float rollDeg; float pitchDeg; float imuDtSec; };
struct ImuAxes { float angleX; float angleY; float angleZ; float gyroX; float gyroY; float gyroZ; float dtSec; };
struct ScalarPair { float first; float second; };
struct SensorAngles { float target; float leftAbsolute; float leftMechanical; float rightAbsolute; float rightMechanical; };
struct ServoOffsets { float servo[4]; };
struct BalanceGeometry { float bodyX; float bodyRoll; float legLength; };
struct BalanceState { float rollOk; float bodyPitch; };
struct PidState { float error; float integralLimit; float integral; float integralOutput; float output; };
struct PidCoefficients { float kp; float ki; float kd; };
struct TouchPoint { int32_t state; int32_t x; int32_t y; };
struct TouchFilteredPoint { int32_t x; int32_t y; float xFiltered; float yFiltered; };
struct PidDerivative { float derivative; };
struct TouchState { int32_t state; int32_t start; };
struct BallPosition { float x; float xFiltered; float y; float yFiltered; };
struct BalanceOutputPair { float bodyPitch; float controllerOutput; };
struct RollOutputState { float rollOk; float bodyPitch; float output; };
struct VoltageState { int32_t adc; float filteredV; float voltageV; };
struct TuningState { int32_t enabled; float targetLegLength; };
struct TumbleState { int32_t tumbled; float rollOk; float angleError; };
struct ChannelState { float seconds; int32_t channel; };
struct CurrentSetpoint { float amperes; };
struct BallBalanceGeometry { float ballX; float bodyRoll; float legLength; };
struct PidIntegralState { float integralLimit; float integral; float integralOutput; float output; };
struct PidIntegralOutputState {
  float integralLimit;
  float integral;
  float integralOutput;
  float auxiliary;
  float output;
};
struct FilteredGeometry {
  float bodyPitch;
  float bodyRoll;
  float legLength;
  float slideStep;
  float ballX;
  float ballY;
};
struct PidTuningState { float kp; float ki; float kd; float derivative; float output; };
struct RollCorrection { float rollOk; float bodyPitch; float correctedBodyPitch; };
struct BodyPitchState { float filteredDeg; float rawDeg; };

union DebugPayload {
  TimedVector3 timedVector3;
  Attitude3 attitude;
  MotorVelocities motorVelocities;
  ReceiverChannels receiverChannels;
  ControllerGainsSet controllerGains;
  MahonyAttitude mahony;
  ImuAxes imuAxes;
  ScalarPair pair;
  SensorAngles sensorAngles;
  ServoOffsets servoOffsets;
  BalanceGeometry balanceGeometry;
  BalanceState balanceState;
  PidState pidState;
  PidCoefficients pidCoefficients;
  TouchPoint touchPoint;
  TouchFilteredPoint touchFilteredPoint;
  PidDerivative pidDerivative;
  TouchState touchState;
  BallPosition ballPosition;
  BalanceOutputPair balanceOutputPair;
  RollOutputState rollOutput;
  VoltageState voltageState;
  TuningState tuningState;
  TumbleState tumbleState;
  ChannelState channelState;
  CurrentSetpoint currentSetpoint;
  BallBalanceGeometry ballBalanceGeometry;
  PidIntegralState pidIntegralState;
  PidIntegralOutputState pidIntegralOutputState;
  FilteredGeometry filteredGeometry;
  PidTuningState pidTuningState;
  RollCorrection rollCorrection;
  BodyPitchState bodyPitchState;
};

struct DebugSample {
  DebugSelector selector;
  DebugPayload payload;
};

constexpr uint32_t kQueueDepth = 32;
constexpr uint8_t kMessageTagSize = 16;
constexpr uint8_t kMessageTextSize = 96;

}
