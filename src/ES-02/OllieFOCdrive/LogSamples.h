#pragma once

#include <stdint.h>

namespace Logging {

enum class Level : uint8_t { Debug, Info, Warning, Error };

enum class Profile : uint8_t { Idle, Diagnostic, SelectedDebug, Trace, Control, Balance, Drive };

enum class IncompleteReason : uint8_t { None, QueueFull, SenderStartFailed, FormatFailed, SenderWriteFailed };

enum class Result : uint8_t {
  Accepted,
  Suppressed,
  Paced,
  Truncated,
  FormattingFailed,
  NotStarted,
  Incomplete,
  QueueFull,
  Invalid
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
  uint32_t imuSequence;
  uint32_t imuReadTimeUs;
  float imuDtSec;
  uint32_t imuAcceptedSamples;
  uint32_t imuLongGapCount;
  uint32_t imuMaxIntervalUs;
  uint32_t controlStartUs;
  uint32_t controlExecutionUs;
  uint32_t maxLoopEntryIntervalUs;
  float requestedSpeed;
  float leftWheelVelocity;
  float rightWheelVelocity;
  float controlDtSec;
  float rollOk;
  float leftMotorTarget;
  float rightMotorTarget;
  float leftCurrentSetpoint;
  float rightCurrentSetpoint;
  float leftVoltageQ;
  float rightVoltageQ;
  float leftShaftVelocity;
  float rightShaftVelocity;
  float batteryInstantRawV;
  float batteryInstantFilteredV;
  int32_t maxServoRange;
};

enum class DebugSelector : uint8_t {
  K1 = 1,
  K2,
  K3,
  K4,
  K5,
  K6,
  K7,
  K8,
  K10 = 10,
  K11,
  K12,
  K13,
  K14,
  K15,
  K16,
  K17,
  K18,
  K19,
  K43 = 43,
  K45 = 45,
  K46 = 46,
  K47 = 47,
  K48 = 48,
  K60 = 60,
  K61,
  K62,
  K63,
  K64,
  K65,
  K66,
  K67,
  K68,
  K69,
  K70,
  K71,
  K72,
  K73,
  K74,
  K75
};

constexpr int kSelectedDebugFirst = 1;
constexpr int kSelectedDebugLast = 48;
constexpr int kChannelDebugFirst = 60;
constexpr int kChannelDebugLast = 75;

constexpr bool isSelectedDebugSelector(int value) {
  return value >= kSelectedDebugFirst && value <= kSelectedDebugLast;
}

constexpr bool isChannelDebugSelector(int value) { return value >= kChannelDebugFirst && value <= kChannelDebugLast; }

struct TimedVector3 {
  float dtSec;
  float x;
  float y;
  float z;
};
struct Attitude3 {
  float firstDeg;
  float secondDeg;
  float thirdDeg;
};
struct MotorVelocities {
  float left;
  float right;
};
struct ReceiverChannels {
  int32_t channels[10];
  float frameDtMs;
};
struct MahonyAttitude {
  float twoKp;
  float twoKi;
  float rollDeg;
  float pitchDeg;
  float imuDtSec;
};
struct ImuAxes {
  float angleX;
  float angleY;
  float angleZ;
  float gyroX;
  float gyroY;
  float gyroZ;
  float dtSec;
};
struct ScalarPair {
  float first;
  float second;
};
struct SensorAngles {
  float target;
  float leftAbsolute;
  float leftMechanical;
  float rightAbsolute;
  float rightMechanical;
};
struct ServoOffsets {
  float servo[4];
};
struct VoltageState {
  int32_t adc;
  float filteredV;
  float voltageV;
};
struct TumbleState {
  int32_t tumbled;
  float rollOk;
};
struct ChannelState {
  float seconds;
  int32_t channel;
};
struct CurrentSetpoint {
  float amperes;
};
struct GyroFftSample {
  uint32_t timestampUs;
  float gyroX;
  float gyroY;
  float gyroZ;
  float gyroXFiltered;
  float gyroYFiltered;
  float gyroZFiltered;
};
struct ImuCadenceSample {
  uint32_t sequence;
  uint32_t readTimeUs;
  float dtSec;
  uint32_t acceptedSamples;
  uint32_t longGapCount;
  uint32_t maxIntervalUs;
};
struct MotorStateSample {
  float leftTarget;
  float rightTarget;
  float leftCurrentSetpoint;
  float rightCurrentSetpoint;
  float leftVoltageQ;
  float rightVoltageQ;
  float leftShaftVelocity;
  float rightShaftVelocity;
  float batteryRawV;
  float batteryFilteredV;
};

union DebugPayload {
  TimedVector3 timedVector3;
  Attitude3 attitude;
  MotorVelocities motorVelocities;
  ReceiverChannels receiverChannels;
  MahonyAttitude mahony;
  ImuAxes imuAxes;
  ScalarPair pair;
  SensorAngles sensorAngles;
  ServoOffsets servoOffsets;
  VoltageState voltageState;
  TumbleState tumbleState;
  ChannelState channelState;
  CurrentSetpoint currentSetpoint;
  GyroFftSample gyroFft;
  ImuCadenceSample imuCadence;
  MotorStateSample motorState;
};

struct DebugSample {
  DebugSelector selector;
  DebugPayload payload;
};

constexpr uint32_t kQueueDepth = 32;
constexpr uint8_t kMessageTagSize = 16;
constexpr uint8_t kMessageTextSize = 96;

}  // namespace Logging
