#include <Arduino.h>
#include <SimpleFOC.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "BoardConfig.h"
#include "Calibration.h"
#include "CalibrationStore.h"
#include "DebugTelemetry.h"
#include "FUTABA_SBUS.h"
#include "LegKinematics.h"
#include "Logging.h"
#include "MotorDiagnostic.h"
#include "OverspeedGuard.h"
#include "WheelBoundary.h"
#include "OllieFOCdrive.h"
#include "RobotLogCapture.h"
#include "ServoControl.h"
#include "SerialCommandRouter.h"
#include "filter.h"
#include "imu.h"

bool motorDiagnosticOwnsOutput = false;

class LoggingCommandStream : public Stream {
 public:
  int available() override { return Serial.available(); }
  int read() override { return Serial.read(); }
  int peek() override { return Serial.peek(); }
  size_t write(uint8_t value) override {
    if (motorDiagnosticOwnsOutput || Logging::profile() != Logging::Profile::Idle) return 1;
    return Serial.write(value);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    if (motorDiagnosticOwnsOutput || Logging::profile() != Logging::Profile::Idle) return size;
    return Serial.write(buffer, size);
  }
  void flush() override {
    if (!motorDiagnosticOwnsOutput && Logging::profile() == Logging::Profile::Idle) Serial.flush();
  }
};

LoggingCommandStream commandStream;
Commander command = Commander(commandStream);
Calibration calibration;
CalibrationStore calibrationStore;

// ----- Editable Constants
#define COMMUNICATION_OBJECT \
  COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE  // 0: two-wheel balance  1: simpleFOC Studio host computer  2: control dual
                                          // motors

// Safe bring-up: read sensors and receiver without energizing wheels or servos.
// Set to 0 only after the diagnostic readings and power supply are checked.
#define SENSOR_DIAGNOSTIC_MODE 0

// Tuned two-wheel drive defaults, including corrected wheel-speed timing and
// CH3 scaling. Apply them at startup and through a CH5 switch transition.
// Set to 0 to restore the repository's mode-dependent defaults.
#define DIAGNOSTIC_LIVE_TUNING_DEFAULTS 1

struct PidPreset {
  float p;
  float i;
  float d;
  float limit;  // Integral limit
};

struct BalancePidPreset {
  PidPreset roll;
  PidPreset speed;
  PidPreset angle;
};

// Two-wheel PID gains for remote control mode.
constexpr BalancePidPreset kBalancePid{{0.08, 1.5, 0.005, 2}, {0.045, 0.005, 0, 50}, {5, 200, 0.11, 0.1}};
constexpr PidPreset kYawPid{4, 0, 0, 0};

#define SBUS_CHANNEL_MAX 1792
#define SBUS_CHANNEL_MIN 192
constexpr uint32_t kRcFrameTimeoutMs = 100;
constexpr float kControlTimestepMinSec = 0.001f;

constexpr unsigned long kSerialBaudRate = 576000;
constexpr size_t kSerialTxBufferBytes = 1024;
#define DIAGNOSTIC_LOG_INTERVAL_US 5000  // Diagnostic CSV rows are emitted at 200 Hz.
constexpr unsigned int kDiagnosticLogSampleRateHz = 1000000U / DIAGNOSTIC_LOG_INTERVAL_US;
// Conservative drive tuning parameters; verify the wheel feedback sign on hardware.
constexpr float kDriveTiltReductionStartDeg = 5.0f;
constexpr float kDriveTiltReductionFullDeg = 10.0f;
// -------------------------------------

struct BodyPose {
  float heightM;
  float roll;
};

// Wheel motor constants, measured 2026-10-22 (docs/robot/software/motor-commissioning.md).
// SimpleFOC uses phase_resistance as the per-phase (line-to-neutral) value in voltage.q = I * R + bemf.
// The meter reads 11.5 ohm phase to phase on both wheels; 5.75 ohm assumes a star winding, not yet verified.
struct WheelMotorParams {
  float phaseResistanceOhm;
  float kvRatingRpmPerV;  // SimpleFOC convention: V_bemf = velocity / (KV * sqrt(3) * RPM_TO_RADS)
};

constexpr float kWheelSupplyVoltageV = 8.4;
constexpr float kWheelVoltageLimitV = 4.2;  // usable q ceiling at about 7.3 V bus; the modulation limit is bus / sqrt(3)
constexpr float kWheelVelocityFilterTfSec = 0.001;
constexpr WheelMotorParams kLeftWheelMotor{5.75f, 79.5f};
constexpr WheelMotorParams kRightWheelMotor{5.75f, 76.7f};

void configureWheelMotor(BLDCMotor &motor, const WheelMotorParams &params);

constexpr float kBodyHeightOverrideDisabledM = 0.0f;
constexpr float kBodyHeightMinM = 0.05f;
constexpr float kBodyHeightDefaultM = 0.06f;
constexpr float kBallPoiseMaxBodyHeightM = 0.07f;
constexpr float kBodyHeightMaxM = 0.09f;
constexpr int kBodyHeightCommandFilterIndex = 2;

// Body
float targetBodyHeightOverrideM = kBodyHeightOverrideDisabledM;  // Zero disables the Commander height override.
float bodyHeightCommandM = kBodyHeightDefaultM;                  // Remote-controlled body height
float bodyPitching = 0;                                          // Pitch
float bodyRoll = 0;                                              // Roll
float movementSpeed = 0;                                         // Movement speed
float bodyTurn = 0;                                              // Turning
int robotTumble = ROBOT_NOT_TUMBLING;  // Fall-detection state, see docs/robot/ROBOT_ROADMAP.md

// Filtered remote-control commands
float bodyHeightCommandFilteredM = kBodyHeightDefaultM;
float bodyPitchingFiltered = 0;           // Pitch
float bodyRollFiltered = 0;               // Roll
biquadFilter_t bodyCommandFilterLpf[12];  // Second-order low-pass filter

float bodyCommandFilterCutoffHz = 200;  // Low-pass cutoff frequency (Hz) for remote-control body command filtering.
float bodyCommandFilteringEnabled = 1;

void setBodyCommandFilterCutoffHz(char *cmd) { command.scalar(&bodyCommandFilterCutoffHz, cmd); }

void setBodyCommandFilteringEnabled(char *cmd) { command.scalar(&bodyCommandFilteringEnabled, cmd); }

int statusLedOn = 1;
int statusLedTicks = 0;
int statusLedPeriodTicks = 100;
// Battery Voltage Measurement
biquadFilter_t voltageFilterLpf;  // Second-order low-pass filter
uint16_t voltageAdc = 0;          // Battery voltage ADC data
float voltageAdcFiltered = 0;     // Battery voltage ADC data
float batteryVoltage = 0;         // Battery voltage

uint32_t controlGateSequence = 0;
DebugTelemetry::TelemetryState debugTelemetryState{};
uint32_t diagnosticSequence = 0;
ServoTrimsDegrees servoTrims{};

bool parseCommandFloat(const char *text, float &value) {
  char *end = nullptr;
  value = strtof(text, &end);
  if (end == text) return false;
  while (end && isspace(static_cast<unsigned char>(*end))) ++end;
  return end && *end == '\0' && isfinite(value);
}

bool parseCommandCutoff(const char *text, uint16_t &value) {
  char *end = nullptr;
  const long parsed = strtol(text, &end, 10);
  while (end && isspace(static_cast<unsigned char>(*end))) ++end;
  if (end == text || !end || *end != '\0' || parsed < 1 || parsed > UINT16_MAX) return false;
  value = static_cast<uint16_t>(parsed);
  return true;
}

void setImuLowPassCutoff(char *cmd) {
  uint16_t cutoff = 0;
  imu::Settings candidate = imu::settings();
  if (!parseCommandCutoff(cmd, cutoff)) {
    Logging::message(Logging::Level::Warning, "IMU", "invalid cutoff");
    return;
  }
  candidate.gyroCutoffHz = cutoff;
  if (!imu::setSettings(candidate)) {
    Logging::message(Logging::Level::Warning, "IMU", "cutoff outside valid range");
    return;
  }
  Logging::message(Logging::Level::Info, "LPF_CUTOFF_FREQ", "%u", cutoff);
}

void setTargetBodyHeight(char *cmd) { command.scalar(&targetBodyHeightOverrideM, cmd); }

// Servo zero position offsets
void zeroBiasServo1(char *cmd) { command.scalar(&servoTrims.servo1Degrees, cmd); }
void zeroBiasServo2(char *cmd) { command.scalar(&servoTrims.servo2Degrees, cmd); }
void zeroBiasServo3(char *cmd) { command.scalar(&servoTrims.servo3Degrees, cmd); }
void zeroBiasServo4(char *cmd) { command.scalar(&servoTrims.servo4Degrees, cmd); }

bool isPidGainsModeEnabled(int mode) {
  return mode == REMOTE_CONTROL_PID_GAINS_MODE_ON;
}

//  Create ServoControl object
ServoControl servoControl(board::kLegServoPin[0], board::kLegServoPin[1], board::kLegServoPin[2],
                          board::kLegServoPin[3]);
int servoTraceAngle[4] = {0, 0, 0, 0};  // Last angle arguments sent to the four servos

// Remote control
FUTABA_SBUS sBus;
int sbusFrameIntervalMs = 0;
int pidGainsMode = REMOTE_CONTROL_PID_GAINS_MODE_OFF;
int postureOrMarkMode = REMOTE_CONTROL_PM_POSTURE_MODE;
int attitudeMode = REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT;
float topBallX = 0;
float topBallY = 0;
float sbusTopBallXSmoothed = 0;
float sbusTopBallYSmoothed = 0;

//  Create PID controller instance
float debugLogSelector = 0;   // Selects debug data and the log profile.
float calibrationSelect = 0;  // Save calibration data 0: Calibration end  1: Calibrate gyroscope  2: Calibrate Euler
                              // angle  3: Calibrate servo

float pidParameterTuning = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? 1 : 0;  // 0: auto gains  1: live tuning

PIDController anglePidSettings(kBalancePid.angle.p, kBalancePid.angle.i, kBalancePid.angle.d, 0,
                               kBalancePid.angle.limit);
PIDController yawPidSettings(kYawPid.p, kYawPid.i, kYawPid.d, 0, kYawPid.limit);

float controlTorqueCompensation = 0;  // Control torque compensation
float driveTiltReductionGain =
    1.0f;  // Scales the speed reduction applied as tilt approaches kDriveTiltReductionFullDeg.

float pidTimestepSec = 0.01;

MyPIDController anglePid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);  // p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController yawPid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);

bool balancePidNeedsPriming = true;  // Priming avoids a derivative kick when CH5 enables the balance loop.

void setControlTorqueCompensation(char *cmd) { command.scalar(&controlTorqueCompensation, cmd); }

void setDriveTiltReductionGain(char *cmd) { command.scalar(&driveTiltReductionGain, cmd); }

void setMahonyKp(char *cmd) {
  float value = 0.0f;
  imu::Settings candidate = imu::settings();
  if (!parseCommandFloat(cmd, value)) return;
  candidate.mahonyTwoKp = value;
  if (imu::setSettings(candidate))
    Logging::message(Logging::Level::Info, "MAHONY_KP", "%.6f", value);
  else
    Logging::message(Logging::Level::Warning, "IMU", "invalid Mahony Kp");
}
void setMahonyKi(char *cmd) {
  float value = 0.0f;
  imu::Settings candidate = imu::settings();
  if (!parseCommandFloat(cmd, value)) return;
  candidate.mahonyTwoKi = value;
  if (imu::setSettings(candidate))
    Logging::message(Logging::Level::Info, "MAHONY_KI", "%.6f", value);
  else
    Logging::message(Logging::Level::Warning, "IMU", "invalid Mahony Ki");
}

void selectDebugLog(char *cmd) {
  command.scalar(&debugLogSelector, cmd);
  RobotLogCapture::setProfileForSelector(static_cast<int>(debugLogSelector));
}

// Cb prefix: Commander callback.
void setAnglePid(char *cmd) { command.pid(&anglePidSettings, cmd); }
void setYawPid(char *cmd) { command.pid(&yawPidSettings, cmd); }

// Wheel Motors and Drivers
// m1 is the left wheel and m2 the right wheel; speeds are forward positive (see WheelBoundary.h).
WheelBoundary::WheelSpeedEstimator wheelSpeedEstimator;
float m1VelocityRadPerSec = 0;
float m1FilteredVelocityRadPerSec = 0;
LowPassFilter m1VelocityFilter = LowPassFilter(0.01);  // Tf = 10ms

float m2VelocityRadPerSec = 0;
float m2FilteredVelocityRadPerSec = 0;
LowPassFilter m2VelocityFilter = LowPassFilter(0.01);  // Tf = 10ms

float controlTimestepSec = 0;
unsigned long loopTimeUs = 0;
unsigned long previousControlTimeUs = 0;
uint32_t controlStartUs = 0;
uint32_t controlExecutionUs = 0;
uint32_t previousLoopEntryUs = 0;
bool hasPreviousLoopEntry = false;

BLDCMotor motor1 = BLDCMotor(board::kWheelMotorPolePairs);
BLDCDriver3PWM driver =
    BLDCDriver3PWM(board::kWheel1PhaseAPin, board::kWheel1PhaseBPin, board::kWheel1PhaseCPin, board::kWheel1EnablePin);

BLDCMotor motor2 = BLDCMotor(board::kWheelMotorPolePairs);
BLDCDriver3PWM driver2 =
    BLDCDriver3PWM(board::kWheel2PhaseAPin, board::kWheel2PhaseBPin, board::kWheel2PhaseCPin, board::kWheel2EnablePin);

MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire i2cOne = TwoWire(0);
TwoWire i2cTwo = TwoWire(1);

MotorDiagnostic::State motorDiagnosticState{};
bool motorDiagnosticApplied = false;

void selectCalibration(char *cmd) {
  float requested = calibrationSelect;
  command.scalar(&requested, cmd);
  if (static_cast<int>(requested) != 0 && (motorDiagnosticState.active || motorDiagnosticApplied)) {
    Logging::message(Logging::Level::Warning, "Calibration", "refused: motor diagnostic active");
    return;
  }
  calibrationSelect = requested;
}
bool diagnosticRestoredAwaitingResume = false;
bool previousBalanceEnabled = false;

char *trimDiagnosticCommand(char *cmd) {
  while (*cmd == ' ' || *cmd == '\t') ++cmd;
  size_t length = strlen(cmd);
  while (length > 0 && (cmd[length - 1] == ' ' || cmd[length - 1] == '\t' || cmd[length - 1] == '\r' ||
                        cmd[length - 1] == '\n'))
    cmd[--length] = '\0';
  return cmd;
}

void zeroMotorOutput(BLDCMotor &motor) {
  motor.target = 0.0f;
  motor.current_sp = 0.0f;
  motor.voltage.q = 0.0f;
  motor.voltage.d = 0.0f;
}

void restoreMotorConfiguration(BLDCMotor &motor, const WheelMotorParams &params) {
  zeroMotorOutput(motor);
  configureWheelMotor(motor, params);
}

MotorDiagnostic::Inputs diagnosticInputs() {
  return {millis(), pidGainsMode != REMOTE_CONTROL_PID_GAINS_MODE_OFF,
          calibrationSelect != 0.0f, motor1.shaft_velocity, motor2.shaft_velocity};
}

void applyMotorDiagnosticRequest() {
  if (!motorDiagnosticState.active) return;
  const MotorDiagnostic::Request &request = motorDiagnosticState.request;
  restoreMotorConfiguration(motor1, kLeftWheelMotor);
  restoreMotorConfiguration(motor2, kRightWheelMotor);
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;
  if (request.mode == MotorDiagnostic::Mode::CurrentTest) {
    motor1.torque_controller = TorqueControlType::estimated_current;
    motor2.torque_controller = TorqueControlType::estimated_current;
    motor1.current_limit = MotorDiagnostic::kMaximumCurrentA;
    motor2.current_limit = MotorDiagnostic::kMaximumCurrentA;
    motor1.voltage_limit = MotorDiagnostic::kMaximumVoltageV;
    motor2.voltage_limit = MotorDiagnostic::kMaximumVoltageV;
    motor1.target = request.leftCurrentA;
    motor2.target = request.rightCurrentA;
    driver.enable();
    driver2.enable();
  } else {
    motor1.torque_controller = TorqueControlType::voltage;
    motor2.torque_controller = TorqueControlType::voltage;
    motor1.phase_resistance = NOT_SET;
    motor2.phase_resistance = NOT_SET;
    motor1.voltage_limit = MotorDiagnostic::kMaximumVoltageV;
    motor2.voltage_limit = MotorDiagnostic::kMaximumVoltageV;
    if (request.selectedMotor == 1) {
      motor1.target = request.voltageV;
      motor2.target = 0.0f;
      driver.enable();
      driver2.disable();
    } else {
      motor1.target = 0.0f;
      motor2.target = request.voltageV;
      driver.disable();
      driver2.enable();
    }
  }
  motorDiagnosticApplied = true;
  motorDiagnosticOwnsOutput = true;
  diagnosticRestoredAwaitingResume = false;
}

OverspeedGuard::State wheelOverspeed{};

// Coast fault: 0 A and both drivers off. It repeats every tick while tripped, so no other path can re-enable them.
void serviceWheelOverspeed() {
  const bool wasTripped = wheelOverspeed.tripped;
  if (!OverspeedGuard::update(wheelOverspeed, millis(), m1VelocityRadPerSec, m2VelocityRadPerSec)) return;
  motor1.target = 0.0f;
  motor2.target = 0.0f;
  driver.disable();
  driver2.disable();
  if (!wasTripped)
    Logging::message(Logging::Level::Warning, "WHEEL_OVERSPEED", "peak=%.1frad_s limit=%.1frad_s; both drivers disabled",
                     wheelOverspeed.peakRadPerSec, OverspeedGuard::kLimitRadPerSec);
}

// RESET rearms the guard only while both wheels are below the limit; the drivers stay off until balance is re-enabled.
void resetWheelOverspeed() {
  if (fabsf(m1VelocityRadPerSec) <= OverspeedGuard::kLimitRadPerSec &&
      fabsf(m2VelocityRadPerSec) <= OverspeedGuard::kLimitRadPerSec)
    OverspeedGuard::reset(wheelOverspeed);
}

// Shared by T and M so a flood of bad lines cannot fill the logger queue.
bool diagnosticWarningDue() {
  static bool warned = false;
  static uint32_t lastWarningMs = 0;
  const uint32_t nowMs = millis();
  if (warned && static_cast<uint32_t>(nowMs - lastWarningMs) < 500) return false;
  warned = true;
  lastWarningMs = nowMs;
  return true;
}

void handleM1MotionCommand(char *cmd) {
  cmd = trimDiagnosticCommand(cmd);
  if (strcmp(cmd, "RESET") == 0) {
    resetWheelOverspeed();
    if (MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs()))
      Logging::message(Logging::Level::Info, "MOTOR_TEST", "fault reset; outputs remain disabled");
    else
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "reset refused; balance off and safe speed required");
    return;
  }
  if (strcmp(cmd, "STOP") == 0) {
    MotorDiagnostic::stop(motorDiagnosticState);
    return;
  }
  MotorDiagnostic::Request request{};
  if (!MotorDiagnostic::parseCurrentRequest(cmd, request)) {
    if (diagnosticWarningDue())
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "use T <left_A> <right_A> <lease_ms>, T STOP, or T RESET");
    return;
  }
  if (wheelOverspeed.tripped) {
    if (diagnosticWarningDue())
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "request refused; wheel overspeed latched, send RESET");
    return;
  }
  const auto result = MotorDiagnostic::start(motorDiagnosticState, request, diagnosticInputs());
  if (result == MotorDiagnostic::RequestResult::Accepted) {
    applyMotorDiagnosticRequest();
    Logging::message(Logging::Level::Info, "MOTOR_TEST", "current request left=%.3fA right=%.3fA lease=%lums",
                     request.leftCurrentA, request.rightCurrentA, static_cast<unsigned long>(request.leaseMs));
  } else if (diagnosticWarningDue()) {
    Logging::message(Logging::Level::Warning, "MOTOR_TEST", "request refused result=%d", static_cast<int>(result));
  }
}

void handleM1Command(char *cmd) {
  cmd = trimDiagnosticCommand(cmd);
  if (strcmp(cmd, "RESET") == 0) {
    resetWheelOverspeed();
    if (MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs()))
      Logging::message(Logging::Level::Info, "MOTOR_TEST", "fault reset; outputs remain disabled");
    else
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "reset refused; balance off and safe speed required");
    return;
  }
  if (strcmp(cmd, "STOP") == 0) {
    MotorDiagnostic::stop(motorDiagnosticState);
    return;
  }
  MotorDiagnostic::Request request{};
  if (!MotorDiagnostic::parseVoltageRequest(cmd, request)) {
    if (diagnosticWarningDue())
      Logging::message(Logging::Level::Warning, "MOTOR_ID", "use M <motor_1|2> <voltage_V> <lease_ms>, M STOP, or M RESET");
    return;
  }
  if (wheelOverspeed.tripped) {
    if (diagnosticWarningDue())
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "request refused; wheel overspeed latched, send RESET");
    return;
  }
  const auto result = MotorDiagnostic::start(motorDiagnosticState, request, diagnosticInputs());
  if (result == MotorDiagnostic::RequestResult::Accepted) {
    applyMotorDiagnosticRequest();
    Logging::message(Logging::Level::Info, "MOTOR_ID", "motor=%u requested_q=%.3fV lease=%lums",
                     request.selectedMotor, request.voltageV, static_cast<unsigned long>(request.leaseMs));
  } else if (diagnosticWarningDue()) {
    Logging::message(Logging::Level::Warning, "MOTOR_ID", "request refused result=%d", static_cast<int>(result));
  }
}

void serviceMotorDiagnostic() {
  const bool balanceEnabled = pidGainsMode != REMOTE_CONTROL_PID_GAINS_MODE_OFF;
  const MotorDiagnostic::Fault before = motorDiagnosticState.fault;
  const MotorDiagnostic::Fault fault = MotorDiagnostic::update(motorDiagnosticState, diagnosticInputs());
  const bool faulted = fault != MotorDiagnostic::Fault::None && motorDiagnosticApplied;
  const bool stopped = !motorDiagnosticState.active && motorDiagnosticApplied;
  if (faulted || stopped) {
    const MotorDiagnostic::Request request = motorDiagnosticState.request;
    const float actualVoltage = request.mode == MotorDiagnostic::Mode::VoltageIdentification
                                    ? (request.selectedMotor == 1 ? motor1.voltage.q : motor2.voltage.q)
                                    : 0.0f;
    const float shaftVelocity = request.mode == MotorDiagnostic::Mode::VoltageIdentification
                                    ? (request.selectedMotor == 1 ? motor1.shaft_velocity : motor2.shaft_velocity)
                                    : 0.0f;
    // Disable before restoring any configuration or emitting a message.
    driver.disable();
    driver2.disable();
    zeroMotorOutput(motor1);
    zeroMotorOutput(motor2);
    restoreMotorConfiguration(motor1, kLeftWheelMotor);
    restoreMotorConfiguration(motor2, kRightWheelMotor);
    motorDiagnosticApplied = false;
    motorDiagnosticOwnsOutput = false;
    diagnosticRestoredAwaitingResume = true;
    if (request.mode == MotorDiagnostic::Mode::VoltageIdentification)
      Logging::message(Logging::Level::Info, "MOTOR_ID_RESULT",
                       "motor=%u requested_q=%.3fV actual_q=%.3fV shaft_velocity=%.3frad_s result=%s fault=%d",
                       request.selectedMotor, request.voltageV, actualVoltage, shaftVelocity,
                       stopped ? "stopped" : "fault", static_cast<int>(fault));
  }
  if (faulted) {
    if (before == MotorDiagnostic::Fault::None)
      Logging::message(Logging::Level::Warning, "MOTOR_TEST", "diagnostic fault=%d; both drivers disabled",
                       static_cast<int>(fault));
  }
  if (motorDiagnosticState.active) {
    if (!motorDiagnosticApplied) applyMotorDiagnosticRequest();
  } else if (diagnosticRestoredAwaitingResume && balanceEnabled && !previousBalanceEnabled &&
             motorDiagnosticState.fault == MotorDiagnostic::Fault::None) {
    driver.enable();
    driver2.enable();
    diagnosticRestoredAwaitingResume = false;
  }
  previousBalanceEnabled = balanceEnabled;
}

void readSbus();
void runPostureController(float dt);
void filterRemoteControl(void);
void readBatteryVoltage(void);
void applyPidParameters(void);
void updateTumbleState(void);
void diagnosticLoop(void);
bool diagnosticImuReady = false;
unsigned long diagnosticLastRcFrameMs = 0;
bool diagnosticHasRcFrame = false;

void logCalibration(const PersistedCalibrationValues &calibrationValues) {
  Logging::message(Logging::Level::Info, "Calibration", "Roll Zero Bias: %.2f, Pitch Zero Bias: %.2f",
                   calibrationValues.attitude.rollDegrees, calibrationValues.attitude.pitchDegrees);
  Logging::message(Logging::Level::Info, "Calibration", "gyroBiasX: %.2f, gyroBiasY: %.2f, gyroBiasZ: %.2f",
                   calibrationValues.gyro.xRawCounts, calibrationValues.gyro.yRawCounts,
                   calibrationValues.gyro.zRawCounts);
  Logging::message(Logging::Level::Info, "Calibration", "servo1: %.2f, servo2: %.2f, servo3: %.2f, servo4: %.2f",
                   calibrationValues.servos.servo1Degrees, calibrationValues.servos.servo2Degrees,
                   calibrationValues.servos.servo3Degrees, calibrationValues.servos.servo4Degrees);
}

void configureWheelMotor(BLDCMotor &motor, const WheelMotorParams &params) {
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::torque;
  motor.motion_downsample = 0.0;

  // shaft_velocity feeds the back-EMF estimate, so the velocity filter stays.
  motor.LPF_velocity.Tf = kWheelVelocityFilterTfSec;

  motor.phase_resistance = params.phaseResistanceOhm;
  motor.KV_rating = params.kvRatingRpmPerV;
  motor.voltage_limit = kWheelVoltageLimitV;
  motor.current_limit = kWheelVoltageLimitV / params.phaseResistanceOhm;  // stall current at the voltage ceiling
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
}

void initIMU(const imu::CalibrationValues &imuCalibration) {
  if (!imu::begin(imu::Settings{}, imuCalibration)) {
    Serial.println("IMU initialization failed!");
    while (1);
  }
  Serial.println("IMU initialized successfully!");
}

void initRemoteControl() {
  sBus.begin();  // TODO rename sBus to rc or similar
}

void initFilters() {
  for (int i = 0; i < 6; i++)
    biquadFilterInitLPF(&bodyCommandFilterLpf[i], (unsigned int)bodyCommandFilterCutoffHz, 1000);

  biquadFilterInitLPF(&bodyCommandFilterLpf[8], 50, 1000);
  biquadFilterInitLPF(&bodyCommandFilterLpf[9], 50, 1000);
}

void initPositionSensors() {
  i2cOne.begin(board::kI2cOneSdaPin, board::kI2cOneSclPin, 400000);
  i2cTwo.begin(board::kI2cTwoSdaPin, board::kI2cTwoSclPin, 400000);
  sensor1.init(&i2cOne);
  sensor2.init(&i2cTwo);

  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
}

void initDrivers() {
  driver.voltage_power_supply = kWheelSupplyVoltageV;
  driver.init();

  driver2.voltage_power_supply = kWheelSupplyVoltageV;
  driver2.init();

  motor1.linkDriver(&driver);
  motor2.linkDriver(&driver2);
}

const SerialCommandEntry commandEntries[] = {
    {'T', handleM1MotionCommand, "bounded per-wheel current test or STOP/RESET"},
    {'M', handleM1Command, "bounded selected-wheel voltage identification or STOP/RESET"},
    {'A', zeroBiasServo1, "my zeroBias_servo1"},
    {'B', zeroBiasServo2, "my zeroBias_servo2"},
    {'C', zeroBiasServo3, "my zeroBias_servo3"},
    {'D', zeroBiasServo4, "my zeroBias_servo4"},
    {'Z', setImuLowPassCutoff, "my ImuLPF_CUTOFF_FREQ"},
    {'Q', setMahonyKp, "my TwoKp"},
    {'I', setMahonyKi, "my TwoKi"},
    {'K', selectDebugLog, "my Select"},
    {'E', selectCalibration, "my CalibrationSelect"},
    {'F', setBodyCommandFilterCutoffHz, "my CutoffFreq"},
    {'J', setBodyCommandFilteringEnabled, "my EnableDFilter"},
    {'P', setAnglePid, "anglePidSettings"},
    {'Y', setYawPid, "yawPidSettings"},
    {'O', setTargetBodyHeight, "my Target_Body_Height"},
    {'W', setDriveTiltReductionGain, "drive tilt reduction gain"},
};

SerialCommandRouter serialCommandRouter;

void setAssemblyPosition() { servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1); }

/**
 * @brief Initializes the robot's hardware and software components.
 *
 * This function runs once at startup. It configures serial communication,
 * initializes sensors (IMU, encoders), sets up motors and drivers with SimpleFOC,
 * configures PID controllers, initializes servos, reads calibration data from flash,
 * and sets up the command interface for serial debugging and tuning.
 */
void setup() {
#if SENSOR_DIAGNOSTIC_MODE
  // SimpleFOC's plain BLDCDriver3PWM enable pins are active high.
  // Hold both drivers disabled before any other peripheral is initialized.
  pinMode(board::kWheel1EnablePin, OUTPUT);
  digitalWrite(board::kWheel1EnablePin, LOW);
  pinMode(board::kWheel2EnablePin, OUTPUT);
  digitalWrite(board::kWheel2EnablePin, LOW);
#endif

  Serial.setTxBufferSize(kSerialTxBufferBytes);
  Serial.begin(kSerialBaudRate);
  Logging::begin();
  const PersistedCalibrationValues loadedCalibration = calibrationStore.load();
  servoTrims = loadedCalibration.servos;
  const imu::CalibrationValues imuCalibration{loadedCalibration.gyro, loadedCalibration.attitude};
  logCalibration(loadedCalibration);

  // TODO is this the right location?
  pinMode(board::kStatusLedPin, OUTPUT);
  digitalWrite(board::kStatusLedPin, LOW);
  Serial.println("system run.");
  delay(500);

#if SENSOR_DIAGNOSTIC_MODE
  biquadFilterInitLPF(&voltageFilterLpf, 20, kDiagnosticLogSampleRateHz);
  diagnosticImuReady = imu::begin(imu::Settings{}, imuCalibration);
  sBus.begin();
  Serial.printf("DIAG,boot,reset_reason=%d,imu_ok=%d,motors=off,servos=off\n", (int)esp_reset_reason(),
                diagnosticImuReady ? 1 : 0);
  Logging::setProfile(Logging::Profile::Diagnostic);
  return;
#endif

  biquadFilterInitLPF(&voltageFilterLpf, 50, 1000);  // TODO replace numbers with names

  //  Initialize servo
  servoControl.initialize();

  setAssemblyPosition();
  _delay(555);

  setAssemblyPosition();  // TODO why is this repeated?

  initIMU(imuCalibration);
  initRemoteControl();
  initFilters();
  SimpleFOCDebug::enable(&Serial);  // TODO a more centralized way to enable debug sections would be nice
  initPositionSensors();
  initDrivers();
  configureWheelMotor(motor1, kLeftWheelMotor);
  configureWheelMotor(motor2, kRightWheelMotor);

  motor1.init();
  motor2.init();

  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  motor1.target = 0;
  motor2.target = 0;

  motor1.useMonitoring(Serial);  // TODO logger lib usable here?
  motor1.monitor_downsample = 10;


  Serial.println("Motor ready.");

  _delay(1000);
}

/**
 * @brief Re-maps a number from one range to another, using floating-point precision.
 *
 * @param x The number to map.
 * @param in_min The lower bound of the value's current range.
 * @param in_max The upper bound of the value's current range.
 * @param out_min The lower bound of the value's target range.
 * @param out_max The upper bound of the value's target range.
 * @return The mapped value.
 */
float mapf(long x, long in_min, long in_max, float out_min, float out_max) {
  long divisor = (in_max - in_min);
  if (divisor == 0) {
    return -1;  // AVR returns -1, SAM returns 0
  }
  return (x - in_min) * (out_max - out_min) / divisor + out_min;
}

constexpr long kSbusBodyHeightCenter = 992;

// Below the stick centre the height falls to the minimum, above it rises to maxHeightM.
float sbusBodyHeightM(long raw, float maxHeightM) {
  if (raw <= kSbusBodyHeightCenter)
    return mapf(raw, SBUS_CHANNEL_MIN, kSbusBodyHeightCenter, kBodyHeightMinM, kBodyHeightDefaultM);
  return mapf(raw, kSbusBodyHeightCenter + 1, SBUS_CHANNEL_MAX, kBodyHeightDefaultM, maxHeightM);
}

/**
 * @brief Maps ten raw S.BUS channels to the body commands and mode switches.
 *
 * Channel 1 drives the body height or the pitching, depending on `attitudeMode`.
 */
void applyRcChannels(const int16_t *ch) {
  movementSpeed = mapf(ch[2], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -15, 15);
  bodyTurn = -mapf(ch[3], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -11, 11);
  pidGainsMode = min(static_cast<int>(map(ch[4], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 2)),
                     REMOTE_CONTROL_PID_GAINS_MODE_ON);
  postureOrMarkMode = map(ch[5], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 1);
  attitudeMode = map(ch[7], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 2);
  // Top ball
  topBallX = mapf(ch[8], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -5, 5);
  topBallY = mapf(ch[9], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -5, 5);

  if (attitudeMode == REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT) {
    bodyHeightCommandM = sbusBodyHeightM(ch[1], kBodyHeightMaxM);
  } else if (attitudeMode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST) {
    bodyPitching = mapf(ch[1], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -12, 12);
  } else if (attitudeMode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) {
    bodyHeightCommandM = sbusBodyHeightM(ch[1], kBallPoiseMaxBodyHeightM);
  }
  bodyRoll = mapf(ch[0], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -0.011, 0.011);
}

/**
 * @brief Reads and processes data from the FUTABA S.BUS remote controller.
 *
 * This function decodes the S.BUS signal, maps the raw channel values to
 * meaningful control variables like `movementSpeed`, `bodyTurn`, `bodyHeightCommandM`,
 * and `bodyPitching`, and updates global state based on the RC switch positions.
 * While frames are missing for `kRcFrameTimeoutMs` or the receiver reports a
 * failsafe, `movementSpeed` and `bodyTurn` are forced to zero and every other
 * command keeps its last received value.
 */
void readSbus() {
  static uint32_t lastFrameMs = millis();

  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    const uint32_t frameMs = millis();
    sbusFrameIntervalMs = frameMs - lastFrameMs;
    lastFrameMs = frameMs;
    sBus.toChannels = 0;
    sBus.UpdateChannels();
    sBus.toChannels = 0;

    applyRcChannels(sBus.channels);
    if (DIAGNOSTIC_LIVE_TUNING_DEFAULTS)
      movementSpeed *= 2.0f;  // Keep drive-command authority near the previous 0.03 x 3.3 setting.
  }

  const bool lost =
      static_cast<uint32_t>(millis() - lastFrameMs) > kRcFrameTimeoutMs || sBus.Failsafe() != FUTABA_SBUS::kSignalOk;
  if (lost) {
    movementSpeed = 0;
    bodyTurn = 0;
  }
}

DebugTelemetry::PidStatus pidStatus(const MyPIDController &pid) {
  return {pid.Kp,   pid.Ki,   pid.Kd,   pid.deriv,  pid.integral, pid.iLimit,
          pid.outP, pid.outI, pid.outD, pid.output, pid.error};
}

/**
 * @brief Collects the sketch state read by the debug selections for DebugTelemetry.
 */
DebugTelemetry::TelemetryInputs collectTelemetryInputs(void) {
  DebugTelemetry::TelemetryInputs in{};
  in.selection = (int)debugLogSelector;
  in.nowMs = millis();
  in.controlGateSequence = controlGateSequence;
  in.balanceActive = isPidGainsModeEnabled(pidGainsMode) && robotTumble == ROBOT_NOT_TUMBLING;

  in.imu = imu::latest();
  in.imuTiming = imu::timingStatistics();
  in.imuSettings = imu::settings();
  in.imuCalibration = imu::calibration();
  in.controlTimestepSec = controlTimestepSec;

  in.m1VelocityRadPerSec = m1VelocityRadPerSec;
  in.m2VelocityRadPerSec = m2VelocityRadPerSec;
  in.m1FilteredVelocityRadPerSec = m1FilteredVelocityRadPerSec;
  in.m2FilteredVelocityRadPerSec = m2FilteredVelocityRadPerSec;
  in.motor1Target = motor1.target;
  in.motor2Target = motor2.target;
  in.motor2CurrentSetpoint = motor2.current_sp;
  in.motor1CurrentSetpoint = motor1.current_sp;
  in.motor1VoltageQ = motor1.voltage.q;
  in.motor2VoltageQ = motor2.voltage.q;
  in.motor1ShaftVelocity = motor1.shaft_velocity;
  in.motor2ShaftVelocity = motor2.shaft_velocity;
  in.sensor1Angle = sensor1.getAngle();
  in.sensor1MechanicalAngle = sensor1.getMechanicalAngle();
  in.sensor2Angle = sensor2.getAngle();
  in.sensor2MechanicalAngle = sensor2.getMechanicalAngle();

  for (int i = 0; i < 18; ++i) in.sbusChannels[i] = sBus.channels[i];
  in.sbusFrameIntervalMs = sbusFrameIntervalMs;

  in.anglePid = pidStatus(anglePid);
  in.yawPid = pidStatus(yawPid);

  in.servoTrims = servoTrims;
  for (int i = 0; i < 4; ++i) in.servoTraceAngle[i] = servoTraceAngle[i];

  in.topBallX = topBallX;
  in.topBallY = topBallY;
  in.sbusTopBallXSmoothed = sbusTopBallXSmoothed;
  in.sbusTopBallYSmoothed = sbusTopBallYSmoothed;

  in.bodyRoll = bodyRoll;
  in.bodyPitching = bodyPitching;
  in.bodyPitchingFiltered = bodyPitchingFiltered;
  in.bodyRollFiltered = bodyRollFiltered;
  in.bodyHeightCommandM = bodyHeightCommandM;
  in.bodyHeightCommandFilteredM = bodyHeightCommandFilteredM;
  in.bodyTurn = bodyTurn;
  in.bodyCommandFilteringEnabled = bodyCommandFilteringEnabled;
  in.bodyCommandFilterCutoffHz = bodyCommandFilterCutoffHz;

  in.movementSpeed = movementSpeed;

  in.voltageAdc = voltageAdc;
  in.voltageAdcFiltered = voltageAdcFiltered;
  in.voltage = batteryVoltage;
  in.batteryInstantRawV = board::kBatteryVoltsPerAdcCount * voltageAdc;
  in.batteryInstantFilteredV = board::kBatteryVoltsPerAdcCount * voltageAdcFiltered;
  in.controlStartUs = controlStartUs;
  in.controlExecutionUs = controlExecutionUs;
  in.maxLoopEntryIntervalUs = debugTelemetryState.maxLoopEntryIntervalUs;

  in.pidGainsMode = pidGainsMode;
  in.postureOrMarkMode = postureOrMarkMode;
  in.robotTumble = robotTumble;
  in.pidParameterTuning = pidParameterTuning;
  in.targetBodyHeightOverrideM = targetBodyHeightOverrideM;
  return in;
}

void applyPidPreset(PIDController &pid, const PidPreset &preset) {
  pid.P = preset.p;
  pid.I = preset.i;
  pid.D = preset.d;
  pid.limit = preset.limit;
}

void applyBalancePidPreset(const BalancePidPreset &preset) {
  applyPidPreset(anglePidSettings, preset.angle);
}

/**
 * @brief Sets the PID gains for the main controllers based on the S.BUS switch `pidGainsMode`.
 *
 * Applies the balance gains while the remote controller's PID switch is on.
 */
void applyPidParameters(void) {
#if DIAGNOSTIC_LIVE_TUNING_DEFAULTS
  const int gainMode = REMOTE_CONTROL_PID_GAINS_MODE_ON;
#else
  const int gainMode = pidGainsMode;
#endif
  if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON) applyBalancePidPreset(kBalancePid);

  applyPidPreset(yawPidSettings, kYawPid);
}


/**
 * @brief Main PID control loop for two-wheel balancing and posture control.
 *
 * This is the core control function for the two-wheeled mode. It orchestrates
 * multiple PID controllers to achieve stable balancing and respond to commands.
 * - It applies the S.BUS top-ball input to `bodyPitching` and the roll command.
 * - It runs a cascade PID for speed and balance control (Speed loop -> Angle loop).
 * - It runs a PID loop for yaw (turning) control.
 * The outputs are summed to produce the final `target` values for each motor.
 *
 * @param dt The time delta since the last call, in seconds.
 */
void runPostureController(float dt) {
  if ((int)pidParameterTuning == 0) applyPidParameters();

  // S.BUS top-ball input: pitch in top-ball mode.
  if (attitudeMode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) {
    bodyPitching = -sbusTopBallXSmoothed;
  }

  // Balance loop
  anglePid.Kp = anglePidSettings.P;
  anglePid.Ki = anglePidSettings.I;
  anglePid.Kd = anglePidSettings.D;
  anglePid.iLimit = anglePidSettings.limit;  // Integral limit

  float angleError = imu::latest().correctedRollDegrees - (-bodyPitchingFiltered);  // Measured value minus target value
  if (balancePidNeedsPriming) {
    // Avoid a derivative kick when CH5 first enables the balance loop.
    anglePid.previousError = angleError;
    anglePid.integral = 0;
    balancePidNeedsPriming = false;
  }
  float angleOutput = anglePid.compute(angleError, dt);

  // Yaw loop
  yawPid.Kp = yawPidSettings.P;
  yawPid.Ki = yawPidSettings.I;
  yawPid.Kd = yawPidSettings.D;
  yawPid.iLimit = yawPidSettings.limit;
  yawPid.Ki = 0;  // Heading lock: the yaw loop has no integral term.
  yawPid.integral = 0;

  float yawError = imu::latest().angularVelocityRadPerSec.z - bodyTurn;  // Measured value minus target value
  float yawOutput = yawPid.compute(yawError, dt);

  float target1 = angleOutput - yawOutput;
  float target2 = angleOutput + yawOutput;

  if (controlTorqueCompensation != 0) {
    if (target1 > 0)
      target1 = target1 + controlTorqueCompensation;
    else if (target1 < 0)
      target1 = target1 + (-controlTorqueCompensation);

    if (target2 > 0)
      target2 = target2 + controlTorqueCompensation;
    else if (target2 < 0)
      target2 = target2 + (-controlTorqueCompensation);
  }

  // The wheel motors now take ampere targets, not the rad/s targets computed above. Until the new controller
  // (package 4) produces current commands, balance holds both wheels at 0 A.
  (void)target1;
  (void)target2;
  motor1.target = 0.0f;
  motor2.target = 0.0f;
}

/**
 * @brief Applies a low-pass filter to the remote control input values.
 *
 * This function smooths the raw values received from the S.BUS controller
 * (`bodyPitching`, `bodyRoll`, `bodyHeightCommandM`, etc.) using biquad low-pass filters.
 * This prevents jerky movements and improves the stability of the robot's response
 * to user commands. The filter cutoff frequency can be adjusted live.
 */
void filterRemoteControl(void)  // Remote control filter
{
  static int bodyCommandFilteringEnabledLast = (int)bodyCommandFilteringEnabled;
  static int bodyCommandFilterCutoffHzLast = (int)bodyCommandFilterCutoffHz;

  sbusTopBallXSmoothed = biquadFilterApply(&bodyCommandFilterLpf[8], topBallX);
  sbusTopBallYSmoothed = biquadFilterApply(&bodyCommandFilterLpf[9], topBallY);

  if ((int)bodyCommandFilteringEnabled == 1) {
    bodyPitchingFiltered = biquadFilterApply(&bodyCommandFilterLpf[0], bodyPitching);
    bodyRollFiltered = biquadFilterApply(&bodyCommandFilterLpf[1], bodyRoll);
    bodyHeightCommandFilteredM =
        biquadFilterApply(&bodyCommandFilterLpf[kBodyHeightCommandFilterIndex], bodyHeightCommandM);
  } else {
    bodyPitchingFiltered = bodyPitching;
    bodyRollFiltered = bodyRoll;
    bodyHeightCommandFilteredM = bodyHeightCommandM;
  }

  if (((int)bodyCommandFilteringEnabled != bodyCommandFilteringEnabledLast) ||
      ((int)bodyCommandFilterCutoffHz != bodyCommandFilterCutoffHzLast)) {
    for (int i = 0; i < 6; i++) {
      biquadFilterInitLPF(&bodyCommandFilterLpf[i], (unsigned int)bodyCommandFilterCutoffHz, 1000);
    }

    bodyCommandFilteringEnabledLast = (int)bodyCommandFilteringEnabled;
    bodyCommandFilterCutoffHzLast = (int)bodyCommandFilterCutoffHz;
    Logging::message(Logging::Level::Info, "Filter", "ok");
  }
}

/**
 * @brief Reads and filters the battery voltage.
 *
 * Reads the analog value from the voltage divider, applies a low-pass filter
 * to get a stable reading, and converts it to the actual voltage.
 */
void readBatteryVoltage(void) {
  voltageAdc = analogRead(board::kBatteryAdcPin);
  if (debugTelemetryState.voltageAdcMin == 0 || voltageAdc < debugTelemetryState.voltageAdcMin)
    debugTelemetryState.voltageAdcMin = voltageAdc;
  voltageAdcFiltered = biquadFilterApply(&voltageFilterLpf, voltageAdc);
  batteryVoltage = board::kBatteryVoltsPerAdcCount * voltageAdcFiltered;
}

/**
 * @brief Detects if the robot has fallen over.
 *
 * If the absolute roll angle exceeds a threshold (35 degrees) for a certain
 * number of consecutive cycles, it sets the `robotTumble` flag to 1. The flag
 * is reset to 0 only after the robot is brought back to a near-level position.
 * This is a safety feature to disable motors upon falling.
 */
void updateTumbleState(void) {
  static int x = 0;
  if (fabsf(imu::latest().correctedRollDegrees) >= 35) {
    x++;
    if (x >= 20) {
      x = 20;
      robotTumble = ROBOT_TUMBLING;  // Machine fall
    }
  } else {
    if ((robotTumble == ROBOT_TUMBLING) && (fabsf(imu::latest().correctedRollDegrees) <= 5))  // Machine fall after fall
    {
      x--;
      if (x <= 0) {
        x = 0;
        robotTumble = ROBOT_NOT_TUMBLING;
      }
    }
  }
}

void diagnosticLoop(void) {
  static uint32_t lastLogScheduleUs = 0;
  static uint32_t lastTimingReportUs = 0;
  static uint32_t lastTimingAcceptedSamples = 0;
  static bool timingReportStarted = false;
  static unsigned long lastVoltageMs = 0;
  const unsigned long nowMs = millis();

  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    diagnosticLastRcFrameMs = nowMs;
    diagnosticHasRcFrame = true;
  }

  if (diagnosticImuReady && imu::service()) {
    const imu::Snapshot &diagnosticImu = imu::latest();
    const uint32_t imuSampleUs = diagnosticImu.readTimeUs;
    if ((uint32_t)(imuSampleUs - lastLogScheduleUs) >= DIAGNOSTIC_LOG_INTERVAL_US) {
      lastLogScheduleUs = imuSampleUs;
      const long rcAgeMs = diagnosticHasRcFrame ? (long)(nowMs - diagnosticLastRcFrameMs) : -1;
      const int rcFailsafe = diagnosticHasRcFrame ? sBus.Failsafe() : -1;
      const float batteryRawV = board::kBatteryVoltsPerAdcCount * voltageAdc;
      Logging::DiagnosticSample sample{};
      sample.timestampUs = imuSampleUs;
      sample.sequence = diagnosticSequence++;
      sample.gyroX = diagnosticImu.angularVelocityRadPerSec.x;
      sample.gyroY = diagnosticImu.angularVelocityRadPerSec.y;
      sample.gyroZ = diagnosticImu.angularVelocityRadPerSec.z;
      sample.accelX = diagnosticImu.accelerationG.x;
      sample.accelY = diagnosticImu.accelerationG.y;
      sample.accelZ = diagnosticImu.accelerationG.z;
      sample.rollDeg = diagnosticImu.attitudeDegrees.roll;
      sample.pitchDeg = diagnosticImu.attitudeDegrees.pitch;
      sample.yawDeg = diagnosticImu.attitudeDegrees.yaw;
      sample.batteryRawV = batteryRawV;
      sample.receiverFrameAgeMs = rcAgeMs;
      sample.receiverFailsafe = rcFailsafe;
      sample.imuReady = diagnosticImuReady ? 1 : 0;
      Logging::submit(sample);
    }
  }

  const uint32_t timingNowUs = micros();
  const imu::TimingStatistics timing = imu::timingStatistics();
  if (!timingReportStarted) {
    lastTimingReportUs = timingNowUs;
    lastTimingAcceptedSamples = timing.acceptedSamples;
    timingReportStarted = true;
  } else {
    const uint32_t elapsedUs = timingNowUs - lastTimingReportUs;
    if (elapsedUs >= 1000000U) {
      const uint32_t acceptedSamples = timing.acceptedSamples - lastTimingAcceptedSamples;
      const float acceptedRateHz = static_cast<float>(acceptedSamples) * 1000000.0f / static_cast<float>(elapsedUs);
      Serial.printf("IMU_TIMING,elapsed_us=%lu,accepted=%lu,rate_hz=%.1f,long_gap_total=%lu,max_interval_us=%lu\n",
                    static_cast<unsigned long>(elapsedUs), static_cast<unsigned long>(acceptedSamples), acceptedRateHz,
                    static_cast<unsigned long>(timing.longGapCount), static_cast<unsigned long>(timing.maxIntervalUs));
      lastTimingReportUs = timingNowUs;
      lastTimingAcceptedSamples = timing.acceptedSamples;
    }
  }

  if (nowMs - lastVoltageMs >= 10) {
    lastVoltageMs = nowMs;
    readBatteryVoltage();
  }
}
void updateMotorsAndCommands() {
  // iterative function setting the outer loop target
  if (motorDiagnosticState.active) readSbus();
  serviceMotorDiagnostic();
  if (!motorDiagnosticState.active) {
    if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO) {
      if (Logging::profile() == Logging::Profile::Idle) motor1.monitor();
    } else if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS) {
      motor2.target = motor1.target;
    }
  }

  motor1.move();
  motor2.move();

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // user communication
  serialCommandRouter.run(commandStream, command, commandEntries);
  // Apply STOP, mode conflicts, and lease checks immediately after dispatch.
  // This also prevents a newly selected blocking calibration from running while leased.
  // Skipped on the normal path: no diagnostic is active or still applied to the motors.
  if (motorDiagnosticState.active || motorDiagnosticApplied) {
    readSbus();
    serviceMotorDiagnostic();
  }
}

/**
 * @return True when the IMU accepted a new sample in this call.
 */
bool readInputs() {
  const bool imuUpdated = imu::service();  // Poll once per loop without gating motor control
  readSbus();                              // S.BUS remote control input
  return imuUpdated;
}

void updateStatusLed() {
  statusLedTicks++;
  if (statusLedTicks >= statusLedPeriodTicks) {
    statusLedTicks = 0;
    if (statusLedOn == 1) {
      digitalWrite(board::kStatusLedPin, LOW);  // On
      statusLedOn = 0;
    } else {
      digitalWrite(board::kStatusLedPin, HIGH);  // Off
      statusLedOn = 1;
    }
  }

  static uint32_t lastLowBatteryWarningMs = millis();
  if (batteryVoltage <= board::kLowBatteryVolts) {
    statusLedPeriodTicks = 20;
    if (static_cast<uint32_t>(millis() - lastLowBatteryWarningMs) >= 1000) {
      lastLowBatteryWarningMs = millis();
      Logging::message(Logging::Level::Warning, "Voltage", "%.5f", batteryVoltage);
    }
  } else {
    statusLedPeriodTicks = 100;
  }
}

void runCalibrationStep(bool imuUpdated) {
  imu::CalibrationValues activeCalibration = imu::calibration();
  const imu::Snapshot &acceptedImu = imu::latest();
  CalibrationResult calibrationResult{CalibrationCompletion::None, false, {false, false, false, false}};
  if (static_cast<int>(calibrationSelect) == 1 && isPidGainsModeEnabled(pidGainsMode) &&
      robotTumble == ROBOT_NOT_TUMBLING) {
    calibrationSelect = 0;
    Logging::message(Logging::Level::Warning, "Calibration", "gyro calibration refused: balance active");
  }
  if (imuUpdated)
    calibrationResult = calibration.update(
        static_cast<int>(calibrationSelect), {acceptedImu.attitudeDegrees.roll, acceptedImu.attitudeDegrees.pitch},
        calibrationStore, {activeCalibration.attitude.rollDegrees, activeCalibration.attitude.pitchDegrees},
        {activeCalibration.gyro.xRawCounts, activeCalibration.gyro.yRawCounts, activeCalibration.gyro.zRawCounts},
        {servoTrims.servo1Degrees, servoTrims.servo2Degrees, servoTrims.servo3Degrees, servoTrims.servo4Degrees},
        [](GyroBiasRawCountRefs stagedBias) {
          imu::calibrateGyroBiasBlocking();
          const GyroBiasRawCounts measured = imu::calibration().gyro;
          stagedBias.xRawCounts = measured.xRawCounts;
          stagedBias.yRawCounts = measured.yRawCounts;
          stagedBias.zRawCounts = measured.zRawCounts;
        });
  if (imuUpdated) imu::setCalibration(activeCalibration);
  if (calibrationResult.clearRequest) calibrationSelect = 0;

  if (calibrationResult.completion == CalibrationCompletion::Gyroscope) {
    Logging::message(Logging::Level::Info, "Calibration", "gyroBiasX: %.2f, gyroBiasY: %.2f, gyroBiasZ: %.2f",
                     activeCalibration.gyro.xRawCounts, activeCalibration.gyro.yRawCounts,
                     activeCalibration.gyro.zRawCounts);
  } else if (calibrationResult.completion == CalibrationCompletion::Attitude) {
    Logging::message(Logging::Level::Info, "Calibration", "Roll Zero Bias: %.2f, Pitch Zero Bias: %.2f",
                     activeCalibration.attitude.rollDegrees, activeCalibration.attitude.pitchDegrees);
  }

  if (calibrationResult.changedServos.servo1Changed)
    Logging::message(Logging::Level::Info, "Calibration", "imuAndServoOffsets.servo1: %.2f", servoTrims.servo1Degrees);
  if (calibrationResult.changedServos.servo2Changed)
    Logging::message(Logging::Level::Info, "Calibration", "imuAndServoOffsets.servo2: %.2f", servoTrims.servo2Degrees);
  if (calibrationResult.changedServos.servo3Changed)
    Logging::message(Logging::Level::Info, "Calibration", "imuAndServoOffsets.servo3: %.2f", servoTrims.servo3Degrees);
  if (calibrationResult.changedServos.servo4Changed)
    Logging::message(Logging::Level::Info, "Calibration", "imuAndServoOffsets.servo4: %.2f", servoTrims.servo4Degrees);
}

void updateWheelVelocities() {
  // shaftAngle() applies each sensor_direction, so the angles share the sign of positive voltage.q.
  const uint32_t readTimeUs = micros();
  WheelBoundary::WheelValues rawForward;
  if (!wheelSpeedEstimator.update(motor1.shaftAngle(), motor2.shaftAngle(), readTimeUs, rawForward)) return;
  m1VelocityRadPerSec = rawForward.left;
  m2VelocityRadPerSec = rawForward.right;
  m1FilteredVelocityRadPerSec = m1VelocityFilter(m1VelocityRadPerSec);
  m2FilteredVelocityRadPerSec = m2VelocityFilter(m2VelocityRadPerSec);
}

/**
 * @brief Runs the balance controller when enabled and returns the body height and roll offset.
 *
 * With balance off or the robot tumbling it zeroes the wheel targets, the body offsets and the
 * controller integrals instead.
 */
BodyPose computeBodyPose() {
  BodyPose pose{kBodyHeightDefaultM, bodyRollFiltered};

  if ((pidGainsMode == REMOTE_CONTROL_PID_GAINS_MODE_OFF) || (robotTumble == ROBOT_TUMBLING)) {
    balancePidNeedsPriming = true;
    if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE && !motorDiagnosticState.active) {
      motor1.target = 0;
      motor2.target = 0;
    }

    pose.heightM = kBodyHeightDefaultM;
    pose.roll = 0;
    bodyPitchingFiltered = 0;

    anglePid.integral = 0;
    yawPid.integral = 0;

  } else if ((isPidGainsModeEnabled(pidGainsMode)) && (robotTumble == ROBOT_NOT_TUMBLING)) {
    runPostureController(controlTimestepSec);  // PID controller

    if (targetBodyHeightOverrideM == kBodyHeightOverrideDisabledM)
      pose.heightM = bodyHeightCommandFilteredM;
    else
      pose.heightM = targetBodyHeightOverrideM;
  }

  if (robotTumble == ROBOT_TUMBLING)  // Machine fall
  {
    pose.heightM = kBodyHeightDefaultM;
    pose.roll = 0;
    bodyPitchingFiltered = 0;
  }

  return pose;
}

void warnIfLegUnreachable(const LegSolveResult &leg, const char *tag, uint32_t &lastWarningMs) {
  if (leg.status != LegSolveStatus::Success && static_cast<uint32_t>(millis() - lastWarningMs) >= 1000) {
    lastWarningMs = millis();
    Logging::message(Logging::Level::Warning, tag, "no");
  }
}

void writeLegServos() {
  servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1,
                              servoTraceAngle[3], 1);
}

void writeServos(const LegSolveResult &right, const LegSolveResult &left) {
  if (postureOrMarkMode == REMOTE_CONTROL_PM_POSTURE_MODE)  // Posture
  {
    // An unreachable leg keeps its previous servo angles.
    if (left.status == LegSolveStatus::Success) {
      servoTraceAngle[0] = left.angles.joint1Degrees - servoTrims.servo1Degrees;
      servoTraceAngle[1] = left.angles.joint2Degrees - servoTrims.servo2Degrees;
    }
    if (right.status == LegSolveStatus::Success) {
      servoTraceAngle[2] = right.angles.joint1Degrees - servoTrims.servo3Degrees;
      servoTraceAngle[3] = right.angles.joint2Degrees - servoTrims.servo4Degrees;
    }
    writeLegServos();
  } else  // Assembly position and calibration
  {
    if ((int)calibrationSelect == 3)  // Servo calibration
    {
      servoTraceAngle[0] = -servoTrims.servo1Degrees;
      servoTraceAngle[1] = -servoTrims.servo2Degrees;
      servoTraceAngle[2] = -servoTrims.servo3Degrees;
      servoTraceAngle[3] = -servoTrims.servo4Degrees;
    } else {
      servoTraceAngle[0] = servoTraceAngle[1] = servoTraceAngle[2] = servoTraceAngle[3] = 0;
    }
    writeLegServos();
  }

  RobotLogCapture::observeServoAngles(servoTraceAngle);
}

void runControlTick(bool imuUpdated) {
  controlStartUs = micros();
  controlGateSequence++;

  filterRemoteControl();                                                  // Remote control signal filtering
  readBatteryVoltage();                                                   // Battery

  updateTumbleState();  // Machine fall detection
  updateStatusLed();
  serviceMotorDiagnostic();
  runCalibrationStep(imuUpdated);
  updateWheelVelocities();
  serviceWheelOverspeed();

  const BodyPose pose = computeBodyPose();

  static uint32_t lastRightLegWarningMs = 0;
  static uint32_t lastLeftLegWarningMs = 0;
  const LegSolveResult rightLeg = LegKinematics::solveRight({0.0f, pose.heightM - pose.roll, bodyPitchingFiltered});
  warnIfLegUnreachable(rightLeg, "RightInverseKinematics", lastRightLegWarningMs);

  const LegSolveResult leftLeg = LegKinematics::solveLeft({0.0f, pose.heightM + pose.roll, bodyPitchingFiltered});
  warnIfLegUnreachable(leftLeg, "LeftInverseKinematics", lastLeftLegWarningMs);

  writeServos(rightLeg, leftLeg);

  controlExecutionUs = static_cast<uint32_t>(micros() - controlStartUs);
  DebugTelemetry::submit(collectTelemetryInputs(), debugTelemetryState);  // Serial port data printing

  previousControlTimeUs = loopTimeUs;
}

/**
 * @brief The main execution loop of the program.
 *
 * This function runs repeatedly after `setup()` is complete:
 * 1. `updateMotorsAndCommands`: SimpleFOC `move()` and `loopFOC()` for both wheels, then Commander input.
 * 2. `readInputs`: IMU sample and S.BUS commands.
 * 3. `runControlTick`, once at least 1 ms has passed since the previous tick: input filtering,
 *    battery reading, telemetry, fall detection, status LED, calibration, wheel velocities,
 *    posture control, leg kinematics and servo output.
 */
void loop() {
#if SENSOR_DIAGNOSTIC_MODE
  diagnosticLoop();
  return;
#endif
  loopTimeUs = micros();
  if (hasPreviousLoopEntry) {
    const uint32_t intervalUs = static_cast<uint32_t>(loopTimeUs - previousLoopEntryUs);
    if (intervalUs > debugTelemetryState.maxLoopEntryIntervalUs)
      debugTelemetryState.maxLoopEntryIntervalUs = intervalUs;
  }
  previousLoopEntryUs = loopTimeUs;
  hasPreviousLoopEntry = true;

  updateMotorsAndCommands();
  const bool imuUpdated = readInputs();

  controlTimestepSec = (loopTimeUs - previousControlTimeUs) / 1000000.0f;
  if (controlTimestepSec >= kControlTimestepMinSec) {
    runControlTick(imuUpdated);
  }
}
