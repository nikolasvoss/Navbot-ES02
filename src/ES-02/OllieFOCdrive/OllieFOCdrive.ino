#include <Arduino.h>
#include <SimpleFOC.h>
#include <esp_system.h>
#include <esp_timer.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "BalanceController.h"
#include "BoardConfig.h"
#include "Calibration.h"
#include "CalibrationStore.h"
#include "DebugTelemetry.h"
#include "FUTABA_SBUS.h"
#include "Logging.h"
#include "MathConstants.h"
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
// Safe bring-up: read sensors and receiver without energizing wheels or servos.
// Set to 0 only after the diagnostic readings and power supply are checked.
#define SENSOR_DIAGNOSTIC_MODE 0

#define SBUS_CHANNEL_MAX 1792
#define SBUS_CHANNEL_MIN 192
constexpr uint32_t kRcFrameTimeoutMs = 100;
constexpr float kControlTimestepMinSec = 0.001f;

constexpr unsigned long kSerialBaudRate = 576000;
constexpr size_t kSerialTxBufferBytes = 1024;
#define DIAGNOSTIC_LOG_INTERVAL_US 5000  // Diagnostic CSV rows are emitted at 200 Hz.
constexpr unsigned int kDiagnosticLogSampleRateHz = 1000000U / DIAGNOSTIC_LOG_INTERVAL_US;
// -------------------------------------

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

// Balance configuration, built once. Gains are zero, so the demand is zero. The envelope can still command a small
// braking current above roughly 50 rad/s, because the plan requires outputs inside the envelope.
// Each value is one of: existing constant (named), plan (decision or stated number), or an uncommissioned placeholder.
const Config &balanceConfig() {
  static const Config config = [] {
    Config c;
    // Arming
    c.armingTimeoutUs = 3000000;                 // uncommissioned: proposed starting value, 3 s
    c.armingTiltWindowRad = 0.0872665f;          // uncommissioned: proposed starting value, rad (5 deg)
    c.armingTiltRateRadPerSec = 0.1f;            // uncommissioned: proposed starting value, rad/s
    c.armingRequestSpeedMps = 0.05f;             // uncommissioned: proposed starting value, m/s
    c.armingRequestYawRadPerSec = 0.05f;         // uncommissioned: proposed starting value, rad/s
    c.armingWheelSpeedRadPerSec = 2.0f;          // uncommissioned: proposed starting value, rad/s per wheel

    // Measurement freshness and loop gap
    c.imuMaxAgeUs = 20000;                       // uncommissioned: proposed starting value, 20 ms
    c.wheelMaxAgeUs = 20000;                     // uncommissioned: proposed starting value, 20 ms
    c.busMaxAgeUs = 100000;                      // uncommissioned: proposed starting value, 100 ms
    c.dtMaxUs = 20000;                           // uncommissioned: proposed starting value, 20 ms

    // Fall: the 35 deg tumble threshold of the former fall detector (existing constant)
    c.fallAngleRad = 35.0f * kDegreesToRadiansF;  // existing tumble threshold, rad
    c.fallDurationUs = 20000;                    // upstream: 20 control cycles at the nominal 1 kHz gate, 20 ms

    // Overspeed: the existing OverspeedGuard limit and debounce
    c.wheelOverspeedRadPerSec = OverspeedGuard::kLimitRadPerSec;   // existing constant, rad/s
    c.overspeedDurationUs = OverspeedGuard::kDebounceMs * 1000U;   // existing constant, us

    // Effort envelope. One KV is used for both wheels: 76.7 (right) and 79.5 (left) measured, about 3.6 percent apart,
    // so the envelope error is small. The lower value is used.
    c.phaseResistanceOhm = kLeftWheelMotor.phaseResistanceOhm;     // existing constant, ohm per phase (5.75 not verified)
    c.kvRating = std::min(kLeftWheelMotor.kvRatingRpmPerV, kRightWheelMotor.kvRatingRpmPerV);  // existing constant, rpm/V
    c.voltageLimitV = kWheelVoltageLimitV;                         // existing constant, V
    c.currentLimitA = kWheelVoltageLimitV / c.phaseResistanceOhm;  // as configureWheelMotor(), A
    c.sagMarginV = 0.5f;                         // uncommissioned: proposed starting value, V
    c.modulationFactor = 0.57735027f;            // plan: SpaceVectorPWM usable amplitude is bus / sqrt(3)
    // Motor speed = sign * forward speed. Forward is negative motor-local on both wheels (WheelBoundary.h).
    c.leftWheelSign = static_cast<int>(WheelBoundary::kLeftMotorSign);
    c.rightWheelSign = static_cast<int>(WheelBoundary::kRightMotorSign);
    c.busMinV = 6.0f;                            // uncommissioned: proposed starting value, V
    c.busMaxV = 9.0f;                            // uncommissioned: proposed starting value, V

    // Demand shaping
    c.speedCapMps = 0.3f;                        // uncommissioned: proposed starting value, m/s
    c.accelCapMps2 = 0.5f;                       // uncommissioned: proposed starting value, m/s^2
    c.brakeCapMps2 = 1.0f;                       // uncommissioned: proposed starting value, m/s^2
    c.yawRateCapRadPerSec = 0.5f;                // uncommissioned: proposed starting value, rad/s
    c.requestMaxAgeUs = kRcFrameTimeoutMs * 1000U;  // existing constant (RC frame timeout), us
    c.demandScaleCapStart = 0.5f;                // uncommissioned: proposed starting value, dimensionless

    // Speed loop (gains zero, integral off)
    c.wheelRadiusM = 0.03f;                      // uncommissioned: NOT MEASURED placeholder, m
    c.speedLoopPeriodUs = 20000;                 // plan: provisional 50 Hz speed loop, us
    c.speedKpRadPerMps = 0.0f;                   // passive: zero gain
    c.speedKiRadPerMpsS = 0.0f;                  // passive: zero gain
    c.speedIntegralEnabled = false;              // passive: integral off
    c.tiltRefLimitRad = 0.0872665f;              // uncommissioned: proposed starting value, rad (5 deg)
    c.tiltRefSlewRadPerSec = 0.5f;               // uncommissioned: proposed starting value, rad/s
    c.tiltClipDurationUs = 500000;               // uncommissioned: proposed starting value, 0.5 s

    // Trim and standstill commissioning
    c.trimEnvelopeRad = 0.0523599f;              // uncommissioned: proposed starting value, rad (3 deg)
    c.trimSlewRadPerSec = 0.05f;                 // uncommissioned: proposed starting value, rad/s
    c.knobPickupBandRad = 0.01f;                 // uncommissioned: proposed starting value, rad
    c.standstillWheelSpeedTolMps = 0.02f;        // plan: average standstill wheel speed below 0.02 m/s
    c.standstillTiltErrorTolRad = 0.01f;         // uncommissioned: proposed starting value, rad
    c.transferToleranceRad = 0.001f;             // uncommissioned: proposed starting value, rad
    c.saveRequestSpeedTolMps = 0.02f;            // uncommissioned: proposed starting value, m/s
    c.saveRequestYawTolRadPerSec = 0.05f;        // uncommissioned: proposed starting value, rad/s

    // Balance and yaw law (passive: zero)
    c.tiltPAPerRad = 0.0f;                       // passive: zero gain, A/rad
    c.gyroDampingAPerRadPerSec = 0.0f;           // passive: zero gain, A/(rad/s)
    c.yawPAPerRadPerSec = 0.0f;                  // passive: zero gain, A/(rad/s)

    // Reserve and authority
    c.reserveLowA = 0.2f;                        // uncommissioned: proposed starting value, A
    c.authorityDurationUs = 2000000;             // uncommissioned: proposed starting value, 2 s

    // Sketch-supplied inputs: first commissioning, trim taken as validated at zero (user decision 2026-10-22)
    c.trimValidated = true;
    c.storedTrimDeg = 0.0f;                      // deg
    c.fixedPoseReachable = true;                 // plan: fixed pose, all joint angles 0
    return c;
  }();
  return config;
}

void configureWheelMotor(BLDCMotor &motor, const WheelMotorParams &params);


// Body
float movementSpeed = 0;                                         // Movement speed
float bodyTurn = 0;                                              // Turning
int robotTumble = ROBOT_NOT_TUMBLING;  // Fall-detection state, see docs/robot/ROBOT_ROADMAP.md

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

// Servo zero position offsets
void zeroBiasServo1(char *cmd) { command.scalar(&servoTrims.servo1Degrees, cmd); }
void zeroBiasServo2(char *cmd) { command.scalar(&servoTrims.servo2Degrees, cmd); }
void zeroBiasServo3(char *cmd) { command.scalar(&servoTrims.servo3Degrees, cmd); }
void zeroBiasServo4(char *cmd) { command.scalar(&servoTrims.servo4Degrees, cmd); }

//  Create ServoControl object
ServoControl servoControl(board::kLegServoPin[0], board::kLegServoPin[1], board::kLegServoPin[2],
                          board::kLegServoPin[3]);
int servoTraceAngle[4] = {0, 0, 0, 0};  // Last angle arguments sent to the four servos

// Remote control
FUTABA_SBUS sBus;
int sbusFrameIntervalMs = 0;
int pidGainsMode = REMOTE_CONTROL_PID_GAINS_MODE_OFF;  // telemetry: 1 while the CH5 balance enable is on
// Raw S.BUS values (SBUS_CHANNEL_MIN 192 to SBUS_CHANNEL_MAX 1792, centre 992). Decided and measured 2026-10-22 (plan: RC map).
constexpr int kRcCenterRaw = (SBUS_CHANNEL_MAX + SBUS_CHANNEL_MIN) / 2;     // 992
constexpr int kRcHalfRangeRaw = (SBUS_CHANNEL_MAX - SBUS_CHANNEL_MIN) / 2;  // 800
constexpr int kRcEnableOnRaw = 592;   // CH5: 0 below, 1 and 2 at or above; 3-position switch
constexpr int kRcSwitchOnRaw = 992;   // CH6 and CH7: 2-position latching switches
constexpr int kRcKnobMinRaw = 172;    // CH9 trim knob: outside 172 to 1811 the knob is invalid
constexpr int kRcKnobMaxRaw = 1811;
int16_t rcChannels[18] = {};          // last S.BUS frame, raw
bool rcFrameSeen = false;
uint32_t rcFrameUs = 0;               // micros() when the last frame was decoded

// Balance controller and the sketch inputs it reads
BalanceController balanceController;
Output balanceOutput{};               // result of the last update()
// Balance measurement timestamps and validity, set by the acquisition functions
uint32_t wheelReadTimeUs = 0;
bool wheelsValid = false;
uint32_t busReadTimeUs = 0;

bool balanceActive() { return balanceOutput.state != State::Disabled; }

//  Create PID controller instance
float debugLogSelector = 0;   // Selects debug data and the log profile.
float calibrationSelect = 0;  // Save calibration data 0: Calibration end  1: Calibrate gyroscope  2: Calibrate Euler
                              // angle  3: Calibrate servo

PIDController yawPidSettings(4, 0, 0, 0, 0);


float pidTimestepSec = 0.01;

MyPIDController yawPid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);


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

// The only owner of wheel enable state. motor.enabled follows SimpleFOC enable()/disable(), and move() and loopFOC()
// return early while it is 0. Disabling also zeroes the output, so no stale target survives a disable.
void setWheelEnabled(BLDCMotor &motor, bool enabled) {
  if (enabled) {
    if (!motor.enabled) motor.enable();
    return;
  }
  if (motor.enabled) motor.disable();
  zeroMotorOutput(motor);
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
    setWheelEnabled(motor1, true);
    setWheelEnabled(motor2, true);
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
      setWheelEnabled(motor1, true);
      setWheelEnabled(motor2, false);
    } else {
      motor1.target = 0.0f;
      motor2.target = request.voltageV;
      setWheelEnabled(motor1, false);
      setWheelEnabled(motor2, true);
    }
  }
  motorDiagnosticApplied = true;
  motorDiagnosticOwnsOutput = true;
}

OverspeedGuard::State wheelOverspeed{};

// Coast fault: 0 A and both drivers off. It repeats every tick while tripped, so no other path can re-enable them.
void serviceWheelOverspeed() {
  const bool wasTripped = wheelOverspeed.tripped;
  if (!OverspeedGuard::update(wheelOverspeed, millis(), m1VelocityRadPerSec, m2VelocityRadPerSec)) return;
  motor1.target = 0.0f;
  motor2.target = 0.0f;
  setWheelEnabled(motor1, false);
  setWheelEnabled(motor2, false);
  if (!wasTripped)
    Logging::message(Logging::Level::Warning, "WHEEL_OVERSPEED", "peak=%.1frad_s limit=%.1frad_s; both drivers disabled",
                     wheelOverspeed.peakRadPerSec, OverspeedGuard::kLimitRadPerSec);
}

// Called only after a successful diagnostic RESET. Rearms the guard only while both wheels are below the limit.
// Rearming does not enable the drivers. balanceRequest() reports enable off while the guard is tripped, so CH5 must
// go off and on again before the controller drives the wheels.
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
    if (MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs())) {
      resetWheelOverspeed();
      Logging::message(Logging::Level::Info, "MOTOR_TEST", "fault reset; outputs remain disabled");
    } else
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
    if (MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs())) {
      resetWheelOverspeed();
      Logging::message(Logging::Level::Info, "MOTOR_TEST", "fault reset; outputs remain disabled");
    } else
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
    // Disable before restoring any configuration or emitting a message. The wheels stay disabled after this:
    // the balance drive path re-enables them only when the controller drives.
    setWheelEnabled(motor1, false);
    setWheelEnabled(motor2, false);
    zeroMotorOutput(motor1);
    zeroMotorOutput(motor2);
    restoreMotorConfiguration(motor1, kLeftWheelMotor);
    restoreMotorConfiguration(motor2, kRightWheelMotor);
    motorDiagnosticApplied = false;
    motorDiagnosticOwnsOutput = false;
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
  if (motorDiagnosticState.active && !motorDiagnosticApplied) applyMotorDiagnosticRequest();
}

void readSbus();
void readBatteryVoltage(void);
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
    {'Y', setYawPid, "yawPidSettings"},
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


  Serial.println("Motor ready.");

  _delay(1000);
}

/**
 * @brief Stores a decoded S.BUS frame with its arrival time. balanceRequest() reads the channels every tick.
 *        The CH5 balance enable also sets the pidGainsMode telemetry field.
 */
void applyRcChannels(const int16_t *ch) {
  for (int i = 0; i < 18; ++i) rcChannels[i] = ch[i];
  rcFrameUs = micros();
  rcFrameSeen = true;
  pidGainsMode = rcChannels[4] >= kRcEnableOnRaw ? REMOTE_CONTROL_PID_GAINS_MODE_ON : REMOTE_CONTROL_PID_GAINS_MODE_OFF;
}

/**
 * @brief Reads and processes data from the FUTABA S.BUS remote controller.
 *
 * Decodes the S.BUS signal into `rcChannels`. Freshness and receiver failsafe are judged in balanceRequest().
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
  in.balanceActive = balanceActive();

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

  in.yawPid = pidStatus(yawPid);

  in.servoTrims = servoTrims;
  for (int i = 0; i < 4; ++i) in.servoTraceAngle[i] = servoTraceAngle[i];


  in.bodyTurn = bodyTurn;

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
  in.robotTumble = robotTumble;
  return in;
}


/**
 * @brief Reads and filters the battery voltage.
 *
 * Reads the analog value from the voltage divider, applies a low-pass filter
 * to get a stable reading, and converts it to the actual voltage.
 */
void readBatteryVoltage(void) {
  busReadTimeUs = micros();
  voltageAdc = analogRead(board::kBatteryAdcPin);
  if (debugTelemetryState.voltageAdcMin == 0 || voltageAdc < debugTelemetryState.voltageAdcMin)
    debugTelemetryState.voltageAdcMin = voltageAdc;
  voltageAdcFiltered = biquadFilterApply(&voltageFilterLpf, voltageAdc);
  batteryVoltage = board::kBatteryVoltsPerAdcCount * voltageAdcFiltered;
}

// Balance measurements, built at the hardware boundary. Sign and axis checked on the supported robot 2026-10-23:
// leaning forward lowers correctedRollDegrees and drives gyro x below 0, so both are negated to make a forward lean
// positive (positive gains then push the wheels forward). Turning left raises gyro z, which is the positive yaw rate.
constexpr float kTiltSign = -1.0f;
constexpr float kTiltRateSign = -1.0f;
constexpr float kYawRateSign = 1.0f;

/**
 * @brief Builds the controller measurements from the latest IMU sample, wheel speeds and bus voltage.
 *        Timestamps are the acquisition times; the caller takes nowUs after this call.
 */
Measurements balanceMeasurements() {
  const imu::Snapshot &imuSnapshot = imu::latest();
  Measurements m{};
  m.imuSequence = imuSnapshot.sequence;
  m.imuReadTimeUs = imuSnapshot.readTimeUs;
  m.imuValid = imuSnapshot.valid;
  m.tiltRad = kTiltSign * imuSnapshot.correctedRollDegrees * kDegreesToRadiansF;
  m.tiltRateRadPerSec = kTiltRateSign * imuSnapshot.angularVelocityRadPerSec.x;
  m.yawRateRadPerSec = kYawRateSign * imuSnapshot.angularVelocityRadPerSec.z;
  m.wheelReadTimeUs = wheelReadTimeUs;
  m.wheelsValid = wheelsValid;
  m.leftWheelRadPerSec = m1VelocityRadPerSec;
  m.rightWheelRadPerSec = m2VelocityRadPerSec;
  m.busReadTimeUs = busReadTimeUs;
  m.busVoltageV = batteryVoltage;
  // Approximation: near the usable voltage ceiling, or at the current limit of the last command.
  m.voltageClipped = fabsf(motor1.voltage.q) >= motor1.voltage_limit - 0.01f ||
                     fabsf(motor2.voltage.q) >= motor2.voltage_limit - 0.01f;
  m.currentClipped = fabsf(motor1.target) >= motor1.current_limit || fabsf(motor2.target) >= motor2.current_limit;
  return m;
}

// Stick position in -1..1 from a raw S.BUS value, centred on kRcCenterRaw.
float rcStickFraction(int16_t raw) {
  const float fraction = static_cast<float>(raw - kRcCenterRaw) / kRcHalfRangeRaw;
  return std::max(-1.0f, std::min(1.0f, fraction));
}

/**
 * @brief Builds the balance request from the latest S.BUS frame. The frame counts only while it is younger than
 *        kRcFrameTimeoutMs and the receiver reports no failsafe.
 */
Request balanceRequest(uint32_t nowUs) {
  const Config &config = balanceConfig();
  const bool fresh = rcFrameSeen && static_cast<uint32_t>(nowUs - rcFrameUs) <= kRcFrameTimeoutMs * 1000U;
  Request request{};
  request.receivedTimeUs = rcFrameUs;
  request.sourceValid = fresh && sBus.Failsafe() == FUTABA_SBUS::kSignalOk;
  request.speedMps = config.speedCapMps * rcStickFraction(rcChannels[2]);         // CH3: forward is positive
  request.yawRateRadPerSec = -config.yawRateCapRadPerSec * rcStickFraction(rcChannels[3]);  // CH4: left is positive
  // A guard trip reads as enable off, so the controller goes Disabled. The operator must switch CH5 off and on to rearm.
  request.enableSwitch = rcChannels[4] >= kRcEnableOnRaw && !wheelOverspeed.tripped;  // CH5
  request.saveSwitch = rcChannels[5] >= kRcSwitchOnRaw;                           // CH6
  request.commissioningSwitch = rcChannels[6] >= kRcSwitchOnRaw;                  // CH7
  request.knobValid = rcChannels[8] >= kRcKnobMinRaw && rcChannels[8] <= kRcKnobMaxRaw;  // CH9
  request.knobTrimDeg = rcStickFraction(rcChannels[8]) * config.trimEnvelopeRad * kRadiansToDegreesF;
  return request;
}

/**
 * @brief Logs state and fault changes, save pulses, and refusal edges. Never logs per tick.
 */
void logBalanceChange(const Output &out) {
  if (out.state != balanceOutput.state || out.fault != balanceOutput.fault)
    Logging::message(Logging::Level::Info, "BALANCE", "state=%d fault=%d commissioningRefused=%d",
                     static_cast<int>(out.state), static_cast<int>(out.fault), out.diag.commissioningRefused ? 1 : 0);
  if (out.saveTrimRequested)  // one-tick pulse
    Logging::message(Logging::Level::Info, "BALANCE", "save requested trim=%.3f deg; storage not wired", out.trimToSaveDeg);
  if (out.diag.saveRefused && !balanceOutput.diag.saveRefused)
    Logging::message(Logging::Level::Info, "BALANCE", "save refused; no pulse");
  if (out.diag.commissioningRefused && !balanceOutput.diag.commissioningRefused)
    Logging::message(Logging::Level::Info, "BALANCE", "commissioning transfer refused");
}

/**
 * @brief Applies the controller output to the wheels. Drive needs driveEnabled and a clear overspeed guard.
 *        Otherwise both wheels get 0 A and are disabled; setWheelEnabled() acts only on a wheel that is still enabled.
 *        The motor diagnostic owns the outputs while it is active or applied.
 */
// Wheels are driven only when this is true. Torque mode has no speed loop: a 0 A command still outputs the estimated
// back-EMF as voltage, which is positive feedback, so any KV error makes an enabled wheel accelerate until the
// voltage ceiling (about 55 rad/s at 6.7 V, below the 60 rad/s overspeed limit). Measured on the robot 2026-10-23
// in Run with all gains 0. Keep false until KV and the speed protection are commissioned; Run then coasts.
constexpr bool kBalanceDrivesWheels = false;

void applyBalanceOutput(const Output &out) {
  if (motorDiagnosticState.active || motorDiagnosticApplied) return;
  if (kBalanceDrivesWheels && out.driveEnabled && !wheelOverspeed.tripped) {
    setWheelEnabled(motor1, true);
    setWheelEnabled(motor2, true);
    const WheelBoundary::MotorValues targets = WheelBoundary::toMotorTargets(out.leftA, out.rightA);
    motor1.target = targets.motor1;
    motor2.target = targets.motor2;
    return;
  }
  motor1.target = 0.0f;
  motor2.target = 0.0f;
  setWheelEnabled(motor1, false);
  setWheelEnabled(motor2, false);
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
  if (static_cast<int>(calibrationSelect) == 1 && balanceActive()) {
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
  wheelReadTimeUs = readTimeUs;
  wheelsValid = true;
  m1VelocityRadPerSec = rawForward.left;
  m2VelocityRadPerSec = rawForward.right;
  m1FilteredVelocityRadPerSec = m1VelocityFilter(m1VelocityRadPerSec);
  m2FilteredVelocityRadPerSec = m2VelocityFilter(m2VelocityRadPerSec);
}

void writeLegServos() {
  servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1,
                              servoTraceAngle[3], 1);
}

void runControlTick(bool imuUpdated) {
  controlStartUs = micros();
  controlGateSequence++;

  readBatteryVoltage();  // Battery

  updateStatusLed();
  serviceMotorDiagnostic();
  runCalibrationStep(imuUpdated);
  updateWheelVelocities();
  serviceWheelOverspeed();

  // Measurements are built before nowUs, so none of their timestamps is later than nowUs.
  const Measurements measurements = balanceMeasurements();
  const uint32_t balanceNowUs = micros();
  const Request request = balanceRequest(balanceNowUs);
  const Output out = balanceController.update(measurements, request, balanceConfig(), balanceNowUs);
  logBalanceChange(out);
  balanceOutput = out;
  robotTumble = out.fault == FaultReason::Fall ? ROBOT_TUMBLING : ROBOT_NOT_TUMBLING;
  movementSpeed = request.sourceValid ? request.speedMps : 0.0f;  // telemetry only
  bodyTurn = request.sourceValid ? request.yawRateRadPerSec : 0.0f;
  applyBalanceOutput(out);

  // Fixed standing pose: every servo at its calibrated zero, which is the servo calibration pose. No IK runs.
  servoTraceAngle[0] = -servoTrims.servo1Degrees;
  servoTraceAngle[1] = -servoTrims.servo2Degrees;
  servoTraceAngle[2] = -servoTrims.servo3Degrees;
  servoTraceAngle[3] = -servoTrims.servo4Degrees;
  writeLegServos();
  RobotLogCapture::observeServoAngles(servoTraceAngle);

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
 * 3. `runControlTick`, once at least 1 ms has passed since the previous tick: battery reading,
 *    status LED, calibration, wheel velocities, the balance controller and wheel outputs,
 *    the fixed servo pose, and telemetry.
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
