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
#include "FUTABA_SBUS.h"
#include "Logging.h"
#include "link_uart.h"
#include "MathConstants.h"
#include "MotorDiagnostic.h"
#include "settings_groups.h"
#include "OverspeedGuard.h"
#include "WheelBoundary.h"
#include "OllieFOCdrive.h"
#include "LegPoseRamp.h"
#include "ServoControl.h"
#include "filter.h"
#include "imu.h"

// SimpleFOC's debug output is a Print that receives lines ending in '\n'. Each line goes to the link as a Log frame.
class SimpleFocLog : public Print {
 public:
  size_t write(uint8_t value) override {
    if (value == '\n') {
      emitLine();
    } else if (value != '\r') {
      if (length_ == sizeof(line_) - 1) emitLine();
      line_[length_++] = static_cast<char>(value);
    }
    return 1;
  }

 private:
  void emitLine() {
    line_[length_] = '\0';
    Logging::message(Logging::Level::Info, "SimpleFOC", "%s", line_);
    length_ = 0;
  }

  char line_[96];
  size_t length_ = 0;
};
SimpleFocLog simpleFocLog;

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

constexpr unsigned long kSerialBaudRate = LINK_USB_BAUD;
constexpr size_t kSerialTxBufferBytes = 1024;
constexpr size_t kSerialRxBufferBytes = 512;
#define DIAGNOSTIC_LOG_INTERVAL_US 5000  // Sample period the diagnostic battery filter is designed for (200 Hz).
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

// SimpleFOC adds the estimated back-EMF as voltage even at 0 A. If that estimate is larger than the real back-EMF
// the wheel accelerates by itself, so the rating handed to SimpleFOC is the measured KV times this factor (a larger
// KV gives a smaller estimate). Measured KV depends on the speed range (79.5 rpm/V over 0.5-2 V, about 108 rpm/V for
// 0.5-1 V). Bench test of the left wheel with motor_current 0 0 on 2026-10-24: 1.0 ran away from rest, 1.4 let the wheel speed
// up again between 17 and 30 rad/s, 1.8 slowed it down continuously. The right wheel takes the same factor on the
// operator's decision; it is not tested separately.
constexpr float kWheelKvSafetyFactor = 1.8f;

// Balance settings group (the PARAM entries of signals.def.h name its fields). Built once at boot; SetParam replaces
// it through BalanceGroup, and the controller reads it at each tick. The balance and yaw gains below are the working
// values (tilt 8 A/rad, gyro damping 0.4), so the controller commands real wheel current as soon as it arms and runs.
// The envelope can also command a small braking current above roughly 50 rad/s, because the plan requires outputs
// inside the envelope. Each value is one of: existing constant (named), plan (decision or stated number), or an
// uncommissioned placeholder.
Config balanceSettings = [] {
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
    // Same rating as handed to SimpleFOC, so the envelope matches the voltage the drive really outputs. 
    c.kvRating = std::min(kLeftWheelMotor.kvRatingRpmPerV, kRightWheelMotor.kvRatingRpmPerV) * kWheelKvSafetyFactor;  // rpm/V
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
    c.yawRateCapRadPerSec = 1.0f;                // uncommissioned: doubled after the stick test 2026-10-24, rad/s
    c.requestMaxAgeUs = kRcFrameTimeoutMs * 1000U;  // existing constant (RC frame timeout), us
    c.demandScaleCapStart = 0.5f;                // uncommissioned: proposed starting value, dimensionless

    // Speed loop (gains zero, integral off)
    c.wheelRadiusM = 0.019f;                     // measured 2026-10-24: diameter 38.3 mm, 38 mm used, m
    c.speedLoopPeriodUs = 20000;                 // plan: provisional 50 Hz speed loop, us
    c.speedKpRadPerMps = 0.25f;                  // uncommissioned: tilt rad per m/s (0.2 m/s error asks 0.05 rad)
    c.speedKiRadPerMpsS = 0.08f;                 // uncommissioned: tilt rad per m (0.1 m/s error for 1 s adds 0.008 rad)
    c.speedIntegralEnabled = true;               // uncommissioned: on, learns the lean that cancels an off-centre centre of mass
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

    // Balance and yaw law
    c.tiltPAPerRad = 8.0f;                       // uncommissioned: A/rad (5 deg lean asks 0.70 A)
    c.gyroDampingAPerRadPerSec = 0.4f;           // uncommissioned: A/(rad/s) (1 rad/s asks 0.4 A)
    c.yawPAPerRadPerSec = 0.6f;                  // uncommissioned: A/(rad/s) (1 rad/s yaw error asks 0.6 A per side)

    // Reserve and authority
    c.reserveLowA = 0.2f;                        // uncommissioned: proposed starting value, A
    c.authorityDurationUs = 2000000;             // uncommissioned: proposed starting value, 2 s

    // Sketch-supplied inputs: first commissioning, trim taken as validated at zero (user decision 2026-10-22)
    c.trimValidated = true;
    c.storedTrimDeg = 0.0f;                      // deg
    c.fixedPoseReachable = true;                 // plan: fixed pose, all joint angles 0
    return c;
}();

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
float batteryInstantRawV = 0;     // Battery voltage from the unfiltered ADC reading

uint32_t controlGateSequence = 0;
LinkSchedule linkOdometrySchedule;
ServoTrimsDegrees servoTrims{};
constexpr int kLegPoseDegrees = 20;  // fixed leg pose for all four servos, degrees
constexpr int kLegSagDegrees = 10;  // assumed leg angle after the robot has sagged, between 0 and kLegPoseDegrees
constexpr float kLegRampDegPerSec = 30.0f;  // leg pose ramp, 0 to 20 degrees in about 0.7 s
constexpr float kLegRampMaxDtSec = 0.02f;   // the first tick after boot reports the whole boot time as dt
float legPoseDegrees = 0.0f;                // commanded leg angle, ramped toward kLegPoseDegrees

//  Create ServoControl object
ServoControl servoControl(board::kLegServoPin[0], board::kLegServoPin[1], board::kLegServoPin[2],
                          board::kLegServoPin[3]);
int servoTraceAngle[4] = {0, 0, 0, 0};  // Last angle arguments sent to the four servos

// Remote control
FUTABA_SBUS sBus;
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
// Balance is requested by the CH5 switch or is running. Motor tests and calibration steps wait for it.
bool balanceRequested() { return pidGainsMode != REMOTE_CONTROL_PID_GAINS_MODE_OFF || balanceActive(); }

// The link's share of a pass, shared by the control tick and the sensor diagnostic loop. Requests that arrived since the
// last pass take effect at its start; the due signal rows leave at its end.
void linkPassStart(uint32_t passStartUs) { LinkServiceRequests(passStartUs, balanceRequested()); }
void linkPassEnd(uint32_t passStartUs) { LinkSampleSignals(passStartUs); }

float calibrationSelect = 0;  // The calibrate action: 0 ends a step, 1 gyroscope, 2 Euler angle, 3 servo

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
uint32_t loopEntryIntervalUs = 0;  // time between the last two loop() entries
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
  return {millis(), balanceRequested(), calibrationSelect != 0.0f, motor1.shaft_velocity, motor2.shaft_velocity};
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

// Called after a successful motor_reset and by balanceRequest() while CH5 is off. Rearms the guard only while both wheels are below the limit.
// Rearming does not enable the drivers. balanceRequest() reports enable off while the guard is tripped, so CH5 must
// go off and on again before the controller drives the wheels.
void resetWheelOverspeed() {
  if (OverspeedGuard::WithinLimit(m1VelocityRadPerSec, m2VelocityRadPerSec)) OverspeedGuard::reset(wheelOverspeed);
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
  motor.KV_rating = params.kvRatingRpmPerV * kWheelKvSafetyFactor;
  motor.voltage_limit = kWheelVoltageLimitV;
  motor.current_limit = kWheelVoltageLimitV / params.phaseResistanceOhm;  // stall current at the voltage ceiling
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
}

void initIMU(const imu::CalibrationValues &imuCalibration) {
  if (!imu::begin(imu::Settings{}, imuCalibration)) {
    Logging::message(Logging::Level::Error, "IMU", "initialization failed");
    while (1);
  }
  Logging::message(Logging::Level::Info, "IMU", "initialized successfully");
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

// Settings groups: the running struct of each PARAM group and the one apply function that replaces it. A refused value
// leaves the struct unchanged (settings_groups.h has the rules; imu::setSettings checks the IMU ones).
struct BalanceGroup {
  using Struct = Config;
  static Config current() { return balanceSettings; }
  static bool apply(const Config &candidate, const char **error) {
    return ApplyBalanceSettings(balanceSettings, candidate, error);
  }
};

struct ImuGroup {
  using Struct = imu::Settings;
  static imu::Settings current() { return imu::settings(); }
  static bool apply(const imu::Settings &candidate, const char **error) {
    if (imu::setSettings(candidate)) return true;
    *error = "out of range";
    return false;
  }
};

struct ServoTrimGroup {
  using Struct = ServoTrimsDegrees;
  static ServoTrimsDegrees current() { return servoTrims; }
  static bool apply(const ServoTrimsDegrees &candidate, const char **error) {
    return ApplyServoTrims(servoTrims, candidate, error);
  }
};

// Actions. The endpoint checks each argument against its ARG bounds before a handler runs. A handler starts the action,
// or sets *error and returns false. Motor safety lives in MotorDiagnostic; these handlers only add the overspeed latch.
bool startMotorRequest(const MotorDiagnostic::Request &request, const char **error) {
  // The diagnostic build initializes no motors or drivers and runs no motor loop, so a motor test must not start.
  if (SENSOR_DIAGNOSTIC_MODE) {
    *error = "refused: sensor diagnostic build has no motor control";
    return false;
  }
  if (wheelOverspeed.tripped) {
    *error = "refused: wheel overspeed latched, send motor_reset";
    return false;
  }
  const MotorDiagnostic::RequestResult result = MotorDiagnostic::start(motorDiagnosticState, request, diagnosticInputs());
  if (result != MotorDiagnostic::RequestResult::Accepted) {
    *error = MotorDiagnostic::RequestResultText(result);
    return false;
  }
  applyMotorDiagnosticRequest();
  return true;
}

bool startMotorCurrent(const float *args, const char **error) {
  MotorDiagnostic::Request request{};
  if (!MotorDiagnostic::MakeCurrentRequest(args[0], args[1], args[2], request)) {
    *error = "invalid request";
    return false;
  }
  return startMotorRequest(request, error);
}

bool startMotorVoltage(const float *args, const char **error) {
  MotorDiagnostic::Request request{};
  if (!MotorDiagnostic::MakeVoltageRequest(args[0], args[1], args[2], request)) {
    *error = "invalid request";
    return false;
  }
  return startMotorRequest(request, error);
}

bool stopMotorTest(const float *, const char **) {
  MotorDiagnostic::stop(motorDiagnosticState);
  return true;
}

bool resetMotorFault(const float *, const char **error) {
  // Checked before the motor fault is cleared, so a refused reset leaves that fault latched too.
  if (!OverspeedGuard::MayRearm(wheelOverspeed, m1VelocityRadPerSec, m2VelocityRadPerSec)) {
    *error = "refused: wheel overspeed latched, a wheel is above 30 rad/s";
    return false;
  }
  if (!MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs())) {
    *error = "refused: balance off and safe speed required";
    return false;
  }
  resetWheelOverspeed();
  return true;
}

bool startCalibration(const float *args, const char **error) {
  if (SENSOR_DIAGNOSTIC_MODE) {  // no calibration step runs in this build, so the request would be accepted and never run
    *error = "refused: sensor diagnostic build runs no calibration";
    return false;
  }
  *error = CalibrationStartRefusal(args[0], balanceRequested(), motorDiagnosticState.active || motorDiagnosticApplied);
  if (*error != nullptr) return false;
  calibrationSelect = args[0];
  return true;
}

// The entry table (signals.def.h), expanded after the groups and handlers it names.
#define SIGNAL_DEF_FILE "signals.def.h"
#include "signal_table_expand.h"
// No git hash is available to the Arduino build, so the build ID is the compile time.
constexpr char kBuildId[] = __DATE__ " " __TIME__;
static_assert(sizeof(kBuildId) <= 33, "the build ID has to fit CatalogEntry.build");

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
  Serial.setRxBufferSize(kSerialRxBufferBytes);
  Serial.begin(kSerialBaudRate);
  LinkUsbInit();
  // Before the first Logging::message: a message queued for UART1 waits for this writer.
  LinkUartInit(kSignals, kSignalCount, kBuildId);
  const PersistedCalibrationValues loadedCalibration = calibrationStore.load();
  servoTrims = loadedCalibration.servos;
  const imu::CalibrationValues imuCalibration{loadedCalibration.gyro, loadedCalibration.attitude};
  logCalibration(loadedCalibration);

  // TODO is this the right location?
  pinMode(board::kStatusLedPin, OUTPUT);
  digitalWrite(board::kStatusLedPin, LOW);
  Logging::message(Logging::Level::Info, "Setup", "system run.");
  delay(500);

#if SENSOR_DIAGNOSTIC_MODE
  biquadFilterInitLPF(&voltageFilterLpf, 20, kDiagnosticLogSampleRateHz);
  diagnosticImuReady = imu::begin(imu::Settings{}, imuCalibration);
  sBus.begin();
  Logging::message(Logging::Level::Info, "DIAG", "boot reset_reason=%d imu_ok=%d motors=off servos=off", (int)esp_reset_reason(),
                diagnosticImuReady ? 1 : 0);
  return;
#endif

  biquadFilterInitLPF(&voltageFilterLpf, 50, 1000);  // TODO replace numbers with names

  //  Initialize servo
  // The unpowered legs sag to somewhere between 0 and kLegPoseDegrees. Attach at the middle of that range so the first
  // move is small, hold it during the motor calibration. runControlTick() ramps the legs up to kLegPoseDegrees.
  legPoseDegrees = kLegSagDegrees;
  servoControl.initialize(kLegSagDegrees);

  initIMU(imuCalibration);
  initRemoteControl();
  SimpleFOCDebug::enable(&simpleFocLog);  // TODO a more centralized way to enable debug sections would be nice
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

  Logging::message(Logging::Level::Info, "Setup", "Motor ready.");

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
  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    sBus.toChannels = 0;
    sBus.UpdateChannels();
    sBus.toChannels = 0;

    applyRcChannels(sBus.channels);
  }
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
  batteryInstantRawV = board::kBatteryVoltsPerAdcCount * voltageAdc;
  voltageAdcFiltered = biquadFilterApply(&voltageFilterLpf, voltageAdc);
  batteryVoltage = board::kBatteryVoltsPerAdcCount * voltageAdcFiltered;
}

// Balance measurements, built at the hardware boundary. Sign and axis checked on the supported robot 2026-10-24:
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
  const Config &config = balanceSettings;
  const bool fresh = rcFrameSeen && static_cast<uint32_t>(nowUs - rcFrameUs) <= kRcFrameTimeoutMs * 1000U;
  // CH5 off rearms a tripped overspeed guard (once both wheels are below the limit), so no serial command is needed.
  if (fresh && rcChannels[4] < kRcEnableOnRaw) resetWheelOverspeed();
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
void applyBalanceOutput(const Output &out) {
  if (motorDiagnosticState.active || motorDiagnosticApplied) return;
  if (out.driveEnabled && !wheelOverspeed.tripped) {
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

// The sensor diagnostic loop: the IMU, the receiver, the battery and the link. It has no control tick, so the entries
// of the motors, servos, controller and loop timing read their initial values (see docs/robot/software/sensor-diagnostic-mode.md).
void diagnosticLoop(void) {
  static uint32_t lastTimingReportUs = 0;
  static uint32_t lastTimingAcceptedSamples = 0;
  static bool timingReportStarted = false;
  static unsigned long lastVoltageMs = 0;
  const unsigned long nowMs = millis();
  const uint32_t passStartUs = micros();

  linkPassStart(passStartUs);
  readSbus();

  if (diagnosticImuReady) imu::service();

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
      // No elapsed_us: the window is one second, and the line must fit the 96 characters of a Log text (81 at worst).
      Logging::message(Logging::Level::Info, "IMU_TIMING", "accepted=%lu,rate_hz=%.1f,long_gap_total=%lu,max_interval_us=%lu",
                    static_cast<unsigned long>(acceptedSamples), acceptedRateHz,
                    static_cast<unsigned long>(timing.longGapCount), static_cast<unsigned long>(timing.maxIntervalUs));
      lastTimingReportUs = timingNowUs;
      lastTimingAcceptedSamples = timing.acceptedSamples;
    }
  }

  if (nowMs - lastVoltageMs >= 10) {
    lastVoltageMs = nowMs;
    readBatteryVoltage();
  }

  linkPassEnd(passStartUs);
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

  // This pass uses the S.BUS frame read above, so a CH5 switch or an expired lease ends a motor test here rather than
  // in the next loop pass. Skipped on the normal path: no diagnostic is active or still applied to the motors.
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

  linkPassStart(controlStartUs);  // before anything is read
  readBatteryVoltage();  // Battery

  updateStatusLed();
  serviceMotorDiagnostic();
  runCalibrationStep(imuUpdated);
  updateWheelVelocities();
  serviceWheelOverspeed();
  if (wheelsValid && LinkScheduleDue(linkOdometrySchedule, controlStartUs, LINK_ODOMETRY_PERIOD_US)) {
    // Forward-positive wheel angles: the same sign as the wheel speeds, applied only through the wheel boundary.
    LinkPublishOdometry(wheelReadTimeUs, WheelBoundary::kLeftMotorSign * motor1.shaftAngle(),
                        WheelBoundary::kRightMotorSign * motor2.shaftAngle(), imu::latest());
  }

  // Measurements are built before nowUs, so none of their timestamps is later than nowUs.
  const Measurements measurements = balanceMeasurements();
  const uint32_t balanceNowUs = micros();
  const Request request = balanceRequest(balanceNowUs);
  const Output out = balanceController.update(measurements, request, balanceSettings, balanceNowUs);
  logBalanceChange(out);
  balanceOutput = out;
  robotTumble = out.fault == FaultReason::Fall ? ROBOT_TUMBLING : ROBOT_NOT_TUMBLING;
  movementSpeed = request.sourceValid ? request.speedMps : 0.0f;  // telemetry only
  bodyTurn = request.sourceValid ? request.yawRateRadPerSec : 0.0f;
  applyBalanceOutput(out);

  // Fixed standing pose: every servo at kLegPoseDegrees, found on the robot 2026-10-24 (body higher than at 0). The
  // servo trims do not apply. No IK runs.
  // The commanded angle ramps up from the boot pose so the servos do not jump at full speed.
  legPoseDegrees = stepLegPoseToward(legPoseDegrees, kLegPoseDegrees, kLegRampDegPerSec,
                                     fminf(controlTimestepSec, kLegRampMaxDtSec));
  for (int &angle : servoTraceAngle) angle = static_cast<int>(lroundf(legPoseDegrees));
  writeLegServos();

  controlExecutionUs = static_cast<uint32_t>(micros() - controlStartUs);
  linkPassEnd(controlStartUs);

  previousControlTimeUs = loopTimeUs;
}

/**
 * @brief The main execution loop of the program.
 *
 * This function runs repeatedly after `setup()` is complete:
 * 1. `updateMotorsAndCommands`: SimpleFOC `move()` and `loopFOC()` for both wheels.
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
  if (hasPreviousLoopEntry) loopEntryIntervalUs = static_cast<uint32_t>(loopTimeUs - previousLoopEntryUs);
  previousLoopEntryUs = loopTimeUs;
  hasPreviousLoopEntry = true;

  updateMotorsAndCommands();
  const bool imuUpdated = readInputs();

  controlTimestepSec = (loopTimeUs - previousControlTimeUs) / 1000000.0f;
  if (controlTimestepSec >= kControlTimestepMinSec) {
    runControlTick(imuUpdated);
  }
}
