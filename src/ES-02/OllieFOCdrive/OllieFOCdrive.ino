#include <Arduino.h>
#include <SimpleFOC.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "BalanceController.h"
#include "BoardConfig.h"
#include "Calibration.h"
#include "CalibrationStore.h"
#include "ControlTick.h"
#include "FUTABA_SBUS.h"
#include "Logging.h"
#include "link_uart.h"
#include "MotorDiagnostic.h"
#include "settings_groups.h"
#include "OverspeedGuard.h"
#include "RobotConfig.h"
#include "WheelBoundary.h"
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

constexpr unsigned long kSerialBaudRate = LINK_USB_BAUD;
constexpr size_t kSerialTxBufferBytes = 1024;
constexpr size_t kSerialRxBufferBytes = 512;
#define DIAGNOSTIC_LOG_INTERVAL_US 5000  // Sample period the diagnostic battery filter is designed for (200 Hz).
constexpr unsigned int kDiagnosticLogSampleRateHz = 1000000U / DIAGNOSTIC_LOG_INTERVAL_US;
// -------------------------------------

// The state of the balance path: the RC frame, the link Command, the balance controller, the wheel and battery readings,
// the overspeed guard and the leg pose. The control tick and the host simulation both use it (ControlTick.h).
ControlTick::State control;
Config balanceSettings = defaultBalanceConfig();

void configureWheelMotor(BLDCMotor &motor, const WheelMotorParams &params);

int statusLedOn = 1;
int statusLedTicks = 0;
int statusLedPeriodTicks = 100;

uint32_t controlGateSequence = 0;
LinkSchedule linkOdometrySchedule;
ServoTrimsDegrees servoTrims{};

//  Create ServoControl object
ServoControl servoControl(board::kLegServoPin[0], board::kLegServoPin[1], board::kLegServoPin[2],
                          board::kLegServoPin[3]);

// Remote control
FUTABA_SBUS sBus;
bool rebootPending = false;
uint32_t rebootAtMs = 0;
constexpr uint32_t kRebootDelayMs = 300;  // a reboot waits this long, so the Ack and the action Log leave first

// Balance is requested by the CH5 switch or is running. Motor tests and calibration steps wait for it.
bool balanceRequested() { return ControlTick::balanceRequested(control); }

// The link's share of a pass, shared by the control tick and the sensor diagnostic loop. Requests that arrived since the
// last pass take effect at its start; the due signal rows leave at its end.
void linkPassStart(uint32_t passStartUs) { LinkServiceRequests(passStartUs, balanceRequested()); }
void linkPassEnd(uint32_t passStartUs) { LinkSampleSignals(passStartUs); }

// System health, the rows of the "Link and system health" section of signals.def.h. systemHealthTask() refreshes it
// once a second on core 0, so the control tick does no stack scans. Each field is one aligned uint32_t that only that
// task writes and the control loop reads unlocked; acceptable on this target.
struct SystemHealth {
  uint32_t uptimeS;
  uint32_t heapFreeB;
  uint32_t heapMinFreeB;
  uint32_t stackFreeLoopB;
  uint32_t stackFreeLinkB;
} systemHealth{};

extern TaskHandle_t loopTaskHandle;  // defined and set by the Arduino core (main.cpp); the health task measures its stack
constexpr uint32_t kSystemHealthStackBytes = 3072;
constexpr UBaseType_t kSystemHealthPriority = 1;

void systemHealthTask(void*) {
  for (;;) {
    const uint32_t nowMs = millis();
    systemHealth.uptimeS = nowMs / 1000;
    systemHealth.heapFreeB = esp_get_free_heap_size();
    systemHealth.heapMinFreeB = esp_get_minimum_free_heap_size();
    systemHealth.stackFreeLoopB = uxTaskGetStackHighWaterMark(loopTaskHandle);
    systemHealth.stackFreeLinkB = LinkMinStackFreeBytes();
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

float calibrationSelect = 0;  // The calibrate action: 0 ends a step, 1 gyroscope, 2 Euler angle, 3 servo

// Wheel Motors and Drivers
// m1 is the left wheel and m2 the right wheel; speeds are forward positive (see WheelBoundary.h). The raw speeds are in
// control; the filtered ones are kept here.
float m1FilteredVelocityRadPerSec = 0;
LowPassFilter m1VelocityFilter = LowPassFilter(0.01);  // Tf = 10ms

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

// Coast fault: 0 A and both drivers off. It repeats every tick while tripped, so no other path can re-enable them.
void serviceWheelOverspeed() {
  if (!ControlTick::serviceOverspeed(control, millis())) return;
  motor1.target = 0.0f;
  motor2.target = 0.0f;
  setWheelEnabled(motor1, false);
  setWheelEnabled(motor2, false);
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

// The log level, in the proto numbers the link uses: 1 DEBUG, 2 INFO, 3 WARN, 4 ERROR (Logging::Level is one less).
struct LogSettings {
  uint32_t minLevel;
};

struct LogGroup {
  using Struct = LogSettings;
  static LogSettings current() { return {static_cast<uint32_t>(Logging::minLevel()) + 1}; }
  static bool apply(const LogSettings &candidate, const char **error) {
    if (candidate.minLevel < 1 || candidate.minLevel > 4) {
      *error = "takes 1 (DEBUG) to 4 (ERROR)";
      return false;
    }
    Logging::setMinLevel(static_cast<Logging::Level>(candidate.minLevel - 1));
    return true;
  }
};

// Actions. The endpoint checks each argument against its ARG bounds before a handler runs. A handler starts the action,
// or sets *error and returns false. Motor safety lives in MotorDiagnostic; these handlers only add the overspeed latch.
bool startMotorRequest(const MotorDiagnostic::Request &request, const char **error) {
  // The diagnostic build initializes no motors or drivers and runs no motor loop, so a motor test must not start.
  if (SENSOR_DIAGNOSTIC_MODE) {
    *error = "sensor diagnostic build has no motor control";
    return false;
  }
  if (control.wheelOverspeed.tripped) {
    *error = "wheel overspeed latched, send motor_reset";
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
  if (!OverspeedGuard::MayRearm(control.wheelOverspeed, control.m1VelocityRadPerSec, control.m2VelocityRadPerSec)) {
    *error = "wheel overspeed latched, a wheel is above 30 rad/s";
    return false;
  }
  if (!MotorDiagnostic::resetFault(motorDiagnosticState, diagnosticInputs())) {
    *error = "balance off and safe speed required";
    return false;
  }
  ControlTick::resetOverspeed(control);
  return true;
}

bool startCalibration(const float *args, const char **error) {
  if (SENSOR_DIAGNOSTIC_MODE) {  // no calibration step runs in this build, so the request would be accepted and never run
    *error = "sensor diagnostic build runs no calibration";
    return false;
  }
  *error = CalibrationStartRefusal(args[0], balanceRequested(), motorDiagnosticState.active || motorDiagnosticApplied);
  if (*error != nullptr) return false;
  calibrationSelect = args[0];
  return true;
}

// Balance on or off from the link. Only in CH5 position 1, so a switch in another position never starts a robot. Entering
// position 1 is applied by the next control tick (ControlTick::runBalance); until then the switch position has not
// been seen, so an on or off here would be undone by that tick. Balance on also waits for any motor test or calibration.
bool setLinkBalance(const float *args, const char **error) {
  if (SENSOR_DIAGNOSTIC_MODE) {
    *error = "sensor diagnostic build has no motor control";
    return false;
  }
  *error = ControlTick::setLinkBalance(control, args[0] >= 0.5f, motorDiagnosticState.active || motorDiagnosticApplied,
                                       calibrationSelect != 0.0f);
  return *error == nullptr;
}

// Reboot. Refused while balance is requested, so a reboot never restarts a robot that drives or is about to.
bool requestReboot(const float *, const char **error) {
  if (balanceRequested()) {
    *error = "CH5 must be 0 and balance off";
    return false;
  }
  if (motorDiagnosticState.active || motorDiagnosticApplied) {
    *error = "motor test running";
    return false;
  }
  if (calibrationSelect != 0.0f) {
    *error = "calibration running";
    return false;
  }
  rebootPending = true;
  rebootAtMs = millis() + kRebootDelayMs;
  return true;
}

// A Command from the link. The handler refuses in this order and returns the first text that applies (nullptr accepts).
const char *handleLinkCommand(float speedMps, float turnRadPerSec) {
  if (SENSOR_DIAGNOSTIC_MODE) return "sensor diagnostic build has no motor control";
  return ControlTick::acceptLinkCommand(control, speedMps, turnRadPerSec, micros());
}

// The entry table (signals.def.h), expanded after the groups and handlers it names.
#define SIGNAL_DEF_FILE "signals.def.h"
#include "signal_table_expand.h"
// Git revision and build time, written by scripts/build_firmware.py. A build without the script says "unknown".
#if __has_include("build_id.h")
#include "build_id.h"
#else
#define NAVBOT_BUILD_ID "unknown " __DATE__
#endif
constexpr char kBuildId[] = NAVBOT_BUILD_ID;
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
  LinkSetCommandHandler(handleLinkCommand);  // only stores the handler once LinkUartInit has run
  xTaskCreatePinnedToCore(systemHealthTask, "health", kSystemHealthStackBytes, nullptr, kSystemHealthPriority, nullptr, 0);
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
  biquadFilterInitLPF(&control.voltageFilterLpf, 20, kDiagnosticLogSampleRateHz);
  diagnosticImuReady = imu::begin(imu::Settings{}, imuCalibration);
  sBus.begin();
  Logging::message(Logging::Level::Info, "DIAG", "boot reset_reason=%d imu_ok=%d motors=off servos=off", (int)esp_reset_reason(),
                diagnosticImuReady ? 1 : 0);
  return;
#endif

  biquadFilterInitLPF(&control.voltageFilterLpf, 50, 1000);  // TODO replace numbers with names

  //  Initialize servo
  // The unpowered legs sag to somewhere between 0 and kLegPoseDegrees. Attach at the middle of that range so the first
  // move is small, hold it during the motor calibration. runControlTick() ramps the legs up to kLegPoseDegrees.
  control.legPoseDegrees = kLegSagDegrees;
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
 * @brief Reads and processes data from the FUTABA S.BUS remote controller.
 *
 * Decodes the S.BUS signal into the RC state of `control`. Freshness and receiver failsafe are judged by the control tick.
 */
void readSbus() {
  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    sBus.toChannels = 0;
    sBus.UpdateChannels();
    sBus.toChannels = 0;

    ControlTick::applyRcFrame(control, sBus.channels, micros());
  }
}

/**
 * @brief Reads the battery voltage from the voltage divider and passes it to the control tick, which filters it.
 */
void readBatteryVoltage(void) {
  const uint32_t readTimeUs = micros();
  ControlTick::readBattery(control, static_cast<uint16_t>(analogRead(board::kBatteryAdcPin)), readTimeUs);
}

/**
 * @brief Applies the wheel command of the balance tick (ControlTick::runBalance). Drive enables both wheels before it sets
 *        the targets. Disable zeroes both targets, then disables the wheels that are still enabled. Keep leaves the
 *        outputs to the motor diagnostic.
 */
void applyBalanceOutput(const ControlTick::WheelCommand &command) {
  switch (command.action) {
    case ControlTick::WheelAction::Keep:
      return;
    case ControlTick::WheelAction::Disable:
      motor1.target = 0.0f;
      motor2.target = 0.0f;
      setWheelEnabled(motor1, false);
      setWheelEnabled(motor2, false);
      return;
    case ControlTick::WheelAction::Drive:
      setWheelEnabled(motor1, true);
      setWheelEnabled(motor2, true);
      motor1.target = command.motor1Target;
      motor2.target = command.motor2Target;
      return;
  }
}

ControlTick::MotorFeedback motorFeedback(const BLDCMotor &motor) {
  return {motor.voltage.q, motor.voltage_limit, motor.target, motor.current_limit};
}

// The RC is a valid source: the frame is fresh and the receiver reports no failsafe. The control tick applies the same
// rule (ControlTick::runBalance); the sensor diagnostic loop sets control.rcValid with it.
bool rcSourceValid(uint32_t nowUs) {
  return ControlTick::rcFrameFresh(control, nowUs) && sBus.Failsafe() == FUTABA_SBUS::kSignalOk;
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
  control.rcValid = rcSourceValid(passStartUs);  // telemetry, as the control tick sets it

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
  if (control.batteryVoltage <= board::kLowBatteryVolts) {
    statusLedPeriodTicks = 20;
    if (static_cast<uint32_t>(millis() - lastLowBatteryWarningMs) >= 1000) {
      lastLowBatteryWarningMs = millis();
      Logging::message(Logging::Level::Warning, "Voltage", "%.5f", control.batteryVoltage);
    }
  } else {
    statusLedPeriodTicks = 100;
  }
}

void runCalibrationStep(bool imuUpdated) {
  imu::CalibrationValues activeCalibration = imu::calibration();
  const imu::Snapshot &acceptedImu = imu::latest();
  CalibrationResult calibrationResult{CalibrationCompletion::None, false, {false, false, false, false}};
  if (static_cast<int>(calibrationSelect) == 1 && control.balanceOutput.state != State::Disabled) {
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
  if (!ControlTick::updateWheels(control, motor1.shaftAngle(), motor2.shaftAngle(), readTimeUs)) return;
  m1FilteredVelocityRadPerSec = m1VelocityFilter(control.m1VelocityRadPerSec);
  m2FilteredVelocityRadPerSec = m2VelocityFilter(control.m2VelocityRadPerSec);
}

void writeLegServos() {
  servoControl.setServosAngle(1, control.servoTraceAngle[0], -1, control.servoTraceAngle[1], -1,
                              control.servoTraceAngle[2], 1, control.servoTraceAngle[3], 1);
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
  if (control.wheelsValid && LinkScheduleDue(linkOdometrySchedule, controlStartUs, LINK_ODOMETRY_PERIOD_US)) {
    // Forward-positive wheel angles: the same sign as the wheel speeds, applied only through the wheel boundary.
    LinkPublishOdometry(control.wheelReadTimeUs, WheelBoundary::kLeftMotorSign * motor1.shaftAngle(),
                        WheelBoundary::kRightMotorSign * motor2.shaftAngle(), imu::latest());
  }

  // The balance controller reads the measurements, the RC state and the motor feedback, then returns the wheel command.
  const bool motorTestOwnsOutputs = motorDiagnosticState.active || motorDiagnosticApplied;
  const ControlTick::WheelCommand command =
      ControlTick::runBalance(control, balanceSettings, imu::latest(), motorFeedback(motor1), motorFeedback(motor2),
                              sBus.Failsafe() == FUTABA_SBUS::kSignalOk, motorTestOwnsOutputs, micros());
  applyBalanceOutput(command);

  // Fixed standing pose: every servo at kLegPoseDegrees, found on the robot 2026-10-24 (body higher than at 0). The
  // servo trims do not apply. No IK runs.
  // The commanded angle ramps up from the boot pose so the servos do not jump at full speed.
  ControlTick::stepLegs(control, controlTimestepSec);
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
  // A reboot requested over the link. Checked before the diagnostic branch, so both images restart.
  if (rebootPending && static_cast<int32_t>(millis() - rebootAtMs) >= 0) {
    // Balance or motor activity during the delay cancels the reboot: the checks of requestReboot() apply again.
    if (balanceRequested() || motorDiagnosticState.active || motorDiagnosticApplied || calibrationSelect != 0.0f) {
      rebootPending = false;
      Logging::message(Logging::Level::Warning, "REBOOT", "reboot cancelled: balance or motor activity");
    } else {
      ESP.restart();
    }
  }
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
