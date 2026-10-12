#include <Arduino.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <SimpleFOC.h>
#include "BoardConfig.h"
#include "FUTABA_SBUS.h"
#include "ServoControl.h"
#include "imu.h"
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include "OllieFOCdrive.h"
#include "filter.h"
#include "touchscreen.h"
#include "ble.h"
#include "robot.h"
#include "Logging.h"
#include "RobotLogCapture.h"
#include "DebugTelemetry.h"
#include "LegKinematics.h"
#include "Calibration.h"
#include "CalibrationStore.h"

class LoggingCommandStream : public Stream {
 public:
  int available() override { return Serial.available(); }
  int read() override { return Serial.read(); }
  int peek() override { return Serial.peek(); }
  size_t write(uint8_t value) override {
    if (Logging::profile() != Logging::Profile::Idle) return 1;
    return Serial.write(value);
  }
  size_t write(const uint8_t *buffer, size_t size) override {
    if (Logging::profile() != Logging::Profile::Idle) return size;
    return Serial.write(buffer, size);
  }
  void flush() override {
    if (Logging::profile() == Logging::Profile::Idle) Serial.flush();
  }
};

LoggingCommandStream commandStream;
Commander command = Commander(commandStream);
Calibration calibration;
CalibrationStore calibrationStore;

// ----- Editable Constants
#define COMMUNICATION_OBJECT COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE          // 0: two-wheel balance  1: simpleFOC Studio host computer  2: control dual motors

// Safe bring-up: read sensors and receiver without energizing wheels or servos.
// Set to 0 only after the diagnostic readings and power supply are checked.
#define SENSOR_DIAGNOSTIC_MODE 0

// Tuned two-wheel drive defaults, including corrected wheel-speed timing and
// CH3 scaling. Apply them at startup and through a CH5 switch transition.
// Set to 0 to restore the repository's mode-dependent defaults.
#define DIAGNOSTIC_LIVE_TUNING_DEFAULTS 1

#define ADJUST_PARAMETER ADJUST_BALANCE_SPEED_YAW_ROLL        // 0: balance, speed, yaw, roll parameter tuning   1: ball pushing

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
constexpr BalancePidPreset kPidWithoutTouch{
    {0.06, 0.0, 0.0, 2},
    {0.045, 0.005, 0, 50},
    {5, 200, 0.11, 0.1}};
constexpr BalancePidPreset kPidWithTouch{
    {0.08, 1.5, 0.005, 2},
    {0.045, 0.005, 0, 50},
    {5, 200, 0.11, 0.1}};
constexpr PidPreset kYawPid{4, 0, 0, 0};
constexpr PidPreset kTouchXPid{0.2, 0, 0.04, 0};
constexpr PidPreset kTouchYPid{0.2, 170, 0.08, 0};

#define SBUS_CHANNEL_MAX 1792
#define SBUS_CHANNEL_MIN 192
constexpr uint32_t kRcFrameTimeoutMs = 100;
constexpr float kControlTimestepMinSec = 0.001f;

struct RcRange {
  long min;
  long max;
};

constexpr unsigned long kSerialBaudRate = 576000;
#define DIAGNOSTIC_LOG_INTERVAL_US 5000 // Diagnostic CSV rows are emitted at 200 Hz.
constexpr unsigned int kDiagnosticLogSampleRateHz = 1000000U / DIAGNOSTIC_LOG_INTERVAL_US;
// Conservative drive tuning parameters; verify the wheel feedback sign on hardware.
constexpr float kDriveBodyXLimitM = 0.010f;
constexpr float kDriveWheelFeedbackLimit = 8.0f;
constexpr float kDriveTiltReductionStartDeg = 5.0f;
constexpr float kDriveTiltReductionFullDeg = 10.0f;
// -------------------------------------

struct BodyPose {
  float heightM;
  float roll;
};

struct WheelPidParams {
  float p;
  float i;
  float d;
};

constexpr float kWheelSupplyVoltageV = 8.4;
constexpr float kWheelVelocityLimitRadPerSec = 88.0;
constexpr float kWheelCurrentLimitA = 5.0;
constexpr float kWheelPhaseResistanceOhm = 22;
constexpr float kWheelVelocityPidOutputRamp = 10000;
constexpr float kWheelVelocityFilterTfSec = 0.001;
constexpr WheelPidParams kWheelVelocityPid{0.006, 0, 0};

constexpr float kBodyHeightOverrideDisabledM = 0.0f;
constexpr float kBodyHeightMinM = 0.05f;
constexpr float kBodyHeightDefaultM = 0.06f;
constexpr float kBallPoiseMaxBodyHeightM = 0.07f;
constexpr float kBodyHeightMaxM = 0.09f;
constexpr int kBodyHeightCommandFilterIndex = 2;

// Body
float TargetBodyHeightOverrideM = kBodyHeightOverrideDisabledM;  // Zero disables the Commander height override.
float BodyHeightCommandM = kBodyHeightDefaultM;  // Remote-controlled body height
float BodyPitching = 0;             // Pitch
float BodyRoll = 0;                 // Roll
float MovementSpeed = 0;            // Movement speed
float BodyTurn = 0;                 // Turning
float BodyX = 0;                    // X position (controller output)
int RobotTumble = ROBOT_NOT_TUMBLING;  // Fall-detection state, see docs/robot/ROBOT_ROADMAP.md


// Filtered remote-control commands
float BodyHeightCommandFilteredM = kBodyHeightDefaultM;
float BodyPitchingFiltered = 0;      // Pitch
float BodyRollFiltered = 0;          // Roll
biquadFilter_t BodyCommandFilterLPF[12];  // Second-order low-pass filter
float TouchYPidOutputFiltered = 0;

float bodyCommandFilterCutoffHz = 200; // Low-pass cutoff frequency (Hz) for remote-control body command filtering.
float bodyCommandFilteringEnabled = 1;

void SetBodyCommandFilterCutoffHz(char *cmd) {
  command.scalar(&bodyCommandFilterCutoffHz, cmd);
}

void SetBodyCommandFilteringEnabled(char *cmd) {
  command.scalar(&bodyCommandFilteringEnabled, cmd);
}

int statusLedOn = 1;
int statusLedTicks = 0;
int statusLedPeriodTicks = 100;
// Battery Voltage Measurement
biquadFilter_t VoltageFilterLPF;  // Second-order low-pass filter
uint16_t VoltageADC = 0;          // Battery voltage ADC data
float VoltageADCf = 0;            // Battery voltage ADC data
float Voltage = 0;                // Battery voltage

uint32_t controlGateSequence = 0;
DebugTelemetry::TelemetryState debugTelemetryState{};
uint32_t diagnosticSequence = 0;
ServoTrimsDegrees servoTrims{};

bool parseCommandFloat(const char *text, float& value) {
  char *end = nullptr;
  value = strtof(text, &end);
  if (end == text) return false;
  while (end && isspace(static_cast<unsigned char>(*end))) ++end;
  return end && *end == '\0' && isfinite(value);
}

bool parseCommandCutoff(const char *text, uint16_t& value) {
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

void Target_Body_Height(char *cmd) {
  command.scalar(&TargetBodyHeightOverrideM, cmd);
}

// Servo zero position offsets
void zeroBias_servo1(char *cmd) {
  command.scalar(&servoTrims.servo1Degrees, cmd);
}
void zeroBias_servo2(char *cmd) {
  command.scalar(&servoTrims.servo2Degrees, cmd);
}
void zeroBias_servo3(char *cmd) {
  command.scalar(&servoTrims.servo3Degrees, cmd);
}
void zeroBias_servo4(char *cmd) {
  command.scalar(&servoTrims.servo4Degrees, cmd);
}

bool pid_gains_mode_is_enabled(int mode) {
  return (mode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH || mode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH);
}

//  Create ServoControl object
ServoControl servoControl(board::kLegServoPin[0], board::kLegServoPin[1],
                          board::kLegServoPin[2], board::kLegServoPin[3]);
int servoTraceAngle[4] = { 0, 0, 0, 0 };  // Last angle arguments sent to the four servos

// Remote control
FUTABA_SBUS sBus;
int sbusFrameIntervalMs = 0;
int pid_gains_mode = REMOTE_CONTROL_PID_GAINS_MODE_OFF;
int posture_or_mark_mode = REMOTE_CONTROL_PM_POSTURE_MODE;
int roll_mode = REMOTE_CONTROL_ROLL_MODE_MANUAL;
int attitude_mode = REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT;
float top_ball_x = 0;
float top_ball_y = 0;
float sbus_top_ball_x_smoothed = 0;
float sbus_top_ball_y_smoothed = 0;

//  Create PID controller instance
float debugLogSelector = 0;  // Selects debug data and the log profile.
float CalibrationSelect = 0;  // Save calibration data 0: Calibration end  1: Calibrate gyroscope  2: Calibrate Euler angle  3: Calibrate servo

float PidParameterTuning = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? 1 : 0;  // 0: auto gains  1: live tuning

PIDController anglePidSettings(kPidWithTouch.angle.p, kPidWithTouch.angle.i,
                               kPidWithTouch.angle.d, 0, kPidWithTouch.angle.limit);
PIDController speedPidSettings(kPidWithTouch.speed.p, kPidWithTouch.speed.i,
                               kPidWithTouch.speed.d, 0, kPidWithTouch.speed.limit);
PIDController yawPidSettings(kYawPid.p, kYawPid.i, kYawPid.d, 0, kYawPid.limit);
PIDController rollPidSettings(
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? kPidWithTouch.roll.p : 0.06,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? kPidWithTouch.roll.i : 1.5,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? kPidWithTouch.roll.d : 0.003, 0,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? kPidWithTouch.roll.limit : 2);
PIDController touchXPidSettings(kTouchXPid.p, kTouchXPid.i, kTouchXPid.d, 0, kTouchXPid.limit);
PIDController touchYPidSettings(kTouchYPid.p, kTouchYPid.i, kTouchYPid.d, 0, kTouchYPid.limit);

float control_torque_compensation = 0;  // Control torque compensation
float wheelSpeedFeedbackGain = 0.0f;
float wheelVelocityFeedbackCorrection = 0;
float driveTiltReductionGain = 1.0f;  // Scales the speed reduction applied as tilt approaches kDriveTiltReductionFullDeg.
float driveEffectiveSpeed = 0;
float driveSpeedBodyXRaw = 0;

float pidTimestepSec = 0.01;

MyPIDController Angle_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);  // p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController Speed_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);
MyPIDController Yaw_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);
MyPIDController Roll_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);

MyPIDController TouchX_Pid(0, 0, 0, 0, 10, pidTimestepSec, 0, 0);
MyPIDController TouchY_Pid(0, 0, 0, 0, 8, pidTimestepSec, 0, 0);
bool balancePidNeedsPriming = true;  // Priming avoids a derivative kick when CH5 enables the balance loop.

void ControlTorqueCompensation(char *cmd) {
  command.scalar(&control_torque_compensation, cmd);
}

void CbWheelSpeedFeedbackGain(char *cmd) {
  command.scalar(&wheelSpeedFeedbackGain, cmd);
}

void CbDriveTiltReductionGain(char *cmd) {
  command.scalar(&driveTiltReductionGain, cmd);
}

void Pid_Parameter_Tuning(char *cmd) {
  command.scalar(&PidParameterTuning, cmd);
}
void User_command(char *cmd) {
  if (*cmd == '{') {
    rp.json_test(cmd);
  }else{
    Pid_Parameter_Tuning(cmd);
  }
}

void TwoKp(char *cmd) {
  float value = 0.0f;
  imu::Settings candidate = imu::settings();
  if (!parseCommandFloat(cmd, value)) return;
  candidate.mahonyTwoKp = value;
  if (imu::setSettings(candidate)) Logging::message(Logging::Level::Info, "MAHONY_KP", "%.6f", value);
  else Logging::message(Logging::Level::Warning, "IMU", "invalid Mahony Kp");
}
void TwoKi(char *cmd) {
  float value = 0.0f;
  imu::Settings candidate = imu::settings();
  if (!parseCommandFloat(cmd, value)) return;
  candidate.mahonyTwoKi = value;
  if (imu::setSettings(candidate)) Logging::message(Logging::Level::Info, "MAHONY_KI", "%.6f", value);
  else Logging::message(Logging::Level::Warning, "IMU", "invalid Mahony Ki");
}

void KeyScalar(char *cmd) {
  command.scalar(&debugLogSelector, cmd);
  RobotLogCapture::setProfileForSelector(static_cast<int>(debugLogSelector));
}
void KeyCalibration(char *cmd) {
  command.scalar(&CalibrationSelect, cmd);
}

#if ADJUST_PARAMETER == ADJUST_BALANCE_SPEED_YAW_ROLL
// Cb prefix: Commander callback.
void CbAnglePid(char *cmd) {
  command.pid(&anglePidSettings, cmd);
}
void CbSpeedPid(char *cmd) {
  command.pid(&speedPidSettings, cmd);
}
void CbYawPid(char *cmd) {
  command.pid(&yawPidSettings, cmd);
}

void CbRollPid(char *cmd) {
  command.pid(&rollPidSettings, cmd);
}

#elif ADJUST_PARAMETER == ADJUST_BALL_PUSHING

void CbTouchXPid(char *cmd) {
  command.pid(&touchXPidSettings, cmd);
}

void CbTouchYPid(char *cmd) {
  command.pid(&touchYPidSettings, cmd);
}
#endif

//Wheel Motors and Drivers
double m1PrevEncoderAngleRad = 0;
float m1VelocityRadPerSec = 0;
float m1FilteredVelocityRadPerSec = 0;
LowPassFilter m1VelocityFilter = LowPassFilter(0.01);  // Tf = 10ms

double m2PrevEncoderAngleRad = 0;
float m2VelocityRadPerSec = 0;
float m2FilteredVelocityRadPerSec = 0;
LowPassFilter m2VelocityFilter = LowPassFilter(0.01);  // Tf = 10ms

float controlTimestepSec = 0;
unsigned long loopTimeUs = 0;
unsigned long previousControlTimeUs = 0;

BLDCMotor motor1 = BLDCMotor(board::kWheelMotorPolePairs);
BLDCDriver3PWM driver = BLDCDriver3PWM(board::kWheel1PhaseAPin, board::kWheel1PhaseBPin,
                                       board::kWheel1PhaseCPin, board::kWheel1EnablePin);

BLDCMotor motor2 = BLDCMotor(board::kWheelMotorPolePairs);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(board::kWheel2PhaseAPin, board::kWheel2PhaseBPin,
                                        board::kWheel2PhaseCPin, board::kWheel2EnablePin);

MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

void handleM1MotionCommand(char *cmd) {
  command.motion(&motor1, cmd);
}
void handleM1Command(char *cmd) {
  command.motor(&motor1, cmd);
}

void RXsbus();
void PIDcontroller_posture(float dt);
void RemoteControlFiltering(void);
void ReadVoltage(void);
void PidParameter(void);
void Robot_Tumble(void);
void DiagnosticLoop(void);
bool diagnosticImuReady = false;
unsigned long diagnosticLastRcFrameMs = 0;
bool diagnosticHasRcFrame = false;

void cpu0_task(void *ptParam) {
  while(1)
  {
    if(ten_msec_tick()){
      ble_loop();
    }
  }
}
bool ten_msec_tick(void) {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 10) {
    lastMillis = currentMillis;
    return 1;
  }
  //Overflow handling
  if (lastMillis > currentMillis) {
    lastMillis = currentMillis;
  }
  return 0;
}

void logCalibration(const PersistedCalibrationValues& calibrationValues) {
  Logging::message(Logging::Level::Info, "Calibration",
                   "Roll Zero Bias: %.2f, Pitch Zero Bias: %.2f",
                   calibrationValues.attitude.rollDegrees, calibrationValues.attitude.pitchDegrees);
  Logging::message(Logging::Level::Info, "Calibration",
                   "gyroBiasX: %.2f, gyroBiasY: %.2f, gyroBiasZ: %.2f",
                   calibrationValues.gyro.xRawCounts, calibrationValues.gyro.yRawCounts, calibrationValues.gyro.zRawCounts);
  Logging::message(Logging::Level::Info, "Calibration",
                   "servo1: %.2f, servo2: %.2f, servo3: %.2f, servo4: %.2f",
                   calibrationValues.servos.servo1Degrees, calibrationValues.servos.servo2Degrees,
                   calibrationValues.servos.servo3Degrees, calibrationValues.servos.servo4Degrees);
}

void configureWheelMotor(BLDCMotor& motor, const WheelPidParams& pid) {
  motor.torque_controller = TorqueControlType::estimated_current;
  motor.controller = MotionControlType::velocity;
  motor.motion_downsample = 0.0;

  motor.PID_velocity.P = pid.p;
  motor.PID_velocity.I = pid.i;
  motor.PID_velocity.D = pid.d;
  motor.PID_velocity.output_ramp = kWheelVelocityPidOutputRamp;
  motor.PID_velocity.limit = kWheelSupplyVoltageV;
  motor.LPF_velocity.Tf = kWheelVelocityFilterTfSec;

  motor.velocity_limit = kWheelVelocityLimitRadPerSec;
  motor.voltage_limit = kWheelSupplyVoltageV;
  motor.current_limit = kWheelCurrentLimitA;
  motor.phase_resistance = kWheelPhaseResistanceOhm;
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.modulation_centered = 1.0;
}

void initIMU(const imu::CalibrationValues& imuCalibration) {
  if (!imu::begin(imu::Settings{}, imuCalibration)) {
    Serial.println("IMU initialization failed!");
    while (1)
      ;
  }
  Serial.println("IMU initialized successfully!");
}

void initRemoteControl() {
  sBus.begin();  // TODO rename sBus to rc or similar
}

void initTouchAndFilters() {
  for (int i = 0; i < 6; i++)
    biquadFilterInitLPF(&BodyCommandFilterLPF[i], (unsigned int)bodyCommandFilterCutoffHz, 1000);

  biquadFilterInitLPF(&BodyCommandFilterLPF[8], 50, 1000);
  biquadFilterInitLPF(&BodyCommandFilterLPF[9], 50, 1000);
  biquadFilterInitLPF(&BodyCommandFilterLPF[10], 200, 1000);

  TouchscreenInit(1000);
}

void initPositionSensors() {
  I2Cone.begin(board::kI2cOneSdaPin, board::kI2cOneSclPin, 400000);
  I2Ctwo.begin(board::kI2cTwoSdaPin, board::kI2cTwoSclPin, 400000);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

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

void registerCommands() {
  command.add('T', handleM1MotionCommand, "motion1 control");  // Set motor target value
  command.add('M', handleM1Command, "motor1");

  command.add('A', zeroBias_servo1, "my zeroBias_servo1");  // Set servo 1 bias
  command.add('B', zeroBias_servo2, "my zeroBias_servo2");
  command.add('C', zeroBias_servo3, "my zeroBias_servo3");
  command.add('D', zeroBias_servo4, "my zeroBias_servo4");

  command.add('Z', setImuLowPassCutoff, "my ImuLPF_CUTOFF_FREQ");

  command.add('Q', TwoKp, "my TwoKp");  // MahonyFilter
  command.add('I', TwoKi, "my TwoKi");  // MahonyFilter

  command.add('K', KeyScalar, "my Select");
  command.add('E', KeyCalibration, "my CalibrationSelect");

  command.add('F', SetBodyCommandFilterCutoffHz, "my CutoffFreq");
  command.add('J', SetBodyCommandFilteringEnabled, "my EnableDFilter");

#if ADJUST_PARAMETER == ADJUST_BALANCE_SPEED_YAW_ROLL
  command.add('P', CbAnglePid, "anglePidSettings");
  command.add('S', CbSpeedPid, "speedPidSettings");
  command.add('Y', CbYawPid, "yawPidSettings");
  command.add('R', CbRollPid, "rollPidSettings");
  command.add('O', Target_Body_Height, "my Target_Body_Height");
#elif ADJUST_PARAMETER == ADJUST_BALL_PUSHING
  command.add('L', CbTouchXPid, "touchXPidSettings");
  command.add('N', CbTouchYPid, "touchYPidSettings");
  command.add('G', ControlTorqueCompensation, "my ControlTorqueCompensation");
#endif

  command.add('U', User_command, "my User_command");
  command.add('V', CbWheelSpeedFeedbackGain, "wheel speed feedback gain");
  command.add('W', CbDriveTiltReductionGain, "drive tilt reduction gain");
}

void setAssemblyPosition() {
  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);
}

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
  biquadFilterInitLPF(&VoltageFilterLPF, 20, kDiagnosticLogSampleRateHz);
  diagnosticImuReady = imu::begin(imu::Settings{}, imuCalibration);
  sBus.begin();
  Serial.printf("DIAG,boot,reset_reason=%d,imu_ok=%d,motors=off,servos=off\n",
                (int)esp_reset_reason(), diagnosticImuReady ? 1 : 0);
  Logging::setProfile(Logging::Profile::Diagnostic);
  return;
#endif

// TODO could be turned off?
  ble_init();
  xTaskCreatePinnedToCore(cpu0_task, "cpu0_task", 4096, NULL, 0, NULL, 0);

  biquadFilterInitLPF(&VoltageFilterLPF, 50, 1000); // TODO replace numbers with names

  //  Initialize servo
  servoControl.initialize();

  setAssemblyPosition();
  _delay(555);

  setAssemblyPosition();  // TODO why is this repeated?

  initIMU(imuCalibration);
  initRemoteControl();
  initTouchAndFilters();
  SimpleFOCDebug::enable(&Serial);  // TODO a more centralized way to enable debug sections would be nice
  initPositionSensors();
  initDrivers();
  configureWheelMotor(motor1, kWheelVelocityPid);
  configureWheelMotor(motor2, kWheelVelocityPid);

  motor1.init();
  motor2.init();

  // align encoder and start FOC
  motor1.initFOC();
  motor2.initFOC();

  motor1.target = 0;
  motor2.target = 0;

  motor1.useMonitoring(Serial);  // TODO logger lib usable here?
  motor1.monitor_downsample = 10;

  registerCommands();

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

constexpr RcRange kBleRcRanges[10] = {
    {BLE_CH0_MIN, BLE_CH0_MAX}, {BLE_CH1_MIN, BLE_CH1_MAX}, {BLE_CH2_MIN, BLE_CH2_MAX},
    {BLE_CH3_MIN, BLE_CH3_MAX}, {BLE_CH4_MIN, BLE_CH4_MAX}, {BLE_CH5_MIN, BLE_CH5_MAX},
    {BLE_CH6_MIN, BLE_CH6_MAX}, {BLE_CH7_MIN, BLE_CH7_MAX}, {BLE_CH8_MIN, BLE_CH8_MAX},
    {BLE_CH9_MIN, BLE_CH9_MAX}};

constexpr RcRange kSbusRcRanges[10] = {
    {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}, {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX},
    {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}, {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX},
    {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}, {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX},
    {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}, {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX},
    {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}, {SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX}};

constexpr long kSbusBodyHeightCenter = 992;

float bleBodyHeightM(long raw, const RcRange& range, float maxHeightM) {
  return mapf(raw, range.min, range.max, kBodyHeightDefaultM, maxHeightM);
}

// Below the stick centre the height falls to the minimum, above it rises to maxHeightM.
float sbusBodyHeightM(long raw, const RcRange& range, float maxHeightM) {
  if (raw <= kSbusBodyHeightCenter)
    return mapf(raw, range.min, kSbusBodyHeightCenter, kBodyHeightMinM, kBodyHeightDefaultM);
  return mapf(raw, kSbusBodyHeightCenter + 1, range.max, kBodyHeightDefaultM, maxHeightM);
}

/**
 * @brief Maps ten raw remote channels to the body commands and mode switches.
 *
 * Channel 1 drives the body height or the pitching, depending on `attitude_mode`.
 * The two receivers differ only in their raw ranges and in the body height curve.
 */
template <typename Raw>
void applyRcChannels(const Raw *ch, const RcRange *range,
                     float (*bodyHeightM)(long, const RcRange&, float)) {
  MovementSpeed = mapf(ch[2], range[2].min, range[2].max, -15, 15);
  BodyTurn = -mapf(ch[3], range[3].min, range[3].max, -11, 11);
  pid_gains_mode = map(ch[4], range[4].min, range[4].max, 0, 2);
  posture_or_mark_mode = map(ch[5], range[5].min, range[5].max, 0, 1);
  roll_mode = map(ch[6], range[6].min, range[6].max, 0, 1);
  attitude_mode = map(ch[7], range[7].min, range[7].max, 0, 2);
  // Top ball
  top_ball_x = mapf(ch[8], range[8].min, range[8].max, -5, 5);
  top_ball_y = mapf(ch[9], range[9].min, range[9].max, -5, 5);

  if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT) {
    BodyHeightCommandM = bodyHeightM(ch[1], range[1], kBodyHeightMaxM);
  } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST) {
    BodyPitching = mapf(ch[1], range[1].min, range[1].max, -12, 12);
  } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) {
    BodyHeightCommandM = bodyHeightM(ch[1], range[1], kBallPoiseMaxBodyHeightM);
  }
  BodyRoll = mapf(ch[0], range[0].min, range[0].max, -0.011, 0.011);
}

/**
 * @brief Choose BLE or remote control input
 *
 * Use the remote control as the input when BLE connection is lost.
 */
void CtrlInput() {
  if (rp.ble_connected) {
    bleCtrl();
  } else {
    RXsbus();
  }
}

void bleCtrl() {
  applyRcChannels(ble_ctrler.ch, kBleRcRanges, bleBodyHeightM);
}

/**
 * @brief Reads and processes data from the FUTABA S.BUS remote controller.
 *
 * This function decodes the S.BUS signal, maps the raw channel values to
 * meaningful control variables like `MovementSpeed`, `BodyTurn`, `BodyHeightCommandM`,
 * and `BodyPitching`, and updates global state based on the RC switch positions.
 * While frames are missing for `kRcFrameTimeoutMs` or the receiver reports a
 * failsafe, `MovementSpeed` and `BodyTurn` are forced to zero and every other
 * command keeps its last received value.
 */
void RXsbus() {
  static uint32_t lastFrameMs = millis();

  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    const uint32_t frameMs = millis();
    sbusFrameIntervalMs = frameMs - lastFrameMs;
    lastFrameMs = frameMs;
    sBus.toChannels = 0;
    sBus.UpdateChannels();
    sBus.toChannels = 0;

    applyRcChannels(sBus.channels, kSbusRcRanges, sbusBodyHeightM);
    if (DIAGNOSTIC_LIVE_TUNING_DEFAULTS)
      MovementSpeed *= 2.0f;  // Keep drive-command authority near the previous 0.03 x 3.3 setting.
  }

  const bool lost = static_cast<uint32_t>(millis() - lastFrameMs) > kRcFrameTimeoutMs ||
                    sBus.Failsafe() != FUTABA_SBUS::kSignalOk;
  if (lost) {
    MovementSpeed = 0;
    BodyTurn = 0;
  }
}

DebugTelemetry::PidStatus pidStatus(const MyPIDController& pid) {
  return {pid.Kp,   pid.Ki,   pid.Kd,   pid.deriv, pid.integral, pid.iLimit,
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
  in.balanceActive = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_NOT_TUMBLING;

  in.imu = imu::latest();
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
  in.sensor1Angle = sensor1.getAngle();
  in.sensor1MechanicalAngle = sensor1.getMechanicalAngle();
  in.sensor2Angle = sensor2.getAngle();
  in.sensor2MechanicalAngle = sensor2.getMechanicalAngle();

  for (int i = 0; i < 18; ++i) in.sbusChannels[i] = sBus.channels[i];
  in.sbusFrameIntervalMs = sbusFrameIntervalMs;

  in.anglePid = pidStatus(Angle_Pid);
  in.speedPid = pidStatus(Speed_Pid);
  in.yawPid = pidStatus(Yaw_Pid);
  in.rollPid = pidStatus(Roll_Pid);
  in.touchXPid = pidStatus(TouchX_Pid);
  in.touchYPid = pidStatus(TouchY_Pid);
  in.touch = {Touch.XPressDat, Touch.YPressDat, Touch.XPdat,  Touch.YPdat, Touch.XLdat,
              Touch.YLdat,     Touch.XPdatF,    Touch.YPdatF, Touch.state, Touch.start};

  in.servoTrims = servoTrims;
  for (int i = 0; i < 4; ++i) in.servoTraceAngle[i] = servoTraceAngle[i];

  in.topBallX = top_ball_x;
  in.topBallY = top_ball_y;
  in.sbusTopBallXSmoothed = sbus_top_ball_x_smoothed;
  in.sbusTopBallYSmoothed = sbus_top_ball_y_smoothed;

  in.bodyRoll = BodyRoll;
  in.bodyPitching = BodyPitching;
  in.bodyPitchingFiltered = BodyPitchingFiltered;
  in.bodyRollFiltered = BodyRollFiltered;
  in.bodyPitchCorrection = BodyPitchingCorrect(BodyPitchingFiltered);
  in.bodyHeightCommandM = BodyHeightCommandM;
  in.bodyHeightCommandFilteredM = BodyHeightCommandFilteredM;
  in.bodyTurn = BodyTurn;
  in.bodyX = BodyX;
  in.bodyCommandFilteringEnabled = bodyCommandFilteringEnabled;
  in.bodyCommandFilterCutoffHz = bodyCommandFilterCutoffHz;

  in.movementSpeed = MovementSpeed;
  in.driveEffectiveSpeed = driveEffectiveSpeed;
  in.driveSpeedBodyXRaw = driveSpeedBodyXRaw;
  in.wheelVelocityFeedbackCorrection = wheelVelocityFeedbackCorrection;

  in.voltageAdc = VoltageADC;
  in.voltageAdcFiltered = VoltageADCf;
  in.voltage = Voltage;

  in.pidGainsMode = pid_gains_mode;
  in.postureOrMarkMode = posture_or_mark_mode;
  in.robotTumble = RobotTumble;
  in.pidParameterTuning = PidParameterTuning;
  in.targetBodyHeightOverrideM = TargetBodyHeightOverrideM;
  return in;
}

/**
 * @brief Applies a quadratic correction to the body pitching value.
 *
 * Computes y = 4E-06 x^2 + 0.0004 x - 0.0008.
 *
 * @param x The raw body pitching value.
 * @return The corrected body pitching value.
 */
float BodyPitchingCorrect(float x)  // Pitch angle correction
{
  float y = 0.000004 * x * x + 0.0004 * x - 0.0008;
  return y;
}

void applyPidPreset(PIDController& pid, const PidPreset& preset) {
  pid.P = preset.p;
  pid.I = preset.i;
  pid.D = preset.d;
  pid.limit = preset.limit;
}

void applyBalancePidPreset(const BalancePidPreset& preset) {
  applyPidPreset(rollPidSettings, preset.roll);
  applyPidPreset(speedPidSettings, preset.speed);
  applyPidPreset(anglePidSettings, preset.angle);
}

/**
 * @brief Sets the PID gains for the main controllers based on the S.BUS switch `pid_gains_mode`.
 *
 * This allows for switching between two sets of PID parameters on the fly using the
 * remote controller. One set is tuned for operation without the touchscreen, and
 * the other for operation with the touchscreen, which may change the robot's
 * weight distribution and dynamics.
 */
void PidParameter(void) {
#if DIAGNOSTIC_LIVE_TUNING_DEFAULTS
  const int gainMode = REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH;
#else
  const int gainMode = pid_gains_mode;
#endif
  if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH)
    applyBalancePidPreset(kPidWithoutTouch);
  else if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH)
    applyBalancePidPreset(kPidWithTouch);

  applyPidPreset(yawPidSettings, kYawPid);
  applyPidPreset(touchXPidSettings, kTouchXPid);
  applyPidPreset(touchYPidSettings, kTouchYPid);
}

constexpr int kTouchPressFrames = 4;
constexpr int kTouchLiftFrames = 44;
constexpr int kTouchKdRampDivisor = 22;
constexpr int kTouchDerivClamp = 11000;

/**
 * @brief Main PID control loop for two-wheel balancing and posture control.
 *
 * This is the core control function for the two-wheeled mode. It orchestrates
 * multiple PID controllers to achieve stable balancing and respond to commands.
 * - It runs PID loops for the touchscreen input to generate `BodyPitching` and roll commands.
 * - It runs a PID loop for roll stabilization.
 * - It runs a cascade PID for speed and balance control (Speed loop -> Angle loop).
 * - It runs a PID loop for yaw (turning) control.
 * The outputs are summed to produce the final `target` values for each motor.
 *
 * @param dt The time delta since the last call, in seconds.
 */
void PIDcontroller_posture(float dt) {
  if ((int)PidParameterTuning == 0)
    PidParameter();

  // Touch screen
  TouchX_Pid.Kp = touchXPidSettings.P / 100;
  TouchX_Pid.Ki = touchXPidSettings.I / 100;
  TouchX_Pid.Kd = touchXPidSettings.D / 100;
  TouchX_Pid.iLimit = touchXPidSettings.limit;  // Integral limit

  TouchY_Pid.Kp = touchYPidSettings.P / 100;
  TouchY_Pid.Ki = touchYPidSettings.I / 100;
  TouchY_Pid.Kd = touchYPidSettings.D / 100;
  TouchY_Pid.iLimit = touchYPidSettings.limit;  // Integral limit

  float touchXError = Touch.XPdatF;
  float touchYError = Touch.YPdatF;
  static float TouchX_kd = 0;
  static float TouchY_kd = 0;

  TouchX_Pid.compute(touchXError, dt);
  TouchY_Pid.compute(touchYError, dt);

  TouchX_Pid.deriv = constrain(TouchX_Pid.deriv, -kTouchDerivClamp, kTouchDerivClamp);
  TouchX_Pid.outD = TouchX_kd * TouchX_Pid.deriv;

  if ((attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) || (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO))  // Top ball mode
  {
    BodyPitching = TouchX_Pid.outP + TouchX_Pid.outI + TouchX_Pid.outD + sbus_top_ball_x_smoothed;
    BodyPitching = -BodyPitching;
  }

  TouchY_Pid.deriv = constrain(TouchY_Pid.deriv, -kTouchDerivClamp, kTouchDerivClamp);
  TouchY_Pid.outD = TouchY_kd * TouchY_Pid.deriv;
  TouchY_Pid.output = TouchY_Pid.outP + TouchY_Pid.outI + TouchY_Pid.outD + sbus_top_ball_y_smoothed;
  if ((int)bodyCommandFilteringEnabled == 1) {
    TouchYPidOutputFiltered = biquadFilterApply(&BodyCommandFilterLPF[10], TouchY_Pid.output);
  } else {
    TouchYPidOutputFiltered = TouchY_Pid.output;
  }

  if ((Touch.state == 1) && (Touch.P_count < kTouchPressFrames)) {
    Touch.P_count++;
    TouchX_Pid.integral = 0;
    TouchY_Pid.integral = 0;
  }

  if (Touch.P_count >= kTouchPressFrames) {
    if (TouchX_kd < TouchX_Pid.Kd) {
      TouchX_kd = TouchX_kd + (TouchX_Pid.Kd / kTouchKdRampDivisor);
    }
    if (TouchY_kd < TouchY_Pid.Kd) {
      TouchY_kd = TouchY_kd + (TouchY_Pid.Kd / kTouchKdRampDivisor);
    }
    Touch.start = 2;
  }

  if ((Touch.state == 0) && (Touch.L_count < kTouchLiftFrames))
    Touch.L_count++;
  else
    Touch.L_count = 0;

  if (Touch.L_count >= kTouchLiftFrames) {
    TouchX_Pid.integral = 0;
    TouchY_Pid.integral = 0;
    Touch.P_count = 0;
    TouchX_kd = 0;
    TouchY_kd = 0;
    Touch.start = -2;
  }

  if ((attitude_mode != REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) || (roll_mode != REMOTE_CONTROL_ROLL_MODE_AUTO))  // Non-top ball mode
  {
    TouchX_Pid.integral = 0;
    TouchX_Pid.output = 0;
    TouchY_Pid.integral = 0;
    TouchY_Pid.output = 0;
    TouchYPidOutputFiltered = 0;
    TouchX_kd = 0;
    TouchY_kd = 0;
  }

  // Roll
  Roll_Pid.Kp = rollPidSettings.P / 100;
  Roll_Pid.Ki = rollPidSettings.I / 100;
  Roll_Pid.Kd = rollPidSettings.D / 100;
  Roll_Pid.iLimit = rollPidSettings.limit;  // Integral limit

  float TargetBodyRoll = BodyRollFiltered * 777;                            // Roll
  if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)  // Top ball mode disables manual roll
    TargetBodyRoll = 0;
  float RollError = (-imu::latest().correctedPitchDegrees) - (-TargetBodyRoll) - (-TouchYPidOutputFiltered);
  if (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO)  // Roll leveling
  {
    Roll_Pid.compute(RollError, dt);
  } else {
    Roll_Pid.output = 0;
    Roll_Pid.integral = 0;
  }

  // Speed loop
  Speed_Pid.Kp = speedPidSettings.P / 100;
  Speed_Pid.Ki = speedPidSettings.I / 100;
  Speed_Pid.Kd = speedPidSettings.D / 100;
  Speed_Pid.iLimit = speedPidSettings.limit;  // Integral limit

  const float avgVelocity = 0.5f * (m1FilteredVelocityRadPerSec + m2FilteredVelocityRadPerSec);
  driveEffectiveSpeed = MovementSpeed;
  // Reduce only further acceleration when tilt consumes balance headroom.
  if (MovementSpeed * avgVelocity >= 0.0f && fabsf(MovementSpeed) > fabsf(avgVelocity)) {
    const float tiltFraction = constrain((fabsf(imu::latest().correctedRollDegrees) - kDriveTiltReductionStartDeg) /
                                         (kDriveTiltReductionFullDeg - kDriveTiltReductionStartDeg),
                                         0.0f, 1.0f);
    const float reduction = tiltFraction * constrain(driveTiltReductionGain, 0.0f, 1.0f);
    driveEffectiveSpeed = avgVelocity + (MovementSpeed - avgVelocity) * (1.0f - reduction);
  }
  const float speedError = avgVelocity - driveEffectiveSpeed;

  const float previousSpeedIntegral = Speed_Pid.integral;
  driveSpeedBodyXRaw = Speed_Pid.compute(speedError, dt);
  if (Speed_Pid.Ki != 0.0f && fabsf(driveSpeedBodyXRaw) > kDriveBodyXLimitM &&
      driveSpeedBodyXRaw * speedError > 0.0f) {
    Speed_Pid.integral = previousSpeedIntegral;
    Speed_Pid.outI = Speed_Pid.Ki * previousSpeedIntegral;
    driveSpeedBodyXRaw = Speed_Pid.outP + Speed_Pid.outI + Speed_Pid.outD;
    Speed_Pid.output = driveSpeedBodyXRaw;
  }
  const float speedBodyX = constrain(driveSpeedBodyXRaw, -kDriveBodyXLimitM, kDriveBodyXLimitM);

  BodyX = speedBodyX + BodyPitchingCorrect(BodyPitchingFiltered);

  // Balance loop
  Angle_Pid.Kp = anglePidSettings.P;
  Angle_Pid.Ki = anglePidSettings.I;
  Angle_Pid.Kd = anglePidSettings.D;
  Angle_Pid.iLimit = anglePidSettings.limit;  // Integral limit

  float angleError = imu::latest().correctedRollDegrees - (-BodyPitchingFiltered);  // Measured value minus target value
  if (balancePidNeedsPriming) {
    // Avoid a derivative kick when CH5 first enables the balance loop.
    Angle_Pid.previousError = angleError;
    Angle_Pid.integral = 0;
    balancePidNeedsPriming = false;
  }
  float angleOutput = Angle_Pid.compute(angleError, dt);

  // Yaw loop
  Yaw_Pid.Kp = yawPidSettings.P;
  Yaw_Pid.Ki = yawPidSettings.I;
  Yaw_Pid.Kd = yawPidSettings.D;
  Yaw_Pid.iLimit = yawPidSettings.limit;
  if (roll_mode != REMOTE_CONTROL_ROLL_MODE_AUTO)  // Lock heading angle
  {
    Yaw_Pid.Ki = 0;
    Yaw_Pid.integral = 0;
  }

  float yawError = imu::latest().angularVelocityRadPerSec.z - BodyTurn;  // Measured value minus target value
  float yawOutput = Yaw_Pid.compute(yawError, dt);

  // Use the existing wheel path only to oppose measured overspeed.
  const float boundedWheelGain = constrain(wheelSpeedFeedbackGain, 0.0f, 0.4f);
  const bool braking = avgVelocity * (avgVelocity - MovementSpeed) > 0.0f;
  const float wheelCorrection = braking ? boundedWheelGain * (avgVelocity - MovementSpeed) : 0.0f;
  wheelVelocityFeedbackCorrection = constrain(wheelCorrection, -kDriveWheelFeedbackLimit,
                                      kDriveWheelFeedbackLimit);
  float target1 = angleOutput - yawOutput + wheelVelocityFeedbackCorrection;
  float target2 = angleOutput + yawOutput + wheelVelocityFeedbackCorrection;

  if (control_torque_compensation != 0) {
    if (target1 > 0)
      target1 = target1 + control_torque_compensation;
    else if (target1 < 0)
      target1 = target1 + (-control_torque_compensation);

    if (target2 > 0)
      target2 = target2 + control_torque_compensation;
    else if (target2 < 0)
      target2 = target2 + (-control_torque_compensation);
  }

  // SimpleFOC's velocity mode does not apply velocity_limit to its target.
  motor1.target = constrain(target1, -motor1.velocity_limit, motor1.velocity_limit);
  motor2.target = constrain(target2, -motor2.velocity_limit, motor2.velocity_limit);
}

/**
 * @brief Applies a low-pass filter to the remote control input values.
 *
 * This function smooths the raw values received from the S.BUS controller
 * (`BodyPitching`, `BodyRoll`, `BodyHeightCommandM`, etc.) using biquad low-pass filters.
 * This prevents jerky movements and improves the stability of the robot's response
 * to user commands. The filter cutoff frequency can be adjusted live.
 */
void RemoteControlFiltering(void)  // Remote control filter
{
  static int bodyCommandFilteringEnabled_last = (int)bodyCommandFilteringEnabled;
  static int bodyCommandFilterCutoffHz_last = (int)bodyCommandFilterCutoffHz;

  sbus_top_ball_x_smoothed = biquadFilterApply(&BodyCommandFilterLPF[8], top_ball_x);
  sbus_top_ball_y_smoothed = biquadFilterApply(&BodyCommandFilterLPF[9], top_ball_y);

  if ((int)bodyCommandFilteringEnabled == 1) {
    BodyPitchingFiltered = biquadFilterApply(&BodyCommandFilterLPF[0], BodyPitching);
    BodyRollFiltered = biquadFilterApply(&BodyCommandFilterLPF[1], BodyRoll);
    BodyHeightCommandFilteredM = biquadFilterApply(
        &BodyCommandFilterLPF[kBodyHeightCommandFilterIndex], BodyHeightCommandM);
  } else {
    BodyPitchingFiltered = BodyPitching;
    BodyRollFiltered = BodyRoll;
    BodyHeightCommandFilteredM = BodyHeightCommandM;
  }

  if (((int)bodyCommandFilteringEnabled != bodyCommandFilteringEnabled_last) ||
      ((int)bodyCommandFilterCutoffHz != bodyCommandFilterCutoffHz_last)) {
    for (int i = 0; i < 6; i++) {
      biquadFilterInitLPF(&BodyCommandFilterLPF[i], (unsigned int)bodyCommandFilterCutoffHz, 1000);
    }

    bodyCommandFilteringEnabled_last = (int)bodyCommandFilteringEnabled;
    bodyCommandFilterCutoffHz_last = (int)bodyCommandFilterCutoffHz;
    Logging::message(Logging::Level::Info, "Filter", "ok");
  }
}

/**
 * @brief Reads and filters the battery voltage.
 *
 * Reads the analog value from the voltage divider, applies a low-pass filter
 * to get a stable reading, and converts it to the actual voltage.
 */
void ReadVoltage(void) {
  VoltageADC = analogRead(board::kBatteryAdcPin);
  if (debugTelemetryState.voltageAdcMin == 0 || VoltageADC < debugTelemetryState.voltageAdcMin)
    debugTelemetryState.voltageAdcMin = VoltageADC;
  VoltageADCf = biquadFilterApply(&VoltageFilterLPF, VoltageADC);
  Voltage = board::kBatteryVoltsPerAdcCount * VoltageADCf;
}

/**
 * @brief Detects if the robot has fallen over.
 *
 * If the absolute roll angle exceeds a threshold (35 degrees) for a certain
 * number of consecutive cycles, it sets the `RobotTumble` flag to 1. The flag
 * is reset to 0 only after the robot is brought back to a near-level position.
 * This is a safety feature to disable motors upon falling.
 */
void Robot_Tumble(void) {
  static int x = 0;
  if (fabsf(imu::latest().correctedRollDegrees) >= 35) {
    x++;
    if (x >= 20) {
      x = 20;
      RobotTumble = ROBOT_TUMBLING;  // Machine fall
    }
  } else {
    if ((RobotTumble == ROBOT_TUMBLING) && (fabsf(imu::latest().correctedRollDegrees) <= 5))  // Machine fall after fall
    {
      x--;
      if (x <= 0) {
        x = 0;
        RobotTumble = ROBOT_NOT_TUMBLING;
      }
    }
  }
}

void DiagnosticLoop(void) {
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
    const imu::Snapshot& diagnosticImu = imu::latest();
    const uint32_t imuSampleUs = diagnosticImu.readTimeUs;
    if ((uint32_t)(imuSampleUs - lastLogScheduleUs) >= DIAGNOSTIC_LOG_INTERVAL_US) {
      lastLogScheduleUs = imuSampleUs;
      const long rcAgeMs = diagnosticHasRcFrame ? (long)(nowMs - diagnosticLastRcFrameMs) : -1;
      const int rcFailsafe = diagnosticHasRcFrame ? sBus.Failsafe() : -1;
      const float batteryRawV = board::kBatteryVoltsPerAdcCount * VoltageADC;
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
      const float acceptedRateHz = static_cast<float>(acceptedSamples) * 1000000.0f /
                                   static_cast<float>(elapsedUs);
      Serial.printf("IMU_TIMING,elapsed_us=%lu,accepted=%lu,rate_hz=%.1f,long_gap_total=%lu,max_interval_us=%lu\n",
                    static_cast<unsigned long>(elapsedUs),
                    static_cast<unsigned long>(acceptedSamples), acceptedRateHz,
                    static_cast<unsigned long>(timing.longGapCount),
                    static_cast<unsigned long>(timing.maxIntervalUs));
      lastTimingReportUs = timingNowUs;
      lastTimingAcceptedSamples = timing.acceptedSamples;
    }
  }

  if (nowMs - lastVoltageMs >= 10) {
    lastVoltageMs = nowMs;
    ReadVoltage();
  }
}
void updateMotorsAndCommands() {
  // iterative function setting the outer loop target
  if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO) {
    if (Logging::profile() == Logging::Profile::Idle) motor1.monitor();
  } else if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS) {
    motor2.target = motor1.target;
  }

  motor1.move();
  motor2.move();

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();

  // user communication
  command.run();
}

/**
 * @return True when the IMU accepted a new sample in this call.
 */
bool readInputs() {
  const bool imuUpdated = imu::service();  // Poll once per loop without gating motor control
  CtrlInput();  // BLE or remote control input
  ReadTouchDat();
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
  if (Voltage <= board::kLowBatteryVolts) {
    statusLedPeriodTicks = 20;
    if (static_cast<uint32_t>(millis() - lastLowBatteryWarningMs) >= 1000) {
      lastLowBatteryWarningMs = millis();
      Logging::message(Logging::Level::Warning, "Voltage", "%.5f", Voltage);
    }
  } else {
    statusLedPeriodTicks = 100;
  }
}

void runCalibrationStep(bool imuUpdated) {
  imu::CalibrationValues activeCalibration = imu::calibration();
  const imu::Snapshot& acceptedImu = imu::latest();
  CalibrationResult calibrationResult{CalibrationCompletion::None, false, {false, false, false, false}};
  if (static_cast<int>(CalibrationSelect) == 1 &&
      pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_NOT_TUMBLING) {
    CalibrationSelect = 0;
    Logging::message(Logging::Level::Warning, "Calibration", "gyro calibration refused: balance active");
  }
  if (imuUpdated) calibrationResult = calibration.update(
      static_cast<int>(CalibrationSelect),
      {acceptedImu.attitudeDegrees.roll, acceptedImu.attitudeDegrees.pitch},
      calibrationStore,
      {activeCalibration.attitude.rollDegrees, activeCalibration.attitude.pitchDegrees},
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
  if (calibrationResult.clearRequest)
    CalibrationSelect = 0;

  if (calibrationResult.completion == CalibrationCompletion::Gyroscope) {
    Logging::message(Logging::Level::Info, "Calibration",
                     "gyroBiasX: %.2f, gyroBiasY: %.2f, gyroBiasZ: %.2f",
                     activeCalibration.gyro.xRawCounts, activeCalibration.gyro.yRawCounts, activeCalibration.gyro.zRawCounts);
  } else if (calibrationResult.completion == CalibrationCompletion::Attitude) {
    Logging::message(Logging::Level::Info, "Calibration",
                     "Roll Zero Bias: %.2f, Pitch Zero Bias: %.2f",
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
  const float wheelVelocityDt = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? controlTimestepSec : 0.01f;
  m1VelocityRadPerSec = (sensor1.getAngle() - m1PrevEncoderAngleRad) / wheelVelocityDt;
  m1FilteredVelocityRadPerSec = m1VelocityFilter(m1VelocityRadPerSec);
  m1PrevEncoderAngleRad = sensor1.getAngle();

  m2VelocityRadPerSec = -(sensor2.getAngle() - m2PrevEncoderAngleRad) / wheelVelocityDt;
  m2FilteredVelocityRadPerSec = m2VelocityFilter(m2VelocityRadPerSec);
  m2PrevEncoderAngleRad = sensor2.getAngle();
}

/**
 * @brief Runs the balance controller when enabled and returns the body height and roll offset.
 *
 * With balance off or the robot tumbling it zeroes the wheel targets, the body offsets and the
 * controller integrals instead.
 */
BodyPose computeBodyPose() {
  BodyPose pose{kBodyHeightDefaultM, BodyRollFiltered};

  if ((pid_gains_mode == REMOTE_CONTROL_PID_GAINS_MODE_OFF) || (RobotTumble == ROBOT_TUMBLING)) {
    balancePidNeedsPriming = true;
    if (COMMUNICATION_OBJECT == COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE) {
      motor1.target = 0;
      motor2.target = 0;
    }

    pose.heightM = kBodyHeightDefaultM;
    BodyX = 0;
    pose.roll = 0;
    BodyPitchingFiltered = 0;

    Angle_Pid.integral = 0;
    Speed_Pid.integral = 0;
    Yaw_Pid.integral = 0;
    wheelVelocityFeedbackCorrection = 0;
    driveEffectiveSpeed = 0;
    driveSpeedBodyXRaw = 0;

  } else if ((pid_gains_mode_is_enabled(pid_gains_mode)) && (RobotTumble == ROBOT_NOT_TUMBLING)) {
    PIDcontroller_posture(controlTimestepSec);  // PID controller

    if (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO)
      pose.roll = Roll_Pid.output;

    if (TargetBodyHeightOverrideM == kBodyHeightOverrideDisabledM)
      pose.heightM = BodyHeightCommandFilteredM;
    else
      pose.heightM = TargetBodyHeightOverrideM;
  }

  if (RobotTumble == ROBOT_TUMBLING)  // Machine fall
  {
    pose.heightM = kBodyHeightDefaultM;
    BodyX = 0;
    pose.roll = 0;
    BodyPitchingFiltered = 0;
  }

  return pose;
}

void warnIfLegUnreachable(const LegSolveResult& leg, const char *tag, uint32_t& lastWarningMs) {
  if (leg.status != LegSolveStatus::Success && static_cast<uint32_t>(millis() - lastWarningMs) >= 1000) {
    lastWarningMs = millis();
    Logging::message(Logging::Level::Warning, tag, "no");
  }
}

void writeLegServos() {
  servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1, servoTraceAngle[3], 1);
}

void writeServos(const LegSolveResult& right, const LegSolveResult& left) {
  if (posture_or_mark_mode == REMOTE_CONTROL_PM_POSTURE_MODE)  // Posture
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
    if ((int)CalibrationSelect == 3)  // Servo calibration
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
  controlGateSequence++;
  TouchBiquadFilter();  // Touch screen filter

  RemoteControlFiltering();  // Remote control signal filtering
  ReadVoltage();             // Battery
  DebugTelemetry::submit(collectTelemetryInputs(), debugTelemetryState);  // Serial port data printing

  Robot_Tumble();  // Machine fall detection
  updateStatusLed();
  runCalibrationStep(imuUpdated);
  updateWheelVelocities();

  const BodyPose pose = computeBodyPose();

  static uint32_t lastRightLegWarningMs = 0;
  static uint32_t lastLeftLegWarningMs = 0;
  const LegSolveResult rightLeg = LegKinematics::solveRight(
      {-BodyX, pose.heightM - pose.roll, BodyPitchingFiltered});
  warnIfLegUnreachable(rightLeg, "RightInverseKinematics", lastRightLegWarningMs);

  const LegSolveResult leftLeg = LegKinematics::solveLeft(
      {-BodyX, pose.heightM + pose.roll, BodyPitchingFiltered});
  warnIfLegUnreachable(leftLeg, "LeftInverseKinematics", lastLeftLegWarningMs);

  writeServos(rightLeg, leftLeg);

  previousControlTimeUs = loopTimeUs;
}

/**
 * @brief The main execution loop of the program.
 *
 * This function runs repeatedly after `setup()` is complete:
 * 1. `updateMotorsAndCommands`: SimpleFOC `move()` and `loopFOC()` for both wheels, then Commander input.
 * 2. `readInputs`: IMU sample, BLE or S.BUS commands, touchscreen.
 * 3. `runControlTick`, once at least 1 ms has passed since the previous tick: input filtering,
 *    battery reading, telemetry, fall detection, status LED, calibration, wheel velocities,
 *    posture control, leg kinematics and servo output.
 */
void loop() {
#if SENSOR_DIAGNOSTIC_MODE
  DiagnosticLoop();
  return;
#endif
  loopTimeUs = micros();

  updateMotorsAndCommands();
  const bool imuUpdated = readInputs();

  controlTimestepSec = (loopTimeUs - previousControlTimeUs) / 1000000.0f;
  if (controlTimestepSec >= kControlTimestepMinSec) {
    runControlTick(imuUpdated);
  }
}
