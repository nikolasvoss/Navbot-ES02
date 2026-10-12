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
#define Communication_object COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE          // 0: two-wheel balance  1: simpleFOC Studio host computer  2: control dual motors

// Safe bring-up: read sensors and receiver without energizing wheels or servos.
// Set to 0 only after the diagnostic readings and power supply are checked.
#define SENSOR_DIAGNOSTIC_MODE 0

// Tuned two-wheel drive defaults, including corrected wheel-speed timing and
// CH3 scaling. Apply them at startup and through a CH5 switch transition.
// Set to 0 to restore the repository's mode-dependent defaults.
#define DIAGNOSTIC_LIVE_TUNING_DEFAULTS 1

#define AdjusParameter ADJUST_BALANCE_SPEED_YAW_ROLL        // 0: balance, speed, yaw, roll parameter tuning   1: ball pushing

//      Two-Wheel PID Gains for Remote Control Mode (Without Touchscreen)
#define PID_ROLL_P_NO_TOUCH 0.06
#define PID_ROLL_I_NO_TOUCH 0.0
#define PID_ROLL_D_NO_TOUCH 0.0
#define PID_ROLL_LIMIT_NO_TOUCH 2
#define PID_SPEED_P_NO_TOUCH 0.045
#define PID_SPEED_I_NO_TOUCH 0.005
#define PID_SPEED_D_NO_TOUCH 0
#define PID_SPEED_LIMIT_NO_TOUCH 50
#define PID_ANGLE_P_NO_TOUCH 5
#define PID_ANGLE_I_NO_TOUCH 200
#define PID_ANGLE_D_NO_TOUCH 0.11
#define PID_ANGLE_LIMIT_NO_TOUCH 0.1

//      Two-Wheel PID Gains for Remote Control Mode (With Touchscreen)
#define PID_ROLL_P_WITH_TOUCH 0.08
#define PID_ROLL_I_WITH_TOUCH 1.5
#define PID_ROLL_D_WITH_TOUCH 0.005
#define PID_ROLL_LIMIT_WITH_TOUCH 2
#define PID_SPEED_P_WITH_TOUCH 0.045
#define PID_SPEED_I_WITH_TOUCH 0.005
#define PID_SPEED_D_WITH_TOUCH 0
#define PID_SPEED_LIMIT_WITH_TOUCH 50
#define PID_ANGLE_P_WITH_TOUCH 5
#define PID_ANGLE_I_WITH_TOUCH 200
#define PID_ANGLE_D_WITH_TOUCH 0.11
#define PID_ANGLE_LIMIT_WITH_TOUCH 0.1
#define PID_YAW_P_DEFAULT 4
#define PID_YAW_I_DEFAULT 0
#define PID_YAW_D_DEFAULT 0
#define PID_YAW_LIMIT_DEFAULT 0
#define TOUCH_X_PID_P_DEFAULT 0.2
#define TOUCH_X_PID_I_DEFAULT 0
#define TOUCH_X_PID_D_DEFAULT 0.04
#define TOUCH_X_PID_LIMIT_DEFAULT 0
#define TOUCH_Y_PID_P_DEFAULT 0.2
#define TOUCH_Y_PID_I_DEFAULT 170
#define TOUCH_Y_PID_D_DEFAULT 0.08
#define TOUCH_Y_PID_LIMIT_DEFAULT 0

#define SBUS_CHANNEL_MAX 1792
#define SBUS_CHANNEL_MIN 192
constexpr uint32_t kRcFrameTimeoutMs = 100;

#define SERIAL_BAUD_RATE 576000
#define DIAGNOSTIC_SERIAL_BAUD_RATE 576000
#define LIVE_TUNING_SERIAL_BAUD_RATE 576000
#define DIAGNOSTIC_LOG_INTERVAL_US 5000 // Diagnostic CSV rows are emitted at 200 Hz.
constexpr unsigned int DIAGNOSTIC_LOG_SAMPLE_RATE_HZ = 1000000U / DIAGNOSTIC_LOG_INTERVAL_US;
// Conservative drive tuning parameters; verify the wheel feedback sign on hardware.
constexpr float DRIVE_BODY_X_LIMIT_M = 0.010f;
constexpr float DRIVE_WHEEL_FEEDBACK_LIMIT = 8.0f;
constexpr float DRIVE_TILT_REDUCTION_START_DEG = 5.0f;
constexpr float DRIVE_TILT_REDUCTION_FULL_DEG = 10.0f;
// -------------------------------------

constexpr float BODY_HEIGHT_OVERRIDE_DISABLED_M = 0.0f;
constexpr float BODY_HEIGHT_MIN_M = 0.05f;
constexpr float BODY_HEIGHT_DEFAULT_M = 0.06f;
constexpr float BALL_POISE_MAX_BODY_HEIGHT_M = 0.07f;
constexpr float BODY_HEIGHT_MAX_M = 0.09f;
constexpr int BODY_HEIGHT_COMMAND_FILTER_INDEX = 2;

// Body
float TargetBodyHeightOverrideM = BODY_HEIGHT_OVERRIDE_DISABLED_M;  // Zero disables the Commander height override.
float BodyHeightCommandM = BODY_HEIGHT_DEFAULT_M;  // Remote-controlled body height
float BarycenterX = 0;              // Center of mass X
float BodyPitching = 0;             // Pitch
float BodyRoll = 0;                 // Roll
float MovementSpeed = 0;            // Movement speed
float BodyTurn = 0;                 // Turning
float SlideStep = 0;                // Slide step
float BodyX = 0;                    // X position (controller output)
int RobotTumble = ROBOT_NOT_TUMBLING;  // Fall-detection state, see docs/robot/ROBOT_ROADMAP.md


// 滤波
float BodyHeightCommandFilteredM = BODY_HEIGHT_DEFAULT_M;
float BodyPitchingFiltered = 0;      // Pitch
float BodyRollFiltered = 0;          // Roll
float SlideStepFiltered = 0;         // Slide step
biquadFilter_t BodyCommandFilterLPF[12];  // Second-order low-pass filter
float TouchYPidOutputFiltered = 0;
float TouchXPidOutputFiltered = 0;

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
uint16_t VoltageADCMin = 0;       // Lowest raw battery reading since the last balance trace row
float VoltageADCf = 0;            // Battery voltage ADC data
float Voltage = 0;                // Battery voltage

uint32_t controlGateSequence = 0;
uint32_t activeTraceSequence = 0;
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

PIDController anglePidSettings(PID_ANGLE_P_WITH_TOUCH, PID_ANGLE_I_WITH_TOUCH,
                               PID_ANGLE_D_WITH_TOUCH, 0, PID_ANGLE_LIMIT_WITH_TOUCH);
PIDController speedPidSettings(PID_SPEED_P_WITH_TOUCH, PID_SPEED_I_WITH_TOUCH,
                               PID_SPEED_D_WITH_TOUCH, 0, PID_SPEED_LIMIT_WITH_TOUCH);
PIDController yawPidSettings(PID_YAW_P_DEFAULT, PID_YAW_I_DEFAULT, PID_YAW_D_DEFAULT,
                             0, PID_YAW_LIMIT_DEFAULT);
PIDController rollPidSettings(
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? PID_ROLL_P_WITH_TOUCH : 0.06,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? PID_ROLL_I_WITH_TOUCH : 1.5,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? PID_ROLL_D_WITH_TOUCH : 0.003, 0,
    DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? PID_ROLL_LIMIT_WITH_TOUCH : 2);
PIDController touchXPidSettings(TOUCH_X_PID_P_DEFAULT, TOUCH_X_PID_I_DEFAULT,
                               TOUCH_X_PID_D_DEFAULT, 0, TOUCH_X_PID_LIMIT_DEFAULT);
PIDController touchYPidSettings(TOUCH_Y_PID_P_DEFAULT, TOUCH_Y_PID_I_DEFAULT,
                               TOUCH_Y_PID_D_DEFAULT, 0, TOUCH_Y_PID_LIMIT_DEFAULT);

float control_torque_compensation = 0;  // Control torque compensation
float wheelSpeedFeedbackGain = 0.0f;
float wheelVelocityFeedbackCorrection = 0;
float driveTiltReductionGain = 1.0f; //TODO what does it do?
float driveEffectiveSpeed = 0;
float driveSpeedBodyXRaw = 0;

float pidTimestepSec = 0.01;

MyPIDController Angle_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);  // p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController Speed_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);
MyPIDController Yaw_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);
MyPIDController Roll_Pid(0, 0, 0, 0, 0, pidTimestepSec, 0, 0);

MyPIDController TouchX_Pid(0, 0, 0, 0, 10, pidTimestepSec, 0, 0);
MyPIDController TouchY_Pid(0, 0, 0, 0, 8, pidTimestepSec, 0, 0);
bool balancePidNeedsPriming = true; // what does this mean? 

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

#if AdjusParameter == ADJUST_BALANCE_SPEED_YAW_ROLL
void CbAnglePid(char *cmd) { //TODO cb meaning?
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

#elif AdjusParameter == ADJUST_BALL_PUSHING

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

double m1AngleRad = 0;
double m2AngleRad = 0;

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
void print_data(void);
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

  Serial.begin(SENSOR_DIAGNOSTIC_MODE ? DIAGNOSTIC_SERIAL_BAUD_RATE :
               (DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? LIVE_TUNING_SERIAL_BAUD_RATE : SERIAL_BAUD_RATE)); //TODO simplify to one serial baud rate
  Logging::begin();
  const PersistedCalibrationValues loadedCalibration = calibrationStore.load();
  servoTrims = loadedCalibration.servos;
  const imu::CalibrationValues imuCalibration{loadedCalibration.gyro, loadedCalibration.attitude};
  Logging::message(Logging::Level::Info, "Calibration",
                   "Roll Zero Bias: %.2f, Pitch Zero Bias: %.2f",
                   loadedCalibration.attitude.rollDegrees, loadedCalibration.attitude.pitchDegrees);
  Logging::message(Logging::Level::Info, "Calibration",
                   "gyroBiasX: %.2f, gyroBiasY: %.2f, gyroBiasZ: %.2f",
                   loadedCalibration.gyro.xRawCounts, loadedCalibration.gyro.yRawCounts, loadedCalibration.gyro.zRawCounts);
  Logging::message(Logging::Level::Info, "Calibration",
                   "servo1: %.2f, servo2: %.2f, servo3: %.2f, servo4: %.2f",
                   servoTrims.servo1Degrees, servoTrims.servo2Degrees, servoTrims.servo3Degrees, servoTrims.servo4Degrees);

  // TODO is this the right location?
  pinMode(board::kStatusLedPin, OUTPUT);
  digitalWrite(board::kStatusLedPin, LOW);
  Serial.println("system run.");
  delay(500);

#if SENSOR_DIAGNOSTIC_MODE
  biquadFilterInitLPF(&VoltageFilterLPF, 20, DIAGNOSTIC_LOG_SAMPLE_RATE_HZ);
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

  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);
  _delay(555);

  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);  // Assembly position TODO why is this repeated?
  // IMU
  if (!imu::begin(imu::Settings{}, imuCalibration)) {
    Serial.println("IMU initialization failed!");
    while (1)
      ;
  }
  Serial.println("IMU initialized successfully!");

  // Remote control
  sBus.begin(); //TODO rename sBus to rc or similar

  //TODO is this the most elegant way to initialize the filters?
  for (int i = 0; i < 6; i++)
    biquadFilterInitLPF(&BodyCommandFilterLPF[i], (unsigned int)bodyCommandFilterCutoffHz, 1000);

  biquadFilterInitLPF(&BodyCommandFilterLPF[8], 50, 1000);
  biquadFilterInitLPF(&BodyCommandFilterLPF[9], 50, 1000);
  biquadFilterInitLPF(&BodyCommandFilterLPF[10], 200, 1000);
  biquadFilterInitLPF(&BodyCommandFilterLPF[11], 200, 1000);

  // use monitoring with serial
  TouchscreenInit(1000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial); // TODO a more centralized way to enable debug sections would be nice

  I2Cone.begin(board::kI2cOneSdaPin, board::kI2cOneSclPin, 400000);
  I2Ctwo.begin(board::kI2cTwoSdaPin, board::kI2cTwoSclPin, 400000);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);

  // link the motor to the sensor
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 8.4;
  driver.init();

  driver2.voltage_power_supply = 8.4;
  driver2.init();
  // link driver
  motor1.linkDriver(&driver);
  motor2.linkDriver(&driver2);
  motor1.torque_controller = TorqueControlType::estimated_current;
  motor1.controller = MotionControlType::velocity;

    // TODO where do all these numbers come from? why hardcoded? some values also could be moved to a class or struct if still relevant
  motor1.motion_downsample = 0.0;  //

  // velocity loop PID
  motor1.PID_velocity.P = 0.006;  // 0.07;
  motor1.PID_velocity.I = 0;
  motor1.PID_velocity.D = 0.0;
  motor1.PID_velocity.output_ramp = 10000;
  motor1.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.001;
  // Limits
  motor1.velocity_limit = 88.0;
  motor1.voltage_limit = 8.4;
  motor1.current_limit = 5.0;
  // general settings
  // motor phase resistance
  motor1.phase_resistance = 22;
  // pwm modulation settings
  motor1.foc_modulation = FOCModulationType::SpaceVectorPWM;
  // Set PWM modulation to center alignment mode
  motor1.modulation_centered = 1.0;

  motor2.torque_controller = TorqueControlType::estimated_current;
  motor2.controller = MotionControlType::velocity;

  // TODO where do all these numbers come from? why hardcoded? some values also could be moved to a class or struct if still relevant
  motor2.motion_downsample = 0.0;

  // velocity loop PID
  motor2.PID_velocity.P = 0.006;
  motor2.PID_velocity.I = 0;
  motor2.PID_velocity.D = 0;
  motor2.PID_velocity.output_ramp = 10000;
  motor2.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.001;
  // Limits
  motor2.velocity_limit = motor1.velocity_limit;
  motor2.voltage_limit = motor1.voltage_limit;
  motor2.current_limit = motor1.current_limit;
  // general settings
  // motor phase resistance
  motor2.phase_resistance = motor1.phase_resistance;
  // pwm modulation settings
  motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor2.modulation_centered = motor1.modulation_centered;

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC

  motor1.initFOC();
  motor2.initFOC();

  // set the inital target value
  motor1.target = 0;
  motor2.target = 0;

  // comment out if not needed

  motor1.useMonitoring(Serial); //TODO logger lib usable here?
  motor1.monitor_downsample = 10;  // disable intially

  // subscribe motor to the commander
  command.add('T', handleM1MotionCommand, "motion1 control");  // Set motor target value
  command.add('M', handleM1Command, "motor1");

  command.add('A', zeroBias_servo1, "my zeroBias_servo1");  // Set servo 1 bias
  command.add('B', zeroBias_servo2, "my zeroBias_servo2");  //
  command.add('C', zeroBias_servo3, "my zeroBias_servo3");  //
  command.add('D', zeroBias_servo4, "my zeroBias_servo4");  //

  command.add('Z', setImuLowPassCutoff, "my ImuLPF_CUTOFF_FREQ");

  command.add('Q', TwoKp, "my TwoKp");  // MahonyFilter
  command.add('I', TwoKi, "my TwoKi");  // MahonyFilter

  command.add('K', KeyScalar, "my Select");
  command.add('E', KeyCalibration, "my CalibrationSelect");

  command.add('F', SetBodyCommandFilterCutoffHz, "my CutoffFreq");
  command.add('J', SetBodyCommandFilteringEnabled, "my EnableDFilter");

#if AdjusParameter == ADJUST_BALANCE_SPEED_YAW_ROLL
  command.add('P', CbAnglePid, "anglePidSettings");
  command.add('S', CbSpeedPid, "speedPidSettings");
  command.add('Y', CbYawPid, "yawPidSettings");
  command.add('R', CbRollPid, "rollPidSettings");
  command.add('O', Target_Body_Height, "my Target_Body_Height");
#elif AdjusParameter == ADJUST_BALL_PUSHING
  command.add('L', CbTouchXPid, "touchXPidSettings");
  command.add('N', CbTouchYPid, "touchYPidSettings");
  command.add('G', ControlTorqueCompensation, "my ControlTorqueCompensation");
#endif

  command.add('U', User_command, "my User_command");
  command.add('V', CbWheelSpeedFeedbackGain, "wheel speed feedback gain");
  command.add('W', CbDriveTiltReductionGain, "drive tilt reduction gain");

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
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

/**
 * @brief Choose BLE or remote control input
 *
 * Use the remote control as the input when BLE connection is lost.
 */

void CtrlInput(){
  if(rp.ble_connected){
    bleCtrl();
  }else{
    RXsbus();
  }
}
void bleCtrl(){

    MovementSpeed         = mapf(ble_ctrler.ch[2], BLE_CH2_MIN, BLE_CH2_MAX, -15, 15);
    BodyTurn              = -mapf(ble_ctrler.ch[3], BLE_CH3_MIN, BLE_CH3_MAX, -11, 11);
    pid_gains_mode        = map(ble_ctrler.ch[4], BLE_CH4_MIN, BLE_CH4_MAX, 0, 2);
    posture_or_mark_mode  = map(ble_ctrler.ch[5], BLE_CH5_MIN, BLE_CH5_MAX, 0, 1);
    roll_mode             = map(ble_ctrler.ch[6], BLE_CH6_MIN, BLE_CH6_MAX, 0, 1);
    attitude_mode         = map(ble_ctrler.ch[7], BLE_CH7_MIN, BLE_CH7_MAX, 0, 2);
    // Top ball
    top_ball_x = mapf(ble_ctrler.ch[8], BLE_CH8_MIN, BLE_CH8_MAX, -5, 5);  //  
    top_ball_y = mapf(ble_ctrler.ch[9], BLE_CH9_MIN, BLE_CH9_MAX, -5, 5);

    if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT)  // Attitude control 1
    {
      BodyHeightCommandM = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX,
                                BODY_HEIGHT_DEFAULT_M, BODY_HEIGHT_MAX_M);  // Body height
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST)  // Attitude control 2
    {
      BodyPitching = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX, -12, 12);  // Pitching       + sbus_vrb
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)              // Attitude control 3
    {
      BodyHeightCommandM = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX,
                                BODY_HEIGHT_DEFAULT_M, BALL_POISE_MAX_BODY_HEIGHT_M);
    }
    BodyRoll = mapf(ble_ctrler.ch[0], BLE_CH0_MIN, BLE_CH0_MAX, -0.011, 0.011);
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

    MovementSpeed = mapf(sBus.channels[2], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -15, 15);
    if (DIAGNOSTIC_LIVE_TUNING_DEFAULTS)
      MovementSpeed *= 2.0f;  // Keep drive-command authority near the previous 0.03 x 3.3 setting.
    BodyTurn = -mapf(sBus.channels[3], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -11, 11);
    pid_gains_mode = map(sBus.channels[4], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 2);
    posture_or_mark_mode = map(sBus.channels[5], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 1);
    roll_mode = map(sBus.channels[6], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 1);
    attitude_mode = map(sBus.channels[7], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, 0, 2);
    // Top ball
    top_ball_x = mapf(sBus.channels[8], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -5, 5);  // Modify top ball target position
    top_ball_y = mapf(sBus.channels[9], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -5, 5);

    if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT)  // Attitude control 1
    {
      if (sBus.channels[1] <= 992)
        BodyHeightCommandM = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, 992,
                                  BODY_HEIGHT_MIN_M, BODY_HEIGHT_DEFAULT_M);  // Body height
      else
        BodyHeightCommandM = mapf(sBus.channels[1], 993, SBUS_CHANNEL_MAX,
                                  BODY_HEIGHT_DEFAULT_M, BODY_HEIGHT_MAX_M);  // Body height
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST)  // Attitude control 2
    {
      BodyPitching = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -12, 12);  // Pitching       + sbus_vrb
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)              // Attitude control 3
    {
      if (sBus.channels[1] <= 992)
        BodyHeightCommandM = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, 992,
                                  BODY_HEIGHT_MIN_M, BODY_HEIGHT_DEFAULT_M);
      else
        BodyHeightCommandM = mapf(sBus.channels[1], 993, SBUS_CHANNEL_MAX,
                                  BODY_HEIGHT_DEFAULT_M, BALL_POISE_MAX_BODY_HEIGHT_M);

    }

    BodyRoll = mapf(sBus.channels[0], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -0.011, 0.011);
  }

  const bool lost = static_cast<uint32_t>(millis() - lastFrameMs) > kRcFrameTimeoutMs ||
                    sBus.Failsafe() != FUTABA_SBUS::kSignalOk;
  if (lost) {
    MovementSpeed = 0;
    BodyTurn = 0;
  }
}

/**
 * @brief Converts an angle from radians to degrees.
 * @param arc The angle in radians.
 * @return The angle in degrees.
 */
/**
 * @brief Prints debugging data to the serial port based on a selection variable.
 *
 * This function uses a large switch statement controlled by the global `debugLogSelector`
 * variable (which can be set via the serial commander). Each case prints a
 * different set of variables, such as IMU data, PID controller states, motor
 * velocities, S.BUS channels, etc. This is a flexible way to debug different
 * parts of the system without recompiling.
 */
void print_data(void) {
  const int selection = (int)debugLogSelector;
  const uint32_t nowMs = millis();
  const imu::Snapshot& sampleImu = imu::latest();
  const imu::Settings imuSettings = imu::settings();
  const imu::CalibrationValues imuCalibration = imu::calibration();
  if (Logging::isChannelDebugSelector(selection)) {
    const Logging::DebugSelector selector = static_cast<Logging::DebugSelector>(selection);
    if (!RobotLogCapture::selectedDebugDue(selector, nowMs)) return;
    Logging::DebugSample sample{};
    sample.selector = selector;
    sample.payload.channelState = {nowMs * 0.001f, sBus.channels[selection - Logging::kChannelDebugFirst]};
    Logging::submit(sample);
    return;
  }

  const Logging::DebugSelector selector = static_cast<Logging::DebugSelector>(selection);
  if (Logging::isSelectedDebugSelector(selection) &&
      RobotLogCapture::selectedDebugDue(selector, nowMs)) {
  Logging::DebugSample sample{};
  sample.selector = selector;
  switch (selection) {
    case 1:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.attitudeDegrees.roll, sampleImu.attitudeDegrees.pitch, sampleImu.attitudeDegrees.yaw};
      break;
    case 2:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.accelerationG.x, sampleImu.accelerationG.y, sampleImu.accelerationG.z};
      break;
    case 3:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.angularVelocityRadPerSec.x, sampleImu.angularVelocityRadPerSec.y, sampleImu.angularVelocityRadPerSec.z};
      break;
    case 4:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.attitudeDegrees.roll - imuCalibration.attitude.rollDegrees,
                                     sampleImu.attitudeDegrees.pitch - imuCalibration.attitude.pitchDegrees,
                                     sampleImu.attitudeDegrees.yaw - 0.0f};
      break;
    case 5:
      sample.payload.timedVector3 = {controlTimestepSec, imuCalibration.attitude.rollDegrees, imuCalibration.attitude.pitchDegrees, 0.0f};
      break;
    case 6:
      sample.payload.motorVelocities = {m1VelocityRadPerSec, m2VelocityRadPerSec};
      break;
    case 7:
      sample.payload.pair = {m1VelocityRadPerSec, m1FilteredVelocityRadPerSec};
      break;
    case 8:
      for (int i = 0; i < 10; ++i) sample.payload.receiverChannels.channels[i] = sBus.channels[i];
      sample.payload.receiverChannels.frameDtMs = sbusFrameIntervalMs;
      break;
    case 9:
      sample.payload.controllerGains = {{Angle_Pid.Kp, Angle_Pid.Ki, Angle_Pid.Kd},
                                        {Speed_Pid.Kp, Speed_Pid.Ki, Speed_Pid.Kd},
                                        {Yaw_Pid.Kp, Yaw_Pid.Ki, Yaw_Pid.Kd}, controlTimestepSec};
      break;
    case 10:
      sample.payload.mahony = {imuSettings.mahonyTwoKp, imuSettings.mahonyTwoKi, sampleImu.attitudeDegrees.roll,
                               sampleImu.attitudeDegrees.pitch, sampleImu.dtSec};
      break;
    case 11:
      sample.payload.imuAxes = {sampleImu.complementaryAnglesDegrees.x, sampleImu.complementaryAnglesDegrees.y, sampleImu.complementaryAnglesDegrees.z, sampleImu.integratedGyroAnglesDegrees.x, sampleImu.integratedGyroAnglesDegrees.y, sampleImu.integratedGyroAnglesDegrees.z, sampleImu.dtSec};
      break;
    case 12:
      sample.payload.pair = {sampleImu.complementaryAnglesDegrees.x, sampleImu.attitudeDegrees.roll};
      break;
    case 13:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.filteredAngularVelocityRadPerSec.x, sampleImu.filteredAngularVelocityRadPerSec.y, sampleImu.filteredAngularVelocityRadPerSec.z};
      break;
    case 14:
      sample.payload.timedVector3 = {controlTimestepSec, sampleImu.filteredAccelerationG.x, sampleImu.filteredAccelerationG.y, sampleImu.filteredAccelerationG.z};
      break;
    case 15:
      sample.payload.pair = {sampleImu.accelerationG.y, sampleImu.filteredAccelerationG.y};
      break;
    case 16:
      sample.payload.pair = {sampleImu.angularVelocityRadPerSec.y, sampleImu.filteredAngularVelocityRadPerSec.y};
      break;
    case 17:
      sample.payload.currentSetpoint = {motor2.current_sp};
      break;
    case 18:
      sample.payload.sensorAngles = {motor2.target, sensor1.getAngle(), sensor1.getMechanicalAngle(),
                                     sensor2.getAngle(), sensor2.getMechanicalAngle()};
      break;
    case 19:
      sample.payload.servoOffsets = {{servoTrims.servo1Degrees, servoTrims.servo2Degrees, servoTrims.servo3Degrees, servoTrims.servo4Degrees}};
      break;
    case 20:
      sample.payload.ballBalanceGeometry = {top_ball_x, BodyRoll, BodyHeightCommandM};
      break;
    case 21:
      sample.payload.balanceState = {sampleImu.correctedRollDegrees, BodyPitching};
      break;
    case 22:
      sample.payload.pidIntegralState = {Angle_Pid.iLimit, Angle_Pid.integral,
                                         Angle_Pid.outI, Angle_Pid.output};
      break;
    case 23:
      sample.payload.pidIntegralOutputState = {Speed_Pid.iLimit, Speed_Pid.integral,
                                               Speed_Pid.outI, BodyPitchingFiltered, Speed_Pid.output};
      break;
    case 24:
      sample.payload.pair = {bodyCommandFilteringEnabled, bodyCommandFilterCutoffHz};
      break;
    case 25:
      sample.payload.bodyPitchState = {BodyPitchingFiltered, BodyPitching};
      break;
    case 26:
      if (Touch.state == 1) {
        sample.payload.touchPoint = {Touch.state, Touch.XPdat, Touch.YPdat};
      } else if (Touch.state == 0) {
        sample.payload.touchPoint = {Touch.state, Touch.XLdat, Touch.YLdat};
      } else return;
      break;
    case 27:
      sample.payload.touchFilteredPoint = {Touch.XPdat, Touch.YPdat, Touch.XPdatF, Touch.YPdatF};
      break;
    case 28:
      sample.payload.filteredGeometry = {BodyPitchingFiltered, BodyRollFiltered, BodyHeightCommandFilteredM, SlideStepFiltered,
                                         top_ball_x, top_ball_y};
      break;
    case 29:
      sample.payload.pidTuningState = {TouchY_Pid.Kp, TouchY_Pid.Ki, TouchY_Pid.Kd,
                                       TouchY_Pid.deriv, TouchY_Pid.output};
      break;
    case 30:
      sample.payload.pidDerivative = {TouchY_Pid.deriv};
      break;
    case 31:
      sample.payload.pidState = {Roll_Pid.error, Roll_Pid.iLimit, Roll_Pid.integral,
                                 Roll_Pid.outI, Roll_Pid.output};
      break;
    case 32:
      sample.payload.pidCoefficients = {Roll_Pid.Kp, Roll_Pid.Ki, Roll_Pid.Kd};
      break;
    case 33:
      sample.payload.pidIntegralOutputState = {Yaw_Pid.iLimit, Yaw_Pid.integral,
                                               Yaw_Pid.outI, BodyPitchingFiltered, Yaw_Pid.output};
      break;
    case 34:
      sample.payload.pidTuningState = {TouchX_Pid.Kp, TouchX_Pid.Ki, TouchX_Pid.Kd,
                                       TouchX_Pid.deriv, TouchX_Pid.output};
      break;
    case 35:
      sample.payload.pidDerivative = {TouchX_Pid.deriv};
      break;
    case 36:
      sample.payload.touchState = {Touch.state, Touch.start};
      break;
    case 37:
      sample.payload.ballPosition = {top_ball_x, sbus_top_ball_x_smoothed,
                                     top_ball_y, sbus_top_ball_y_smoothed};
      break;
    case 38:
      sample.payload.balanceOutputPair = {BodyPitching, TouchY_Pid.output};
      break;
    case 39:
      sample.payload.pidIntegralState = {TouchY_Pid.iLimit, TouchY_Pid.integral,
                                         TouchY_Pid.outI, TouchY_Pid.output};
      break;
    case 40:
      if (Touch.state != 0 && Touch.state != 1) return;
      sample.payload.touchPoint = {Touch.state, Touch.XPressDat, Touch.YPressDat};
      break;
    case 41:
      sample.payload.rollOutput = {sampleImu.correctedRollDegrees, BodyPitching, Speed_Pid.output};
      break;
    case 42:
      sample.payload.rollCorrection = {sampleImu.correctedRollDegrees, BodyPitching,
                                       BodyPitchingCorrect(BodyPitchingFiltered)};
      break;
    case 43:
      sample.payload.voltageState = {VoltageADC, VoltageADCf, Voltage};
      break;
    case 44:
      sample.payload.tuningState = {PidParameterTuning, TargetBodyHeightOverrideM};
      break;
    case 45:
      sample.payload.tumbleState = {RobotTumble, sampleImu.correctedRollDegrees, Angle_Pid.error};
      break;
    case 46:
      sample.payload.gyroFft = {
          static_cast<uint32_t>(sampleImu.readTimeUs),
          sampleImu.angularVelocityRadPerSec.x, sampleImu.angularVelocityRadPerSec.y,
          sampleImu.angularVelocityRadPerSec.z, sampleImu.filteredAngularVelocityRadPerSec.x,
          sampleImu.filteredAngularVelocityRadPerSec.y, sampleImu.filteredAngularVelocityRadPerSec.z};
      break;
    default: return;
  }
  Logging::submit(sample);
  }
  switch ((int)debugLogSelector) {
    case 55: {
      if (controlGateSequence % 7 == 0) {
        const uint32_t traceMs = millis();
        Logging::TraceSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.postureOrMarkMode = posture_or_mark_mode;
        sample.minimumBatteryRawV = board::kBatteryVoltsPerAdcCount * VoltageADCMin;
        sample.batteryV = Voltage;
        sample.rollOk = sampleImu.correctedRollDegrees;
        sample.pitchOk = sampleImu.correctedPitchDegrees;
        int32_t ranges[4];
        RobotLogCapture::copyServoRangesAndBeginNextWindow(ranges);
        for (int i = 0; i < 4; ++i) {
          sample.servoAngle[i] = servoTraceAngle[i];
          sample.servoRange[i] = ranges[i];
        }
        sample.leftMotorTarget = motor1.target;
        sample.rightMotorTarget = motor2.target;
        Logging::submit(sample);
        VoltageADCMin = VoltageADC;
      }
      break;
    }

    case 56: {
      if (controlGateSequence % 7 == 0) {
        const uint32_t traceMs = millis();
        const float rawMinV = board::kBatteryVoltsPerAdcCount * VoltageADCMin;
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_NOT_TUMBLING;
        int32_t ranges[4];
        RobotLogCapture::copyServoRangesAndBeginNextWindow(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::ControlSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = rawMinV;
        sample.batteryV = Voltage;
        sample.rollOk = sampleImu.correctedRollDegrees;
        sample.yawRateRadPerSec = sampleImu.angularVelocityRadPerSec.z;
        sample.bodyTurn = BodyTurn;
        sample.angleError = active ? Angle_Pid.error : 0.0f;
        sample.angleOutput = active ? Angle_Pid.output : 0.0f;
        sample.yawError = active ? Yaw_Pid.error : 0.0f;
        sample.yawOutput = active ? Yaw_Pid.output : 0.0f;
        sample.leftMotorTarget = motor1.target;
        sample.rightMotorTarget = motor2.target;
        sample.maxServoRange = maxServoRange;
        Logging::submit(sample);
        VoltageADCMin = VoltageADC;
      }
      break;
    }

    case 57: {
      if (controlGateSequence % 7 == 0) {
        const uint32_t traceMs = millis();
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_NOT_TUMBLING;
        int32_t ranges[4];
        RobotLogCapture::copyServoRangesAndBeginNextWindow(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::BalanceSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = board::kBatteryVoltsPerAdcCount * VoltageADCMin;
        sample.rollOk = sampleImu.correctedRollDegrees;
        sample.angleError = active ? Angle_Pid.error : 0.0f;
        sample.angleProportional = active ? Angle_Pid.outP : 0.0f;
        sample.angleIntegral = active ? Angle_Pid.outI : 0.0f;
        sample.angleDerivative = active ? Angle_Pid.outD : 0.0f;
        sample.bodyX = active ? BodyX : 0.0f;
        sample.leftMotorTarget = motor1.target;
        sample.rightMotorTarget = motor2.target;
        sample.maxServoRange = maxServoRange;
        Logging::submit(sample);
        VoltageADCMin = VoltageADC;
      }
      break;
    }

    case 58: {
      if (controlGateSequence % 7 == 0) {
        const uint32_t traceMs = millis();
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_NOT_TUMBLING;
        int32_t ranges[4];
        RobotLogCapture::copyServoRangesAndBeginNextWindow(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::DriveSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = board::kBatteryVoltsPerAdcCount * VoltageADCMin;
        sample.batteryV = Voltage;
        sample.requestedSpeed = MovementSpeed;
        sample.effectiveSpeed = active ? driveEffectiveSpeed : 0.0f;
        sample.leftWheelVelocity = m1FilteredVelocityRadPerSec;
        sample.rightWheelVelocity = m2FilteredVelocityRadPerSec;
        sample.controlDtSec = controlTimestepSec;
        sample.speedError = active ? Speed_Pid.error : 0.0f;
        sample.speedProportional = active ? Speed_Pid.outP : 0.0f;
        sample.speedIntegral = active ? Speed_Pid.outI : 0.0f;
        sample.speedDerivative = active ? Speed_Pid.outD : 0.0f;
        sample.speedOutput = active ? Speed_Pid.output : 0.0f;
        sample.driveBodyXRaw = active ? driveSpeedBodyXRaw : 0.0f;
        sample.bodyX = active ? BodyX : 0.0f;
        sample.bodyPitchFiltered = BodyPitchingFiltered;
        sample.rollOk = sampleImu.correctedRollDegrees;
        sample.angleOutput = active ? Angle_Pid.output : 0.0f;
        sample.angleProportional = active ? Angle_Pid.outP : 0.0f;
        sample.angleIntegral = active ? Angle_Pid.outI : 0.0f;
        sample.angleDerivative = active ? Angle_Pid.outD : 0.0f;
        sample.wheelSpeedFeedbackOutput = active ? wheelVelocityFeedbackCorrection : 0.0f;
        sample.leftMotorTarget = motor1.target;
        sample.rightMotorTarget = motor2.target;
        sample.ballX = top_ball_x;
        sample.touchXFiltered = Touch.XPdatF;
        sample.bodyPitch = BodyPitching;
        sample.maxServoRange = maxServoRange;
        Logging::submit(sample);
        VoltageADCMin = VoltageADC;
      }
      break;
    }

    default:

      break;
  }
}

/**
 * @brief Applies a polynomial correction to the body pitching value.
 *
 * This function implements a quadratic correction formula. This is likely an
 * empirical correction factor to compensate for nonlinearities in the system
 * dynamics or sensor readings, improving balance performance.
 *
 * @param x The raw body pitching value.
 * @return The corrected body pitching value.
 */
float BodyPitchingCorrect(float x)  // Pitch angle correction
{
  float y = 0.000004 * x * x + 0.0004 * x - 0.0008;  // y = 4E-06x2 + 0.0004x - 0.0008    y = -2E-07x2 + 0.0002x - 0.0029
  return y;
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
  if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH)  // No touch screen
  {
    // Roll
    rollPidSettings.P = PID_ROLL_P_NO_TOUCH;
    rollPidSettings.I = PID_ROLL_I_NO_TOUCH;
    rollPidSettings.D = PID_ROLL_D_NO_TOUCH;
    rollPidSettings.limit = PID_ROLL_LIMIT_NO_TOUCH;  // Integral limit

    // Speed loop
    speedPidSettings.P = PID_SPEED_P_NO_TOUCH;
    speedPidSettings.I = PID_SPEED_I_NO_TOUCH;
    speedPidSettings.D = PID_SPEED_D_NO_TOUCH;
    speedPidSettings.limit = PID_SPEED_LIMIT_NO_TOUCH;  // Integral limit

    // Balance loop
    anglePidSettings.P = PID_ANGLE_P_NO_TOUCH;
    anglePidSettings.I = PID_ANGLE_I_NO_TOUCH;
    anglePidSettings.D = PID_ANGLE_D_NO_TOUCH;
    anglePidSettings.limit = PID_ANGLE_LIMIT_NO_TOUCH;  // Integral limit
  } else if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH)  // With touch screen
  {
    // Roll
    rollPidSettings.P = PID_ROLL_P_WITH_TOUCH;
    rollPidSettings.I = PID_ROLL_I_WITH_TOUCH;
    rollPidSettings.D = PID_ROLL_D_WITH_TOUCH;
    rollPidSettings.limit = PID_ROLL_LIMIT_WITH_TOUCH;  // Integral limit

    // Speed loop
    speedPidSettings.P = PID_SPEED_P_WITH_TOUCH;
    speedPidSettings.I = PID_SPEED_I_WITH_TOUCH;
    speedPidSettings.D = PID_SPEED_D_WITH_TOUCH;
    speedPidSettings.limit = PID_SPEED_LIMIT_WITH_TOUCH;  // Integral limit

    // Balance loop
    anglePidSettings.P = PID_ANGLE_P_WITH_TOUCH;
    anglePidSettings.I = PID_ANGLE_I_WITH_TOUCH;
    anglePidSettings.D = PID_ANGLE_D_WITH_TOUCH;
    anglePidSettings.limit = PID_ANGLE_LIMIT_WITH_TOUCH;  // Integral limit
  }

  yawPidSettings.P = PID_YAW_P_DEFAULT;
  yawPidSettings.I = PID_YAW_I_DEFAULT;
  yawPidSettings.D = PID_YAW_D_DEFAULT;
  yawPidSettings.limit = PID_YAW_LIMIT_DEFAULT;

  // Touch screen
  touchXPidSettings.P = TOUCH_X_PID_P_DEFAULT;
  touchXPidSettings.I = TOUCH_X_PID_I_DEFAULT;
  touchXPidSettings.D = TOUCH_X_PID_D_DEFAULT;
  touchXPidSettings.limit = TOUCH_X_PID_LIMIT_DEFAULT;  // Integral limit

  touchYPidSettings.P = TOUCH_Y_PID_P_DEFAULT;
  touchYPidSettings.I = TOUCH_Y_PID_I_DEFAULT;
  touchYPidSettings.D = TOUCH_Y_PID_D_DEFAULT;
  touchYPidSettings.limit = TOUCH_Y_PID_LIMIT_DEFAULT;  // Integral limit
}

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

  TouchX_Pid.deriv = constrain(TouchX_Pid.deriv, -11000, 11000);
  TouchX_Pid.outD = TouchX_kd * TouchX_Pid.deriv;

  if ((attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE) || (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO))  // Top ball mode
  {
    BodyPitching = TouchX_Pid.outP + TouchX_Pid.outI + TouchX_Pid.outD + sbus_top_ball_x_smoothed;
    BodyPitching = -BodyPitching;
  }

  TouchY_Pid.deriv = constrain(TouchY_Pid.deriv, -11000, 11000);
  TouchY_Pid.outD = TouchY_kd * TouchY_Pid.deriv;
  TouchY_Pid.output = TouchY_Pid.outP + TouchY_Pid.outI + TouchY_Pid.outD + sbus_top_ball_y_smoothed;
  if ((int)bodyCommandFilteringEnabled == 1) {
    TouchYPidOutputFiltered = biquadFilterApply(&BodyCommandFilterLPF[10], TouchY_Pid.output);
    TouchXPidOutputFiltered = biquadFilterApply(&BodyCommandFilterLPF[11], TouchX_Pid.output);
  } else {
    TouchYPidOutputFiltered = TouchY_Pid.output;
    TouchXPidOutputFiltered = TouchX_Pid.output;
  }

  if ((Touch.state == 1) && (Touch.P_count < 4)) {
    Touch.P_count++;
    TouchX_Pid.integral = 0;
    TouchY_Pid.integral = 0;
  }

  if (Touch.P_count >= 4) {
    if (TouchX_kd < TouchX_Pid.Kd) {
      TouchX_kd = TouchX_kd + (TouchX_Pid.Kd / 22);
    }
    if (TouchY_kd < TouchY_Pid.Kd) {
      TouchY_kd = TouchY_kd + (TouchY_Pid.Kd / 22);
    }
    Touch.start = 2;
  }

  if ((Touch.state == 0) && (Touch.L_count < 44))
    Touch.L_count++;
  else
    Touch.L_count = 0;

  if (Touch.L_count >= 44) {
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
  if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)  // Top ball禁止手动横滚
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
    const float tiltFraction = constrain((fabsf(imu::latest().correctedRollDegrees) - DRIVE_TILT_REDUCTION_START_DEG) /
                                         (DRIVE_TILT_REDUCTION_FULL_DEG - DRIVE_TILT_REDUCTION_START_DEG),
                                         0.0f, 1.0f);
    const float reduction = tiltFraction * constrain(driveTiltReductionGain, 0.0f, 1.0f);
    driveEffectiveSpeed = avgVelocity + (MovementSpeed - avgVelocity) * (1.0f - reduction);
  }
  const float speedError = avgVelocity - driveEffectiveSpeed;

  const float previousSpeedIntegral = Speed_Pid.integral;
  driveSpeedBodyXRaw = Speed_Pid.compute(speedError, dt);
  if (Speed_Pid.Ki != 0.0f && fabsf(driveSpeedBodyXRaw) > DRIVE_BODY_X_LIMIT_M &&
      driveSpeedBodyXRaw * speedError > 0.0f) {
    Speed_Pid.integral = previousSpeedIntegral;
    Speed_Pid.outI = Speed_Pid.Ki * previousSpeedIntegral;
    driveSpeedBodyXRaw = Speed_Pid.outP + Speed_Pid.outI + Speed_Pid.outD;
    Speed_Pid.output = driveSpeedBodyXRaw;
  }
  const float speedBodyX = constrain(driveSpeedBodyXRaw, -DRIVE_BODY_X_LIMIT_M, DRIVE_BODY_X_LIMIT_M);

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
  wheelVelocityFeedbackCorrection = constrain(wheelCorrection, -DRIVE_WHEEL_FEEDBACK_LIMIT,
                                      DRIVE_WHEEL_FEEDBACK_LIMIT);
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
        &BodyCommandFilterLPF[BODY_HEIGHT_COMMAND_FILTER_INDEX], BodyHeightCommandM);
    SlideStepFiltered = biquadFilterApply(&BodyCommandFilterLPF[3], SlideStep);
  } else {
    BodyPitchingFiltered = BodyPitching;
    BodyRollFiltered = BodyRoll;
    BodyHeightCommandFilteredM = BodyHeightCommandM;
    SlideStepFiltered = SlideStep;
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
  if (VoltageADCMin == 0 || VoltageADC < VoltageADCMin)
    VoltageADCMin = VoltageADC;
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
  if (abs(imu::latest().correctedRollDegrees) >= 35) {
    x++;
    if (x >= 20) {
      x = 20;
      RobotTumble = ROBOT_TUMBLING;  // Machine fall
    }
  } else {
    if ((RobotTumble == ROBOT_TUMBLING) && (abs(imu::latest().correctedRollDegrees) <= 5))  // Machine fall after fall
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
/**
 * @brief The main execution loop of the program.
 *
 * This function runs repeatedly after `setup()` is complete. It is the heart
 * of the robot's operation, responsible for:
 * 1. Calling the SimpleFOC `move()` and `loopFOC()` methods to update motor states.
 * 2. Handling serial communication for commands and telemetry.
 * 3. Reading sensors (IMU, RC, Touchscreen).
 * 4. Running posture control for two-wheel balancing.
 * 5. Calculating inverse kinematics to determine servo angles.
 * 6. Sending final commands to the servos.
 * 7. Handling safety checks like fall detection.
 * 8. Printing debug data.
 */
void loop() {
#if SENSOR_DIAGNOSTIC_MODE
  DiagnosticLoop();
  return;
#endif
  loopTimeUs = micros();

  // iterative function setting the outter loop target

  if (Communication_object == COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO) {
    if (Logging::profile() == Logging::Profile::Idle) motor1.monitor();
  } else if (Communication_object == COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS) {
    motor2.target = motor1.target;
  }

  motor1.move();
  motor2.move();

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();


  m1AngleRad = -sensor1.getPreciseAngle();
  m2AngleRad = sensor2.getPreciseAngle();

  // user communication
  command.run();

  const bool imuUpdated = imu::service();  // Poll once per loop without gating motor control
  CtrlInput();  // BLE or remote control input

  ReadTouchDat();

  controlTimestepSec = (loopTimeUs - previousControlTimeUs) / 1000000.0f;
  if (controlTimestepSec >= 0.001f) {   //1kHz
    controlGateSequence++;
    TouchBiquadFilter();  // Touch screen filter

    RemoteControlFiltering();  // Remote control signal filtering
    ReadVoltage();             // Battery
    print_data();              // Serial port data printing

    Robot_Tumble();  // Machine fall detection
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

    if (Voltage <= board::kLowBatteryVolts)
      statusLedPeriodTicks = 20;
    else
      statusLedPeriodTicks = 100;

    imu::CalibrationValues activeCalibration = imu::calibration();
    const imu::Snapshot& acceptedImu = imu::latest();
    CalibrationResult calibrationResult{CalibrationCompletion::None, false, {false, false, false, false}};
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

    const float wheelVelocityDt = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? controlTimestepSec : 0.01f;
    m1VelocityRadPerSec = (sensor1.getAngle() - m1PrevEncoderAngleRad) / wheelVelocityDt;
    m1FilteredVelocityRadPerSec = m1VelocityFilter(m1VelocityRadPerSec);
    m1PrevEncoderAngleRad = sensor1.getAngle();

    m2VelocityRadPerSec = -(sensor2.getAngle() - m2PrevEncoderAngleRad) / wheelVelocityDt;
    m2FilteredVelocityRadPerSec = m2VelocityFilter(m2VelocityRadPerSec);
    m2PrevEncoderAngleRad = sensor2.getAngle();

    float bodyH = BODY_HEIGHT_DEFAULT_M;
    float bodyRoll = BodyRollFiltered;

    if ((pid_gains_mode == REMOTE_CONTROL_PID_GAINS_MODE_OFF) || (RobotTumble == ROBOT_TUMBLING)) {
      balancePidNeedsPriming = true;
      if (Communication_object == COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE) {
        motor1.target = 0;
        motor2.target = 0;
      }

      bodyH = BODY_HEIGHT_DEFAULT_M;
      BodyX = 0;
      bodyRoll = 0;
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
        bodyRoll = Roll_Pid.output;

      if (TargetBodyHeightOverrideM == BODY_HEIGHT_OVERRIDE_DISABLED_M)
        bodyH = BodyHeightCommandFilteredM;
      else
        bodyH = TargetBodyHeightOverrideM;
    }

    if (RobotTumble == ROBOT_TUMBLING)  // Machine fall
    {
      bodyH = BODY_HEIGHT_DEFAULT_M;
      BodyX = 0;
      bodyRoll = 0;
      BodyPitchingFiltered = 0;
    }

    const LegSolveResult rightLeg = LegKinematics::solveRight(
        {BarycenterX - BodyX, bodyH - bodyRoll, BodyPitchingFiltered});
    if (rightLeg.status != LegSolveStatus::Success)
      Logging::message(Logging::Level::Warning, "RightInverseKinematics", "no");

    const LegSolveResult leftLeg = LegKinematics::solveLeft(
        {BarycenterX - BodyX, bodyH + bodyRoll, BodyPitchingFiltered});
    if (leftLeg.status != LegSolveStatus::Success)
      Logging::message(Logging::Level::Warning, "LeftInverseKinematics", "no");

    if (posture_or_mark_mode == REMOTE_CONTROL_PM_POSTURE_MODE)  // Posture
    {

      // Set the angle of the four servos
      servoTraceAngle[0] = leftLeg.angles.joint1Degrees - servoTrims.servo1Degrees;
      servoTraceAngle[1] = leftLeg.angles.joint2Degrees - servoTrims.servo2Degrees;
      servoTraceAngle[2] = rightLeg.angles.joint1Degrees - servoTrims.servo3Degrees;
      servoTraceAngle[3] = rightLeg.angles.joint2Degrees - servoTrims.servo4Degrees;
      servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1, servoTraceAngle[3], 1);
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
      servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1, servoTraceAngle[3], 1);
    }

    RobotLogCapture::observeServoAngles(servoTraceAngle);

    previousControlTimeUs = loopTimeUs;
  }
}
