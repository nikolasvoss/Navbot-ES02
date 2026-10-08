#include <Arduino.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <SimpleFOC.h>
#include <Preferences.h>  // This library is used for key-value data storage and retrieval in ESP32, enabling data persistence
#include "SlotCalibration.h"
#include "FUTABA_SBUS.h"
#include "ServoControl.h"
#include "ICM42688.h"
#include "MahonyFilter.h"
#include "OllieFOCdrive.h"
#include "filter.h"
#include "touchscreen.h"
#include "ble.h"
#include "robot.h"
#include "Logging.h"
#include "RobotLogCapture.h"


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

// ----- Editable Constants
#define SensorSwitch SENSOR_SWITCH_IIC_AS5600                                // 1: SPI  2: IIC AS5600
#define Communication_object COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE  // 0: two-wheel balance  1: simpleFOC Studio host computer  2: control dual motors  3: sample torque data
#define TorqueCompensation TORQUE_COMPENSATION_OFF                           // 1: torque compensation  0: no torque compensation (cannot be modified)
#define SwitchUser SWITCH_USER_MODE_SPEED_MODE                               // 0: view encoder position and direction  1: sample motor 1 torque compensation data  2: sample motor 2 torque compensation data  3: torque  4: speed  5: angle mode
#define CurrentUser CURRENT_LOOP_OFF                                         // 1: enable current loop
#define M2CurrentUser CURRENT_LOOP_OFF                                       // 1: enable current loop for motor 2

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

#define IMU_SAMPLING_RATE_HZ 1000.0f  // Sampling frequency
#define IMU_LPF_CUTOFF_FREQ_HZ 50.0f  // Cutoff frequency for low-pass filter
#define IMU_CALL_COUNT 100            // Number of times the function is called

#define BODY_THIGH_LENGTH_M 0.035f  // Thigh length (m)
#define BODY_SHANK_LENGTH_M 0.072f  // Shank length (m)

#define BOARD_PIN_LED 35        // LED IO
#define BOARD_PIN_ANALOG_IN 17  // Battery voltage IO

#define IMU_ACCEL_RANGE_G 8.0              // Unit: g
#define IMU_GYRO_RANGE_DEG_PER_SEC 2000.0  // Unit: °/s

#define CUSTOM_SERVO_1_PIN 11
#define CUSTOM_SERVO_2_PIN 12
#define CUSTOM_SERVO_3_PIN 21
#define CUSTOM_SERVO_4_PIN 14

#define CURRENT_SENSOR_MV_PER_AMP 90.0f  // ACS712-05B sensitivity is 185mV/A

#define SBUS_CHANNEL_MAX 1792
#define SBUS_CHANNEL_MIN 192

#define SERIAL_BAUD_RATE 576000
#define DIAGNOSTIC_SERIAL_BAUD_RATE 576000
#define LIVE_TUNING_SERIAL_BAUD_RATE 576000
#define DIAGNOSTIC_IMU_INTERVAL_US 10000
constexpr unsigned int DIAGNOSTIC_IMU_SAMPLE_RATE_HZ = 1000000U / DIAGNOSTIC_IMU_INTERVAL_US;
// Conservative drive tuning parameters; verify the wheel feedback sign on hardware.
constexpr float DRIVE_BODY_X_LIMIT_M = 0.010f;
constexpr float DRIVE_WHEEL_FEEDBACK_LIMIT = 8.0f;
constexpr float DRIVE_TILT_REDUCTION_START_DEG = 5.0f;
constexpr float DRIVE_TILT_REDUCTION_FULL_DEG = 10.0f;
// -------------------------------------

// Body
float TargetLegLength = 0;          // Target leg length
float LegLength = 0.06f;            // Leg length
float BarycenterX = 0;              // Center of mass X
float BodyPitching = 0;             // Pitch
float BodyRoll = 0;                 // Roll
float MovementSpeed = 0;            // Movement speed
float BodyTurn = 0;                 // Turning
float SlideStep = 0;                // Slide step
float BodyX = 0;                    // X position (controller output)
int RobotTumble = ROBOT_TUMBLE_NO;  // Robot tumble (fall detection)


// 滤波
float LegLength_f = 0.06f;     // Leg length
float BodyPitching_f = 0;      // Pitch
float BodyRoll_f = 0;          // Roll
float SlideStep_f = 0;         // Slide step
biquadFilter_t FilterLPF[12];  // Second-order low-pass filter
float TouchY_Pid_outputF = 0;
float TouchX_Pid_outputF = 0;

float cutoffFreq = 200;
float enableDFilter = 1;

void CutoffFreq(char *cmd) {
  command.scalar(&cutoffFreq, cmd);
}

void EnableDFilter(char *cmd) {
  command.scalar(&enableDFilter, cmd);
}

int LED_HL = 1;
int LED_count = 0;
int LED_dt = 100;
biquadFilter_t VoltageFilterLPF;  // Second-order low-pass filter
uint16_t VoltageADC = 0;          // Battery voltage ADC data
uint16_t VoltageADCMin = 0;       // Lowest raw battery reading since the last balance trace row
float VoltageADCf = 0;            // Battery voltage ADC data
float Voltage = 0;                // Battery voltage

// IMU
//  Create MahonyFilter object, set proportional gain and integral gain
MahonyFilter mahonyFilter(0.4f, 0.001f);
// Accelerometer range (here set to ±8g)
// Gyroscope range (here assumed to be ±2000°/s)
attitude_t attitude;
float roll_ok;   //
float pitch_ok;  //

zeroBias_t zeroBias;  // Zero offset
unsigned long timestamp_prev = 0;
uint32_t controlGateSequence = 0;
uint32_t activeTraceSequence = 0;
uint32_t diagnosticSequence = 0;
float IMUtime_dt = 0;

/* Low-pass filter parameters */
float RATE_HZ_last = IMU_SAMPLING_RATE_HZ;            // Sampling frequency
float LPF_CUTOFF_FREQ_last = IMU_LPF_CUTOFF_FREQ_HZ;  // Cutoff frequency

float RATE_HZ = IMU_SAMPLING_RATE_HZ;            // Sampling frequency
float LPF_CUTOFF_FREQ = IMU_LPF_CUTOFF_FREQ_HZ;  // Cutoff frequency
biquadFilter_t ImuFilterLPF[6];                  // Second-order low-pass filter

void ImuRATE_HZ(char *cmd) {
  command.scalar(&RATE_HZ, cmd);
}
void ImuLPF_CUTOFF_FREQ(char *cmd) {
  command.scalar(&LPF_CUTOFF_FREQ, cmd);
}

void Target_Leg_Length(char *cmd) {
  command.scalar(&TargetLegLength, cmd);
}

// Complementary filter
float angleGyroX, angleGyroY, angleGyroZ,
  angleAccX, angleAccY;
float angleX, angleY, angleZ;
float accCoef = 0.02f;
float gyroCoef = 0.98f;

//  Declare a Preferences object for subsequent read/write operations to flash memory
Preferences preferences;
//  Define a floating-point array to store Euler angle data
float zeroBiasFlash[9];
//  Define a character string pointer array to store the keys corresponding to the roll and pitch angles, which can be directly modified by the user
//  The key name is used to uniquely identify data in flash memory
const char *zeroBiasKeys[9] = {
  "roll",
  "pitch",
  "gyroX",
  "gyroY",
  "gyroZ",
  "servoAngle1",
  "servoAngle2",
  "servoAngle3",
  "servoAngle4"
};

int IMUCallCounter = 0;  //  Call counter

// Servo
void zeroBias_servo1(char *cmd) {
  command.scalar(&zeroBias.servo1, cmd);
}
void zeroBias_servo2(char *cmd) {
  command.scalar(&zeroBias.servo2, cmd);
}
void zeroBias_servo3(char *cmd) {
  command.scalar(&zeroBias.servo3, cmd);
}
void zeroBias_servo4(char *cmd) {
  command.scalar(&zeroBias.servo4, cmd);
}

bool pid_gains_mode_is_enabled(int mode) {
  return (mode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITHOUT_TOUCH || mode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH);
}

//  Create ServoControl object, pass in the custom pin
ServoControl servoControl(CUSTOM_SERVO_1_PIN, CUSTOM_SERVO_2_PIN, CUSTOM_SERVO_3_PIN, CUSTOM_SERVO_4_PIN);
int servoTraceAngle[4] = { 0, 0, 0, 0 };  // Last angle arguments sent to the four servos

// Remote control
FUTABA_SBUS sBus;
int sbus_dt_ms = 0;
int pid_gains_mode = REMOTE_CONTROL_PID_GAINS_MODE_OFF;
int posture_or_mark_mode = REMOTE_CONTROL_PM_POSTURE_MODE;
int roll_mode = REMOTE_CONTROL_ROLL_MODE_MANUAL;
int attitude_mode = REMOTE_CONTROL_ATTITUDE_MODE_DEFAULT;
float top_ball_x = 0;
float top_ball_y = 0;
float sbus_top_ball_x_smoothed = 0;
float sbus_top_ball_y_smoothed = 0;

//  Create PID controller instance
float Select = 0;             // Select the data to print
float CalibrationSelect = 0;  // Save calibration data 0: Calibration end  1: Calibrate gyroscope  2: Calibrate Euler angle  3: Calibrate servo

float PidParameterTuning = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? 1 : 0;  // 0: auto gains  1: live tuning

PIDController AnglePid(5, 200, 0.11, 0, 0.1);
PIDController SpeedPid(0.045, 0.005, 0, 0, 50);
PIDController YawPid(4, 0, 0, 0, 0);
PIDController RollPid(0.06, 1.5, 0.003, 0, 2);  //
PIDController TouchXPid(0.2, 0, 0.04, 0, 0);    //
PIDController TouchYPid(0.2, 0, 0.08, 0, 0);    //

float control_torque_compensation = 0;  // Control torque compensation
float wheelSpeedFeedbackGain = 0.0f;
float wheelSpeedFeedbackOutput = 0;
float driveTiltReductionGain = 1.0f;
float driveEffectiveSpeed = 0;
float driveSpeedBodyXRaw = 0;

float PidDt = 0.01;

//  Create MyPIDController instance, set initial parameters
MyPIDController Angle_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);  // p i d iLimit outputLimit dt EnableDFilter cutoffFreq
MyPIDController Speed_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);
MyPIDController Yaw_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);
MyPIDController Roll_Pid(0, 0, 0, 0, 0, PidDt, 0, 0);

MyPIDController TouchX_Pid(0, 0, 0, 0, 10, PidDt, 0, 0);
MyPIDController TouchY_Pid(0, 0, 0, 0, 8, PidDt, 0, 0);
bool balancePidNeedsPriming = true;

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
  command.scalar(&mahonyFilter.twoKp, cmd);
}
void TwoKi(char *cmd) {
  command.scalar(&mahonyFilter.twoKi, cmd);
}

void KeyScalar(char *cmd) {
  command.scalar(&Select, cmd);
  RobotLogCapture::setProfileForSelector(static_cast<int>(Select));
}
void KeyCalibration(char *cmd) {
  command.scalar(&CalibrationSelect, cmd);
}

#if AdjusParameter == ADJUST_BALANCE_SPEED_YAW_ROLL
void CbAnglePid(char *cmd) {
  command.pid(&AnglePid, cmd);
}
void CbSpeedPid(char *cmd) {
  command.pid(&SpeedPid, cmd);
}
void CbYawPid(char *cmd) {
  command.pid(&YawPid, cmd);
}

void CbRollPid(char *cmd) {
  command.pid(&RollPid, cmd);
}

#elif AdjusParameter == ADJUST_BALL_PUSHING

void CbTouchXPid(char *cmd) {
  command.pid(&TouchXPid, cmd);
}

void CbTouchYPid(char *cmd) {
  command.pid(&TouchYPid, cmd);
}
#endif

double Motor1_place_last = 0;
float Motor1_Velocity = 0;
float Motor1_Velocity_f = 0;
LowPassFilter Motor1_Velocity_filter = LowPassFilter(0.01);  // Tf = 10ms

double Motor2_place_last = 0;
float Motor2_Velocity = 0;
float Motor2_Velocity_f = 0;
LowPassFilter Motor2_Velocity_filter = LowPassFilter(0.01);  // Tf = 10ms


float Motor1_Target = 0;
float Motor2_Target = 0;

double RightMotorAngle = 0;
double LeftMotorAngle = 0;

float time_dt = 0;
unsigned long now_us = 0;
unsigned long now_us1 = 0;
// BLDC motor & driver instance
BLDCMotor motor1 = BLDCMotor(7);  // Motor pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(15, 7, 6, 16);

BLDCMotor motor2 = BLDCMotor(7);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(40, 39, 38, 37);

#if SensorSwitch == SENSOR_SWITCH_SPI
// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 19);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, 23);
// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
SPIClass *hspi = NULL;

#elif SensorSwitch == SENSOR_SWITCH_IIC_AS5600
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);

#endif

#if CurrentUser == CURRENT_LOOP_ON
// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense1 = InlineCurrentSense(CURRENT_SENSOR_MV_PER_AMP, 18, 17);
#endif

#if M2CurrentUser == CURRENT_LOOP_ON
// inline current sensor instance
// ACS712-05B has the resolution of 0.185mV per Amp
InlineCurrentSense current_sense2 = InlineCurrentSense(CURRENT_SENSOR_MV_PER_AMP, 35, 36);
#endif

void doMotion1(char *cmd) {
  command.motion(&motor1, cmd);
}
void doMotor1(char *cmd) {
  command.motor(&motor1, cmd);
}

void RXsbus();
int RightInverseKinematics(float x, float y, float p, float *ax);
int LeftInverseKinematics(float x, float y, float p, float *ax);
void print_data(void);
void ImuUpdate(void);
void FlashSave(int sw);
void FlashInit(void);
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
  pinMode(16, OUTPUT);
  digitalWrite(16, LOW);
  pinMode(37, OUTPUT);
  digitalWrite(37, LOW);
#endif

  Serial.begin(SENSOR_DIAGNOSTIC_MODE ? DIAGNOSTIC_SERIAL_BAUD_RATE :
               (DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? LIVE_TUNING_SERIAL_BAUD_RATE : SERIAL_BAUD_RATE));
  Logging::begin();
  FlashInit();  // Read flash data
  pinMode(BOARD_PIN_LED, OUTPUT);
  digitalWrite(BOARD_PIN_LED, LOW);  // 亮
  Serial.println("system run.");
  delay(500);

#if SENSOR_DIAGNOSTIC_MODE
  biquadFilterInitLPF(&VoltageFilterLPF, 20, DIAGNOSTIC_IMU_SAMPLE_RATE_HZ);
  for (int axis = 0; axis < 6; axis++) {
    biquadFilterInitLPF(&ImuFilterLPF[axis], 20, DIAGNOSTIC_IMU_SAMPLE_RATE_HZ);
  }
  diagnosticImuReady = initICM42688();
  sBus.begin();
  timestamp_prev = micros();
  Serial.printf("DIAG,boot,reset_reason=%d,imu_ok=%d,motors=off,servos=off\n",
                (int)esp_reset_reason(), diagnosticImuReady ? 1 : 0);
  Logging::setProfile(Logging::Profile::Diagnostic);
  return;
#endif

  ble_init();
  xTaskCreatePinnedToCore(cpu0_task, "cpu0_task", 4096, NULL, 0, NULL, 0);

  // Initialize second-order low-pass filter
  for (int axis = 0; axis < 6; axis++) {
    biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
  }

  biquadFilterInitLPF(&VoltageFilterLPF, 50, 1000);

  //  Initialize servo
  servoControl.initialize();

  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);
  _delay(555);

  servoControl.setServosAngle(1, 0, -1, 0, -1, 0, 1, 0, 1);  // Assembly position
  // IMU
  if (!initICM42688()) {
    Serial.println("ICM42688 initialization failed!");
    while (1)
      ;
  }
  Serial.println("ICM42688 initialized successfully!");

  // Remote control
  sBus.begin();

  for (int i = 0; i < 6; i++)
    biquadFilterInitLPF(&FilterLPF[i], (unsigned int)cutoffFreq, 1000);

  biquadFilterInitLPF(&FilterLPF[8], 50, 1000);
  biquadFilterInitLPF(&FilterLPF[9], 50, 1000);
  biquadFilterInitLPF(&FilterLPF[10], 200, 1000);
  biquadFilterInitLPF(&FilterLPF[11], 200, 1000);

  // use monitoring with serial
  TouchscreenInit(1000);
  // enable more verbose output for debugging
  // comment out if not needed
  SimpleFOCDebug::enable(&Serial);

#if SensorSwitch == SENSOR_SWITCH_SPI
  hspi = new SPIClass(HSPI);
  hspi->begin(18, 5, 17);  //(sck, miso, mosi)
  // initialise magnetic sensor1 hardware
  sensor1.init(hspi);
  sensor2.init(hspi);
#elif SensorSwitch == SENSOR_SWITCH_IIC_AS5600
  I2Cone.begin(4, 5, 400000);
  I2Ctwo.begin(41, 42, 400000);  // SDA1,SCL1
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
#endif

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
  // link current sense and the driver
#if CurrentUser == CURRENT_LOOP_ON
  current_sense1.linkDriver(&driver);
#endif

#if M2CurrentUser == CURRENT_LOOP_ON
  current_sense2.linkDriver(&driver2);
#endif

  // control loop type and torque mode  velocity angle
  if (CurrentUser == CURRENT_LOOP_ON)
    motor1.torque_controller = TorqueControlType::dc_current;  // foc_current   dc_current  voltage
  else
    motor1.torque_controller = TorqueControlType::voltage;

  if ((SwitchUser == SWITCH_USER_MODE_SAMPLE_TORQUE_M1) || (SwitchUser == SWITCH_USER_MODE_ANGLE_MODE))
    motor1.controller = MotionControlType::angle;
  else if (SwitchUser == SWITCH_USER_MODE_TORQUE_MODE)
    motor1.controller = MotionControlType::torque;
  else if (SwitchUser == SWITCH_USER_MODE_SPEED_MODE)
    motor1.controller = MotionControlType::velocity;

  motor1.motion_downsample = 0.0;  //

  // velocity loop PID
  motor1.PID_velocity.P = 0.006;  // 0.07;
  motor1.PID_velocity.I = 0;
  motor1.PID_velocity.D = 0.0;
  motor1.PID_velocity.output_ramp = 10000;
  motor1.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor1.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motor1.P_angle.P = 15.0;
  motor1.P_angle.I = 22.0;
  motor1.P_angle.D = 0.0;
  motor1.P_angle.output_ramp = 10000;
  motor1.P_angle.limit = 111.0;
  // Low pass filtering time constant
  motor1.LPF_angle.Tf = 0.001;
  // current q loop PID
  motor1.PID_current_q.P = 2;
  motor1.PID_current_q.I = 222;
  motor1.PID_current_q.D = 0.0;
  motor1.PID_current_q.output_ramp = 11111;
  motor1.PID_current_q.limit = 8.4;
  // Low pass filtering time constant
  motor1.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor1.PID_current_d.P = motor1.PID_current_q.P;
  motor1.PID_current_d.I = motor1.PID_current_q.I;
  motor1.PID_current_d.D = motor1.PID_current_q.D;
  motor1.PID_current_d.output_ramp = motor1.PID_current_q.output_ramp;
  motor1.PID_current_d.limit = motor1.PID_current_q.limit;
  // Low pass filtering time constant
  motor1.LPF_current_d.Tf = motor1.LPF_current_q.Tf;
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

  if (M2CurrentUser == CURRENT_LOOP_ON)
    // control loop type and torque mode velocity angle
    motor2.torque_controller = TorqueControlType::foc_current;  // foc_current   dc_current  voltage
  else
    // control loop type and torque mode velocity angle
    motor2.torque_controller = TorqueControlType::voltage;  // foc_current   dc_current  voltage

  if ((SwitchUser == SWITCH_USER_MODE_SAMPLE_TORQUE_M2) || (SwitchUser == SWITCH_USER_MODE_ANGLE_MODE))
    motor2.controller = MotionControlType::angle;
  else if (SwitchUser == SWITCH_USER_MODE_TORQUE_MODE)
    motor2.controller = MotionControlType::torque;
  else if (SwitchUser == SWITCH_USER_MODE_SPEED_MODE)
    motor2.controller = MotionControlType::velocity;

  motor2.motion_downsample = 0.0;

  // velocity loop PID
  motor2.PID_velocity.P = 0.006;
  motor2.PID_velocity.I = 0;
  motor2.PID_velocity.D = 0;
  motor2.PID_velocity.output_ramp = 10000;
  motor2.PID_velocity.limit = 8.4;
  // Low pass filtering time constant
  motor2.LPF_velocity.Tf = 0.001;
  // angle loop PID
  motor2.P_angle.P = 22;
  motor2.P_angle.I = 111;
  motor2.P_angle.D = 0;
  motor2.P_angle.output_ramp = 10000;
  motor2.P_angle.limit = 88;
  // Low pass filtering time constant
  motor2.LPF_angle.Tf = 0.001;

  // current q loop PID
  motor2.PID_current_q.P = 2;
  motor2.PID_current_q.I = 222;
  motor2.PID_current_q.D = 0.0;
  motor2.PID_current_q.output_ramp = 11111;
  motor2.PID_current_q.limit = 5.0;
  // Low pass filtering time constant
  motor2.LPF_current_q.Tf = 0.01;
  // current d loop PID
  motor2.PID_current_d.P = motor2.PID_current_q.P;
  motor2.PID_current_d.I = motor2.PID_current_q.I;
  motor2.PID_current_d.D = motor2.PID_current_q.D;
  motor2.PID_current_d.output_ramp = motor2.PID_current_q.output_ramp;
  motor2.PID_current_d.limit = motor2.PID_current_q.limit;
  // Low pass filtering time constant
  motor2.LPF_current_d.Tf = motor2.LPF_current_q.Tf;

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

#if CurrentUser == CURRENT_LOOP_ON
  // current sense init and linking
  current_sense1.init();
  motor1.linkCurrentSense(&current_sense1);
#endif

#if M2CurrentUser == CURRENT_LOOP_ON
  // current sense init and linking
  current_sense2.init();
  motor2.linkCurrentSense(&current_sense2);
#endif

  // initialise motor
  motor1.init();
  motor2.init();
  // align encoder and start FOC

  if (SwitchUser == SWITCH_USER_MODE_VIEW_ENCODER) {
    motor1.initFOC();
    motor2.initFOC();
    Serial.print("Sensor1 zero offset is:");
    Serial.print(motor1.zero_electric_angle, 6);  // Initial electrical angle
    Serial.print("  Sensor1 natural direction is: ");
    Serial.println(motor1.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");  // Motor rotation direction (clockwise, counterclockwise)

    Serial.print("Sensor2 zero offset is:");
    Serial.print(motor2.zero_electric_angle, 6);  // Initial electrical angle
    Serial.print("  Sensor2 natural direction is: ");
    Serial.println(motor2.sensor_direction == 1 ? "Direction::CW" : "Direction::CCW");  // Motor rotation direction (clockwise, counterclockwise)

    while (1)
      ;
  } else {

    motor1.initFOC();

    motor2.initFOC();
  }

  // set the inital target value
  motor1.target = 0;
  motor2.target = 0;

  // comment out if not needed

  motor1.useMonitoring(Serial);
  motor1.monitor_downsample = 10;  // disable intially

  // subscribe motor to the commander
  command.add('T', doMotion1, "motion1 control");  // Set motor target value
  command.add('M', doMotor1, "motor1");

  command.add('A', zeroBias_servo1, "my zeroBias_servo1");  // Set servo 1 bias
  command.add('B', zeroBias_servo2, "my zeroBias_servo2");  //
  command.add('C', zeroBias_servo3, "my zeroBias_servo3");  //
  command.add('D', zeroBias_servo4, "my zeroBias_servo4");  //

  command.add('H', ImuRATE_HZ, "my ImuRATE_HZ");
  command.add('Z', ImuLPF_CUTOFF_FREQ, "my ImuLPF_CUTOFF_FREQ");

  command.add('Q', TwoKp, "my TwoKp");  // MahonyFilter
  command.add('I', TwoKi, "my TwoKi");  // MahonyFilter

  command.add('K', KeyScalar, "my Select");
  command.add('E', KeyCalibration, "my CalibrationSelect");

  command.add('F', CutoffFreq, "my CutoffFreq");
  command.add('J', EnableDFilter, "my EnableDFilter");

#if AdjusParameter == ADJUST_BALANCE_SPEED_YAW_ROLL
  command.add('P', CbAnglePid, "my AnglePid");
  command.add('S', CbSpeedPid, "my SpeedPid");
  command.add('Y', CbYawPid, "my YawPid");
  command.add('R', CbRollPid, "my RollPid");
  command.add('O', Target_Leg_Length, "my Target_Leg_Length");
#elif AdjusParameter == ADJUST_BALL_PUSHING
  command.add('L', CbTouchXPid, "my CbTouchXPid");
  command.add('N', CbTouchYPid, "my CbTouchYPid");
  command.add('G', ControlTorqueCompensation, "my ControlTorqueCompensation");
#endif

  command.add('U', User_command, "my User_command");
  command.add('V', CbWheelSpeedFeedbackGain, "wheel speed feedback gain");
  command.add('W', CbDriveTiltReductionGain, "drive tilt reduction gain");

#if DIAGNOSTIC_LIVE_TUNING_DEFAULTS
  // Preload the gentle gains before CH5 can pass briefly through mode 1.
  PidParameter();
#endif
  

  // Run user commands to configure and the motor (find the full command list in docs.simplefoc.com)
  Serial.println("Motor ready.");

  _delay(1000);
  timestamp_prev = micros();
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
      LegLength = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX, 0.06, 0.09);       // Leg height
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST)  // Attitude control 2
    {
      BodyPitching = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX, -12, 12);  // Pitching       + sbus_vrb
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)              // Attitude control 3
    {
      LegLength = mapf(ble_ctrler.ch[1], BLE_CH1_MIN, BLE_CH1_MAX, 0.06, 0.07);
    }
    BodyRoll = mapf(ble_ctrler.ch[0], BLE_CH0_MIN, BLE_CH0_MAX, -0.011, 0.011);
} 
/**
 * @brief Reads and processes data from the FUTABA S.BUS remote controller.
 *
 * This function decodes the S.BUS signal, maps the raw channel values to
 * meaningful control variables like `MovementSpeed`, `BodyTurn`, `LegLength`,
 * and `BodyPitching`, and updates global state based on the RC switch positions.
 */
void RXsbus() {
  static unsigned long now_ms = millis();

  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    sbus_dt_ms = millis() - now_ms;
    now_ms = millis();
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
        LegLength = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, 992, 0.05, 0.06);  // Leg height
      else
        LegLength = mapf(sBus.channels[1], 993, SBUS_CHANNEL_MAX, 0.06, 0.09);       // Leg height
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_PITCHING_ADJUST)  // Attitude control 2
    {
      BodyPitching = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -12, 12);  // Pitching       + sbus_vrb
    } else if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)              // Attitude control 3
    {
      if (sBus.channels[1] <= 992)
        LegLength = mapf(sBus.channels[1], SBUS_CHANNEL_MIN, 992, 0.05, 0.06);
      else
        LegLength = mapf(sBus.channels[1], 993, SBUS_CHANNEL_MAX, 0.06, 0.07);

    }

    BodyRoll = mapf(sBus.channels[0], SBUS_CHANNEL_MIN, SBUS_CHANNEL_MAX, -0.011, 0.011);

  if (Voltage <= 7.4)
    Logging::message(Logging::Level::Warning, "Voltage", "%.5f", Voltage);

  }
}

/**
 * @brief Converts an angle from radians to degrees.
 * @param arc The angle in radians.
 * @return The angle in degrees.
 */
float ArcToAngle(float arc)  // Convert radians to degrees
{
  float angle = arc * (180 / PI);
  return angle;
}

/**
 * @brief Converts an angle from degrees to radians.
 * @param angle The angle in degrees.
 * @return The angle in radians.
 */
float AngleToArc(float angle)  // Convert degrees to radians
{
  float art = angle * (PI / 180);
  return art;
}

/**
 * @brief Calculates the required servo angles for the right leg using inverse kinematics.
 *
 * This function solves the geometry of the five-bar linkage for the right leg
 * to determine the two servo angles needed to place the foot at a desired
 * (x, y) coordinate relative to the body, considering the body's pitch angle.
 *
 * @param x The target horizontal position of the foot (m).
 * @param y The target vertical position (height) of the foot (m).
 * @param p The pitch angle of the robot's body (degrees).
 * @param ax A pointer to a float array where the two calculated servo angles (in degrees) will be stored.
 * @return An error code (0 for success, 1 or 2 if the target is out of reach).
 */
int RightInverseKinematics(float x, float y, float p, float *ax) {
  x = constrain(x, -0.05, 0.05);
  y = constrain(y, 0.05, 0.1);

  int error = 0;                   // Coordinate setting exception
  float AB = BODY_THIGH_LENGTH_M;  // Thigh length (m) AB=ED
  float BC = BODY_SHANK_LENGTH_M;  // Shank length (m) BC=DC

  float OA = 0.017f;  //
  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float OE = OA;  //
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = AngleToArc(p);  // Pitch angle

  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);

  OF = x1;
  FC = y1;

  // Joint 1
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);  // Joint 1 angle

  // Joint 2
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);  // Joint 1 angle

  if (AC >= (AB + BC))  // Exceeds the maximum range of the structure
    return error = 1;
  else if (EC >= (AB + BC))  // Exceeds the maximum range of the structure
    return error = 2;

  return error;
}

/**
 * @brief Calculates the required servo angles for the left leg using inverse kinematics.
 *
 * This function solves the geometry of the five-bar linkage for the left leg.
 * It mirrors the calculation of the right leg to find the servo angles needed
 * to place the foot at a desired (x, y) coordinate.
 *
 * @param x The target horizontal position of the foot (m).
 * @param y The target vertical position (height) of the foot (m).
 * @param p The pitch angle of the robot's body (degrees).
 * @param ax A pointer to a float array where the two calculated servo angles (in degrees) will be stored.
 * @return An error code (0 for success, 1 or 2 if the target is out of reach).
 */
int LeftInverseKinematics(float x, float y, float p, float *ax) {
  x = constrain(x, -0.05, 0.05);
  y = constrain(y, 0.05, 0.1);

  x = -x;
  p = -p;
  int error = 0;                   // Coordinate setting exception
  float AB = BODY_THIGH_LENGTH_M;  // Thigh length (m) AB=ED
  float BC = BODY_SHANK_LENGTH_M;  // Shank length (m) BC=DC

  float OA = 0.017f;  //
  float aOCF = 0;
  float aOCF2 = 0;
  float aAOC = 0;
  float OC = 0;
  float OF = 0;
  float FC = 0;
  float AC = 0;
  float aOAC = 0;
  float aOCA = 0;
  float aBAC = 0;
  float aBAG = 0;
  float OE = OA;  //
  float aEOC = 0;
  float EC = 0;
  float aOCE = 0;
  float aOEC = 0;
  float aDEC = 0;
  float aDEH = 0;

  float pitch, x1, y1;

  pitch = AngleToArc(p);  // Pitch angle

  x1 = x * cosf(pitch) - y * sinf(pitch);
  y1 = x * sinf(pitch) + y * cosf(pitch);

  OF = -x1;
  FC = y1;

  // Joint 1
  OC = sqrtf(pow(OF, 2) + pow(FC, 2));
  aOCF = asinf(OF / OC);
  aAOC = AngleToArc(90) + aOCF;

  AC = sqrtf(pow(OA, 2) + pow(OC, 2) - 2 * OA * OC * cos(aAOC));
  aOCA = acosf((pow(OC, 2) + pow(AC, 2) - pow(OA, 2)) / (2 * OC * AC));
  aOAC = PI - aOCA - aAOC;
  aBAC = acos((pow(AB, 2) + pow(AC, 2) - pow(BC, 2)) / (2 * AB * AC));
  aBAG = PI - aBAC - aOAC;
  ax[0] = ArcToAngle(aBAG);  // Joint 1 angle

  // Joint 2
  aOCF2 = -aOCF;
  aEOC = AngleToArc(90) + aOCF2;

  EC = sqrtf(pow(OE, 2) + pow(OC, 2) - 2 * OE * OC * cos(aEOC));
  aOCE = acosf((pow(OC, 2) + pow(EC, 2) - pow(OE, 2)) / (2 * OC * EC));
  aOEC = PI - aOCE - aEOC;
  aDEC = acos((pow(AB, 2) + pow(EC, 2) - pow(BC, 2)) / (2 * AB * EC));
  aDEH = PI - aDEC - aOEC;
  ax[1] = ArcToAngle(aDEH);  // Joint 1 angle

  if (AC >= (AB + BC))  // Exceeds the maximum range of the structure
    return error = 1;
  else if (EC >= (AB + BC))  // Exceeds the maximum range of the structure
    return error = 2;

  return error;
}

/**
 * @brief Reads data from the IMU, filters it, and updates the robot's attitude.
 *
 * This function is called periodically. It performs the following steps:
 * 1. Reads raw accelerometer and gyroscope data from the ICM42688 sensor.
 * 2. Applies pre-calibrated gyro bias offsets.
 * 3. Converts raw sensor values to standard units (g and rad/s).
 * 4. Applies a digital biquad low-pass filter to smooth the sensor data.
 * 5. Feeds the filtered data into the Mahony filter to get a stable orientation quaternion.
 * 6. Converts the quaternion to Euler angles (roll, pitch, yaw).
 * 7. Applies the roll and pitch zero-bias offsets to get the final, corrected attitude.
 * It also runs a complementary filter in parallel, likely for comparison or debugging.
 */
void ImuUpdate(void) {
  unsigned long timestamp_now = micros();
  IMUtime_dt = (timestamp_now - timestamp_prev) * 1e-6f;

  int16_t accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp;
  readIMUData(accelX, accelY, accelZ, gyroX, gyroY, gyroZ, temp);

  // Subtract the gyroscope zero bias value
  gyroX -= (int16_t)gyroBiasX;
  gyroY -= (int16_t)gyroBiasY;
  gyroZ -= (int16_t)gyroBiasZ;

  // Convert the accelerometer data to g units
  attitude.acc.x = (float)accelX * IMU_ACCEL_RANGE_G / 32768.0;
  attitude.acc.y = (float)accelY * IMU_ACCEL_RANGE_G / 32768.0;
  attitude.acc.z = (float)accelZ * IMU_ACCEL_RANGE_G / 32768.0;

  // Convert the gyroscope data to rad/s
  attitude.gyro.x = (float)gyroX * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0 * (3.1415926f / 180.0f);
  attitude.gyro.y = (float)gyroY * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0 * (3.1415926f / 180.0f);
  attitude.gyro.z = (float)gyroZ * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0 * (3.1415926f / 180.0f);

  // Software second-order low-pass filter
  attitude.gyrof.x = biquadFilterApply(&ImuFilterLPF[0], attitude.gyro.x);
  attitude.gyrof.y = biquadFilterApply(&ImuFilterLPF[1], attitude.gyro.y);
  attitude.gyrof.z = biquadFilterApply(&ImuFilterLPF[2], attitude.gyro.z);

  attitude.accf.x = biquadFilterApply(&ImuFilterLPF[3], attitude.acc.x);
  attitude.accf.y = biquadFilterApply(&ImuFilterLPF[4], attitude.acc.y);
  attitude.accf.z = biquadFilterApply(&ImuFilterLPF[5], attitude.acc.z);

  // Convert temperature data
  attitude.temp = (float)temp / 132.48 + 25;
  mahonyFilter.update(attitude.gyrof.x, attitude.gyrof.y, attitude.gyrof.z, attitude.accf.x, attitude.accf.y, attitude.accf.z, IMUtime_dt);

  // Get the filtered quaternion
  float q0_out, q1_out, q2_out, q3_out;
  mahonyFilter.getQuaternion(q0_out, q1_out, q2_out, q3_out);
  // Convert quaternion to Euler angles
  quaternionToEuler(q0_out, q1_out, q2_out, q3_out, &attitude.roll, &attitude.pitch, &attitude.yaw);

  roll_ok = attitude.roll - zeroBias.roll;
  pitch_ok = attitude.pitch - zeroBias.pitch;

  //////Complementary filter//////
  angleAccX = atan2(attitude.acc.y, attitude.acc.z + abs(attitude.acc.x)) * 360 / 2.0 / PI;
  angleAccY = atan2(attitude.acc.x, attitude.acc.z + abs(attitude.acc.y)) * 360 / -2.0 / PI;

  const float gyroRateX = (float)gyroX * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0;
  const float gyroRateY = (float)gyroY * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0;
  const float gyroRateZ = (float)gyroZ * IMU_GYRO_RANGE_DEG_PER_SEC / 32768.0;

  angleGyroX += gyroRateX * IMUtime_dt;
  angleGyroY += gyroRateY * IMUtime_dt;
  angleGyroZ += gyroRateZ * IMUtime_dt;

  angleX = (gyroCoef * (angleX + gyroRateX * IMUtime_dt)) + (accCoef * angleAccX);
  angleY = (gyroCoef * (angleY + gyroRateY * IMUtime_dt)) + (accCoef * angleAccY);
  angleZ = angleGyroZ;

  timestamp_prev = timestamp_now;
}

/**
 * @brief Initializes and reads calibration data from the ESP32's non-volatile flash memory.
 *
 * This function uses the Preferences library to load saved values for:
 * - Roll and pitch angle zero-bias offsets.
 * - Gyroscope X, Y, and Z axis bias offsets.
 * - Servo trim/offset values for all four leg servos.
 * If a value is not found in flash, it defaults to 0.0. The loaded values are printed to the serial monitor.
 */
void FlashInit(void) {
  preferences.begin("preferences", false);

  // Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
  zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
  zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

  // Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
  gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
  gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
  gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

  // Servo angle
  zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
  zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
  zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
  zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);

  // Close flash access, release related resources
  preferences.end();

  Serial.println(" ");
  // Output zero bias
  Serial.print("  Roll Zero Bias: ");
  Serial.print(zeroBias.roll);
  Serial.print("  Pitch Zero Bias: ");
  Serial.println(zeroBias.pitch);

  // Output zero bias
  Serial.print("  gyroBiasX:");
  Serial.print(gyroBiasX);
  Serial.print("  gyroBiasY:");
  Serial.print(gyroBiasY);
  Serial.print("  gyroBiasZ:");
  Serial.println(gyroBiasZ);

  // Output zero bias
  Serial.print("  servo1:");
  Serial.print(zeroBias.servo1);
  Serial.print("  servo2:");
  Serial.print(zeroBias.servo2);
  Serial.print("  servo3:");
  Serial.print(zeroBias.servo3);
  Serial.print("  servo4:");
  Serial.println(zeroBias.servo4);
}

/**
 * @brief Calculates and saves the zero-bias offset for the IMU's roll and pitch angles.
 *
 * This function should be called when the robot is stationary and level.
 * It accumulates a number of IMU readings (defined by `CALL_COUNT`),
 * calculates the average roll and pitch, and saves these averages to flash
 * memory as the zero-bias offset. This ensures the robot knows what "level" is.
 */
void calculateZeroBias() {
  // Initialize the accumulator
  static float rollSum = 0;
  static float pitchSum = 0;

  // Accumulate the Euler angle
  rollSum += attitude.roll;
  pitchSum += attitude.pitch;

  // Increase the call counter
  IMUCallCounter++;

  if (IMUCallCounter >= IMU_CALL_COUNT) {
    // Calculate the average value to get the zero bias
    zeroBias.roll = rollSum / IMU_CALL_COUNT;
    zeroBias.pitch = pitchSum / IMU_CALL_COUNT;

    // Initialize flash access, open the "preferences" namespace
    // The second parameter is false, indicating that the namespace is opened in write mode
    preferences.begin("preferences", false);

    // Write data
    zeroBiasFlash[0] = zeroBias.roll;
    preferences.putFloat(zeroBiasKeys[0], zeroBiasFlash[0]);
    zeroBiasFlash[1] = zeroBias.pitch;
    preferences.putFloat(zeroBiasKeys[1], zeroBiasFlash[1]);
    // Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
    zeroBias.roll = preferences.getFloat(zeroBiasKeys[0], 0.0);
    zeroBias.pitch = preferences.getFloat(zeroBiasKeys[1], 0.0);

    // Close flash access, release related resources
    preferences.end();

    // Output zero bias
    if (Logging::profile() == Logging::Profile::Idle) {
      Serial.print("  Roll Zero Bias: ");
      Serial.print(zeroBias.roll);
      Serial.print("  Pitch Zero Bias: ");
      Serial.println(zeroBias.pitch);
    }
    rollSum = 0;
    pitchSum = 0;
    IMUCallCounter = 0;     // Clear the next time
    CalibrationSelect = 0;  // Calibration complete exit calibration
  }
}

/**
 * @brief Manages the saving of different calibration profiles to flash memory.
 *
 * This function is controlled by the `CalibrationSelect` variable, which is set
 * via the serial commander.
 *
 * @param sw The calibration mode to execute:
 *           - 1: Calibrates the gyroscope and saves its bias values.
 *           - 2: Calls `calculateZeroBias()` to calibrate the roll/pitch angle offsets.
 *           - 3: Saves any adjustments made to the servo trim values.
 */
void FlashSave(int sw) {

  static float servo1_last = zeroBias.servo1;  // Last deviation
  static float servo2_last = zeroBias.servo2;
  static float servo3_last = zeroBias.servo3;
  static float servo4_last = zeroBias.servo4;

  switch (sw) {
    case 1:
      // Gyroscope calibration
      calibrateGyro();
      preferences.begin("preferences", false);

      // Write data
      preferences.putFloat(zeroBiasKeys[2], gyroBiasX);
      preferences.putFloat(zeroBiasKeys[3], gyroBiasY);
      preferences.putFloat(zeroBiasKeys[4], gyroBiasZ);

      // Read data   If the read fails (that is, the data does not exist in the flash), the default value is 0.0
      gyroBiasX = preferences.getFloat(zeroBiasKeys[2], 0.0);
      gyroBiasY = preferences.getFloat(zeroBiasKeys[3], 0.0);
      gyroBiasZ = preferences.getFloat(zeroBiasKeys[4], 0.0);

      // Close flash access, release related resources
      preferences.end();

      // Output zero bias
      if (Logging::profile() == Logging::Profile::Idle) {
        Serial.print("  gyroBiasX:");
        Serial.print(gyroBiasX);
        Serial.print("  gyroBiasY:");
        Serial.print(gyroBiasY);
        Serial.print("  gyroBiasZ:");
        Serial.println(gyroBiasZ);
      }

      CalibrationSelect = 0;  // Calibration complete
      break;

    case 2:
      // Function to calculate the Euler angle zero bias
      calculateZeroBias();

      break;

    case 3:

      preferences.begin("preferences", false);

      if (zeroBias.servo1 != servo1_last)  // Parameter adjusted, save
      {
        servo1_last = zeroBias.servo1;  //
        // Write data
        preferences.putFloat(zeroBiasKeys[5], zeroBias.servo1);
        // Read servo angle
        zeroBias.servo1 = preferences.getFloat(zeroBiasKeys[5], 0.0);
        // Print data
        if (Logging::profile() == Logging::Profile::Idle) {
          Serial.print("  zeroBias.servo1:");
          Serial.println(zeroBias.servo1);
        }
      }

      if (zeroBias.servo2 != servo2_last)  // Parameter adjusted, save
      {
        servo2_last = zeroBias.servo2;  //
        // Write data
        preferences.putFloat(zeroBiasKeys[6], zeroBias.servo2);
        // Read servo angle
        zeroBias.servo2 = preferences.getFloat(zeroBiasKeys[6], 0.0);
        // Print data
        if (Logging::profile() == Logging::Profile::Idle) {
          Serial.print("  zeroBias.servo2:");
          Serial.println(zeroBias.servo2);
        }
      }

      if (zeroBias.servo3 != servo3_last)  // Parameter adjusted, save
      {
        servo3_last = zeroBias.servo3;  //
        // Write data
        preferences.putFloat(zeroBiasKeys[7], zeroBias.servo3);
        // Read servo angle
        zeroBias.servo3 = preferences.getFloat(zeroBiasKeys[7], 0.0);
        // Print data
        if (Logging::profile() == Logging::Profile::Idle) {
          Serial.print("  zeroBias.servo3:");
          Serial.println(zeroBias.servo3);
        }
      }

      if (zeroBias.servo4 != servo4_last)  // Parameter adjusted, save
      {
        servo4_last = zeroBias.servo4;  //
        // Write data
        preferences.putFloat(zeroBiasKeys[8], zeroBias.servo4);
        // Read servo angle
        zeroBias.servo4 = preferences.getFloat(zeroBiasKeys[8], 0.0);
        // Print data
        if (Logging::profile() == Logging::Profile::Idle) {
          Serial.print("  zeroBias.servo4:");
          Serial.println(zeroBias.servo4);
        }
      }

      // Close flash access, release related resources
      preferences.end();

      break;

    default:

      break;
  }
}

/**
 * @brief Prints debugging data to the serial port based on a selection variable.
 *
 * This function uses a large switch statement controlled by the global `Select`
 * variable (which can be set via the serial commander). Each case prints a
 * different set of variables, such as IMU data, PID controller states, motor
 * velocities, S.BUS channels, etc. This is a flexible way to debug different
 * parts of the system without recompiling.
 */
void print_data(void) {
  const int selection = (int)Select;
  const uint32_t nowMs = millis();
  if (selection >= 60 && selection <= 75) {
    const Logging::DebugSelector selector = static_cast<Logging::DebugSelector>(selection);
    if (!RobotLogCapture::selectedDebugDue(selector, nowMs)) return;
    Logging::DebugSample sample{};
    sample.selector = selector;
    sample.payload.channelState = {nowMs * 0.001f, sBus.channels[selection - 60]};
    Logging::submit(sample);
    return;
  }

  const Logging::DebugSelector selector = static_cast<Logging::DebugSelector>(selection);
  if (selection < 1 || selection > 45 || !RobotLogCapture::selectedDebugDue(selector, nowMs)) return;
  Logging::DebugSample sample{};
  sample.selector = selector;
  switch (selection) {
    case 1:
      sample.payload.timedVector3 = {time_dt, attitude.roll, attitude.pitch, attitude.yaw};
      break;
    case 2:
      sample.payload.timedVector3 = {time_dt, attitude.acc.x, attitude.acc.y, attitude.acc.z};
      break;
    case 3:
      sample.payload.timedVector3 = {time_dt, attitude.gyro.x, attitude.gyro.y, attitude.gyro.z};
      break;
    case 4:
      sample.payload.timedVector3 = {time_dt, attitude.roll - zeroBias.roll,
                                     attitude.pitch - zeroBias.pitch,
                                     attitude.yaw - zeroBias.yaw};
      break;
    case 5:
      sample.payload.timedVector3 = {time_dt, zeroBias.roll, zeroBias.pitch, zeroBias.yaw};
      break;
    case 6:
      sample.payload.motorVelocities = {Motor1_Velocity, Motor2_Velocity};
      break;
    case 7:
      sample.payload.pair = {Motor1_Velocity, Motor1_Velocity_f};
      break;
    case 8:
      for (int i = 0; i < 10; ++i) sample.payload.receiverChannels.channels[i] = sBus.channels[i];
      sample.payload.receiverChannels.frameDtMs = sbus_dt_ms;
      break;
    case 9:
      sample.payload.controllerGains = {{Angle_Pid.Kp, Angle_Pid.Ki, Angle_Pid.Kd},
                                        {Speed_Pid.Kp, Speed_Pid.Ki, Speed_Pid.Kd},
                                        {Yaw_Pid.Kp, Yaw_Pid.Ki, Yaw_Pid.Kd}, time_dt};
      break;
    case 10:
      sample.payload.mahony = {mahonyFilter.twoKp, mahonyFilter.twoKi, attitude.roll,
                               attitude.pitch, IMUtime_dt};
      break;
    case 11:
      sample.payload.imuAxes = {angleX, angleY, angleZ, angleGyroX, angleGyroY, angleGyroZ, IMUtime_dt};
      break;
    case 12:
      sample.payload.pair = {angleX, attitude.roll};
      break;
    case 13:
      sample.payload.timedVector3 = {time_dt, attitude.gyrof.x, attitude.gyrof.y, attitude.gyrof.z};
      break;
    case 14:
      sample.payload.timedVector3 = {time_dt, attitude.accf.x, attitude.accf.y, attitude.accf.z};
      break;
    case 15:
      sample.payload.pair = {attitude.acc.y, attitude.accf.y};
      break;
    case 16:
      sample.payload.pair = {attitude.gyro.y, attitude.gyrof.y};
      break;
    case 17:
      sample.payload.currentSetpoint = {motor2.current_sp};
      break;
    case 18:
      sample.payload.sensorAngles = {motor2.target, sensor1.getAngle(), sensor1.getMechanicalAngle(),
                                     sensor2.getAngle(), sensor2.getMechanicalAngle()};
      break;
    case 19:
      sample.payload.servoOffsets = {{zeroBias.servo1, zeroBias.servo2, zeroBias.servo3, zeroBias.servo4}};
      break;
    case 20:
      sample.payload.ballBalanceGeometry = {top_ball_x, BodyRoll, LegLength};
      break;
    case 21:
      sample.payload.balanceState = {roll_ok, BodyPitching};
      break;
    case 22:
      sample.payload.pidIntegralState = {Angle_Pid.iLimit, Angle_Pid.integral,
                                         Angle_Pid.outI, Angle_Pid.output};
      break;
    case 23:
      sample.payload.pidIntegralOutputState = {Speed_Pid.iLimit, Speed_Pid.integral,
                                               Speed_Pid.outI, BodyPitching_f, Speed_Pid.output};
      break;
    case 24:
      sample.payload.pair = {enableDFilter, cutoffFreq};
      break;
    case 25:
      sample.payload.bodyPitchState = {BodyPitching_f, BodyPitching};
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
      sample.payload.filteredGeometry = {BodyPitching_f, BodyRoll_f, LegLength_f, SlideStep_f,
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
                                               Yaw_Pid.outI, BodyPitching_f, Yaw_Pid.output};
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
      if (Touch.state == 1) {
        sample.payload.touchPoint = {Touch.state, Touch.XPressDat, Touch.YPressDat};
      } else if (Touch.state == 0) {
        sample.payload.touchPoint = {Touch.state, Touch.XPressDat, Touch.YPressDat};
      } else return;
      break;
    case 41:
      sample.payload.rollOutput = {roll_ok, BodyPitching, Speed_Pid.output};
      break;
    case 42:
      sample.payload.rollCorrection = {roll_ok, BodyPitching,
                                       BodyPitchingCorrect(BodyPitching_f)};
      break;
    case 43:
      sample.payload.voltageState = {VoltageADC, VoltageADCf, Voltage};
      break;
    case 44:
      sample.payload.tuningState = {PidParameterTuning, TargetLegLength};
      break;
    case 45:
      sample.payload.tumbleState = {RobotTumble, roll_ok, Angle_Pid.error};
      break;
    default: return;
  }
  Logging::submit(sample);
  switch ((int)Select) {
    case 55: {
      if (controlGateSequence % 7 == 0) {
        const uint32_t traceMs = millis();
        Logging::TraceSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.postureOrMarkMode = posture_or_mark_mode;
        sample.minimumBatteryRawV = (float)7.77 / 813.43 * VoltageADCMin;
        sample.batteryV = Voltage;
        sample.rollOk = roll_ok;
        sample.pitchOk = pitch_ok;
        int32_t ranges[4];
        RobotLogCapture::copyServoRanges(ranges);
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
        const float rawMinV = (float)7.77 / 813.43 * VoltageADCMin;
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_TUMBLE_NO;
        int32_t ranges[4];
        RobotLogCapture::copyServoRanges(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::ControlSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = rawMinV;
        sample.batteryV = Voltage;
        sample.rollOk = roll_ok;
        sample.yawRateRadPerSec = attitude.gyro.z;
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
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_TUMBLE_NO;
        int32_t ranges[4];
        RobotLogCapture::copyServoRanges(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::BalanceSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = (float)7.77 / 813.43 * VoltageADCMin;
        sample.rollOk = roll_ok;
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
        const bool active = pid_gains_mode_is_enabled(pid_gains_mode) && RobotTumble == ROBOT_TUMBLE_NO;
        int32_t ranges[4];
        RobotLogCapture::copyServoRanges(ranges);
        const int maxServoRange = max(max(ranges[0], ranges[1]), max(ranges[2], ranges[3]));
        Logging::DriveSample sample{};
        sample.timestampMs = traceMs;
        sample.sequence = activeTraceSequence++;
        sample.gainMode = pid_gains_mode;
        sample.minimumBatteryRawV = (float)7.77 / 813.43 * VoltageADCMin;
        sample.batteryV = Voltage;
        sample.requestedSpeed = MovementSpeed;
        sample.effectiveSpeed = active ? driveEffectiveSpeed : 0.0f;
        sample.leftWheelVelocity = Motor1_Velocity_f;
        sample.rightWheelVelocity = Motor2_Velocity_f;
        sample.controlDtSec = time_dt;
        sample.speedError = active ? Speed_Pid.error : 0.0f;
        sample.speedProportional = active ? Speed_Pid.outP : 0.0f;
        sample.speedIntegral = active ? Speed_Pid.outI : 0.0f;
        sample.speedDerivative = active ? Speed_Pid.outD : 0.0f;
        sample.speedOutput = active ? Speed_Pid.output : 0.0f;
        sample.driveBodyXRaw = active ? driveSpeedBodyXRaw : 0.0f;
        sample.bodyX = active ? BodyX : 0.0f;
        sample.bodyPitchFiltered = BodyPitching_f;
        sample.rollOk = roll_ok;
        sample.angleOutput = active ? Angle_Pid.output : 0.0f;
        sample.angleProportional = active ? Angle_Pid.outP : 0.0f;
        sample.angleIntegral = active ? Angle_Pid.outI : 0.0f;
        sample.angleDerivative = active ? Angle_Pid.outD : 0.0f;
        sample.wheelSpeedFeedbackOutput = active ? wheelSpeedFeedbackOutput : 0.0f;
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
    RollPid.P = PID_ROLL_P_NO_TOUCH;
    RollPid.I = PID_ROLL_I_NO_TOUCH;
    RollPid.D = PID_ROLL_D_NO_TOUCH;
    RollPid.limit = PID_ROLL_LIMIT_NO_TOUCH;  // Integral limit

    // Speed loop
    SpeedPid.P = PID_SPEED_P_NO_TOUCH;
    SpeedPid.I = PID_SPEED_I_NO_TOUCH;
    SpeedPid.D = PID_SPEED_D_NO_TOUCH;
    SpeedPid.limit = PID_SPEED_LIMIT_NO_TOUCH;  // Integral limit

    // Balance loop
    AnglePid.P = PID_ANGLE_P_NO_TOUCH;
    AnglePid.I = PID_ANGLE_I_NO_TOUCH;
    AnglePid.D = PID_ANGLE_D_NO_TOUCH;
    AnglePid.limit = PID_ANGLE_LIMIT_NO_TOUCH;                                    // Integral limit
  } else if (gainMode == REMOTE_CONTROL_PID_GAINS_MODE_ON_WITH_TOUCH)  // With touch screen
  {
    // Roll
    RollPid.P = PID_ROLL_P_WITH_TOUCH;
    RollPid.I = PID_ROLL_I_WITH_TOUCH;
    RollPid.D = PID_ROLL_D_WITH_TOUCH;
    RollPid.limit = PID_ROLL_LIMIT_WITH_TOUCH;  // Integral limit

    // Speed loop
    SpeedPid.P = PID_SPEED_P_WITH_TOUCH;
    SpeedPid.I = PID_SPEED_I_WITH_TOUCH;
    SpeedPid.D = PID_SPEED_D_WITH_TOUCH;
    SpeedPid.limit = PID_SPEED_LIMIT_WITH_TOUCH;  // Integral limit

    // Balance loop
    AnglePid.P = PID_ANGLE_P_WITH_TOUCH;
    AnglePid.I = PID_ANGLE_I_WITH_TOUCH;
    AnglePid.D = PID_ANGLE_D_WITH_TOUCH;
    AnglePid.limit = PID_ANGLE_LIMIT_WITH_TOUCH;  // Integral limit
  }

  YawPid.P = 4;
  YawPid.I = 0;
  YawPid.D = 0;
  YawPid.limit = 0;

#if DIAGNOSTIC_LIVE_TUNING_DEFAULTS
  AnglePid.P = 5;
  AnglePid.I = 200;
  AnglePid.D = 0.11;
  AnglePid.limit = 0.1;
  // Keep the verified live-tuning gains as the startup defaults.
  SpeedPid.P = 0.045;
  SpeedPid.I = 0.005;
  SpeedPid.D = 0;
  SpeedPid.limit = 50;
  YawPid.P = 4;
  YawPid.I = 0;
  YawPid.D = 0;
  YawPid.limit = 0;
#endif

  // Touch screen
  TouchXPid.P = 0.2;
  TouchXPid.I = 0;
  TouchXPid.D = 0.04;
  TouchXPid.limit = 0;  // Integral limit

  TouchYPid.P = 0.2;
  TouchYPid.I = 0;
  TouchYPid.D = 0.08;
  TouchYPid.limit = 0;  // Integral limit
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
  TouchX_Pid.Kp = TouchXPid.P / 100;
  TouchX_Pid.Ki = TouchXPid.I / 100;
  TouchX_Pid.Kd = TouchXPid.D / 100;
  TouchX_Pid.iLimit = TouchXPid.limit;  // Integral limit

  TouchY_Pid.Kp = TouchYPid.P / 100;
  TouchY_Pid.Ki = TouchYPid.I / 100;
  TouchY_Pid.Kd = TouchYPid.D / 100;
  TouchY_Pid.iLimit = TouchYPid.limit;  // Integral limit

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
  if ((int)enableDFilter == 1) {
    TouchY_Pid_outputF = biquadFilterApply(&FilterLPF[10], TouchY_Pid.output);
    TouchX_Pid_outputF = biquadFilterApply(&FilterLPF[11], TouchX_Pid.output);
  } else {
    TouchY_Pid_outputF = TouchY_Pid.output;
    TouchX_Pid_outputF = TouchX_Pid.output;
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
    TouchY_Pid_outputF = 0;
    TouchX_kd = 0;
    TouchY_kd = 0;
  }

  // Roll
  Roll_Pid.Kp = RollPid.P / 100;
  Roll_Pid.Ki = RollPid.I / 100;
  Roll_Pid.Kd = RollPid.D / 100;
  Roll_Pid.iLimit = RollPid.limit;  // Integral limit

  float TargetBodyRoll = BodyRoll_f * 777;                            // Roll
  if (attitude_mode == REMOTE_CONTROL_ATTITUDE_MODE_BALL_POISE)  // Top ball禁止手动横滚
    TargetBodyRoll = 0;
  float RollError = (-pitch_ok) - (-TargetBodyRoll) - (-TouchY_Pid_outputF);
  if (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO)  // Roll leveling
  {
    Roll_Pid.compute(RollError, dt);
  } else {
    Roll_Pid.output = 0;
    Roll_Pid.integral = 0;
  }

  // Speed loop
  Speed_Pid.Kp = SpeedPid.P / 100;
  Speed_Pid.Ki = SpeedPid.I / 100;
  Speed_Pid.Kd = SpeedPid.D / 100;
  Speed_Pid.iLimit = SpeedPid.limit;  // Integral limit

  const float avgVelocity = 0.5f * (Motor1_Velocity_f + Motor2_Velocity_f);
  driveEffectiveSpeed = MovementSpeed;
  // Reduce only further acceleration when tilt consumes balance headroom.
  if (MovementSpeed * avgVelocity >= 0.0f && fabsf(MovementSpeed) > fabsf(avgVelocity)) {
    const float tiltFraction = constrain((fabsf(roll_ok) - DRIVE_TILT_REDUCTION_START_DEG) /
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

  BodyX = speedBodyX + BodyPitchingCorrect(BodyPitching_f);

  // Balance loop
  Angle_Pid.Kp = AnglePid.P;
  Angle_Pid.Ki = AnglePid.I;
  Angle_Pid.Kd = AnglePid.D;
  Angle_Pid.iLimit = AnglePid.limit;  // Integral limit

  float angleError = roll_ok - (-BodyPitching_f);  // Measured value minus target value
  if (balancePidNeedsPriming) {
    // Avoid a derivative kick when CH5 first enables the balance loop.
    Angle_Pid.previousError = angleError;
    Angle_Pid.integral = 0;
    balancePidNeedsPriming = false;
  }
  float angleOutput = Angle_Pid.compute(angleError, dt);

  // Yaw loop
  Yaw_Pid.Kp = YawPid.P;
  Yaw_Pid.Ki = YawPid.I;
  Yaw_Pid.Kd = YawPid.D;
  Yaw_Pid.iLimit = YawPid.limit;
  if (roll_mode != REMOTE_CONTROL_ROLL_MODE_AUTO)  // Lock heading angle
  {
    Yaw_Pid.Ki = 0;
    Yaw_Pid.integral = 0;
  }

  float yawError = attitude.gyro.z - BodyTurn;  // Measured value minus target value
  float yawOutput = Yaw_Pid.compute(yawError, dt);

  // Use the existing wheel path only to oppose measured overspeed.
  const float boundedWheelGain = constrain(wheelSpeedFeedbackGain, 0.0f, 0.4f);
  const bool braking = avgVelocity * (avgVelocity - MovementSpeed) > 0.0f;
  const float wheelCorrection = braking ? boundedWheelGain * (avgVelocity - MovementSpeed) : 0.0f;
  wheelSpeedFeedbackOutput = constrain(wheelCorrection, -DRIVE_WHEEL_FEEDBACK_LIMIT,
                                      DRIVE_WHEEL_FEEDBACK_LIMIT);
  float target1 = angleOutput - yawOutput + wheelSpeedFeedbackOutput;
  float target2 = angleOutput + yawOutput + wheelSpeedFeedbackOutput;

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
 * (`BodyPitching`, `BodyRoll`, `LegLength`, etc.) using biquad low-pass filters.
 * This prevents jerky movements and improves the stability of the robot's response
 * to user commands. The filter cutoff frequency can be adjusted live.
 */
void RemoteControlFiltering(void)  // Remote control filter
{
  static int enableDFilter_last = (int)enableDFilter;
  static int cutoffFreq_last = (int)cutoffFreq;

  sbus_top_ball_x_smoothed = biquadFilterApply(&FilterLPF[8], top_ball_x);
  sbus_top_ball_y_smoothed = biquadFilterApply(&FilterLPF[9], top_ball_y);

  if ((int)enableDFilter == 1) {
    BodyPitching_f = biquadFilterApply(&FilterLPF[0], BodyPitching);
    BodyRoll_f = biquadFilterApply(&FilterLPF[1], BodyRoll);
    LegLength_f = biquadFilterApply(&FilterLPF[2], LegLength);
    SlideStep_f = biquadFilterApply(&FilterLPF[3], SlideStep);
  } else {
    BodyPitching_f = BodyPitching;
    BodyRoll_f = BodyRoll;
    LegLength_f = LegLength;
    SlideStep_f = SlideStep;
  }

  if (((int)enableDFilter != enableDFilter_last) || ((int)cutoffFreq != cutoffFreq_last)) {
    for (int i = 0; i < 6; i++) {
      biquadFilterInitLPF(&FilterLPF[i], (unsigned int)cutoffFreq, 1000);
    }

    enableDFilter_last = (int)enableDFilter;
    cutoffFreq_last = (int)cutoffFreq;
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
  VoltageADC = analogRead(BOARD_PIN_ANALOG_IN);
  if (VoltageADCMin == 0 || VoltageADC < VoltageADCMin)
    VoltageADCMin = VoltageADC;
  VoltageADCf = biquadFilterApply(&VoltageFilterLPF, VoltageADC);
  Voltage = (float)7.77 / 813.43 * VoltageADCf;
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
  if (abs(roll_ok) >= 35) {
    x++;
    if (x >= 20) {
      x = 20;
      RobotTumble = ROBOT_TUMBLE_YES;  // Machine fall
    }
  } else {
    if ((RobotTumble == ROBOT_TUMBLE_YES) && (abs(roll_ok) <= 5))  // Machine fall after fall
    {
      x--;
      if (x <= 0) {
        x = 0;
        RobotTumble = ROBOT_TUMBLE_NO;
      }
    }
  }
}

void DiagnosticLoop(void) {
  static uint32_t lastImuScheduleUs = 0;
  static unsigned long lastVoltageMs = 0;
  const unsigned long nowMs = millis();
  const uint32_t nowUs = micros();

  sBus.FeedLine();
  if (sBus.toChannels == 1) {
    sBus.UpdateChannels();
    sBus.toChannels = 0;
    diagnosticLastRcFrameMs = nowMs;
    diagnosticHasRcFrame = true;
  }

  if (diagnosticImuReady && (uint32_t)(nowUs - lastImuScheduleUs) >= DIAGNOSTIC_IMU_INTERVAL_US) {
    lastImuScheduleUs = nowUs;
    ImuUpdate();
    const long rcAgeMs = diagnosticHasRcFrame ? (long)(nowMs - diagnosticLastRcFrameMs) : -1;
    const int rcFailsafe = diagnosticHasRcFrame ? sBus.Failsafe() : -1;
    const float batteryRawV = (float)7.77 / 813.43 * VoltageADC;
    Logging::DiagnosticSample sample{};
    sample.timestampUs = (uint32_t)timestamp_prev;
    sample.sequence = diagnosticSequence++;
    sample.gyroX = attitude.gyro.x;
    sample.gyroY = attitude.gyro.y;
    sample.gyroZ = attitude.gyro.z;
    sample.accelX = attitude.acc.x;
    sample.accelY = attitude.acc.y;
    sample.accelZ = attitude.acc.z;
    sample.rollDeg = attitude.roll;
    sample.pitchDeg = attitude.pitch;
    sample.yawDeg = attitude.yaw;
    sample.batteryRawV = batteryRawV;
    sample.receiverFrameAgeMs = rcAgeMs;
    sample.receiverFailsafe = rcFailsafe;
    sample.imuReady = diagnosticImuReady ? 1 : 0;
    Logging::submit(sample);
  }
  if (nowMs - lastVoltageMs >= 10) {
    lastVoltageMs = nowMs;
    ReadVoltage();
  }
  delay(1);
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
  now_us = micros();

  // iterative function setting the outter loop target

  if (Communication_object == COMMUNICATION_OBJECT_SIMPLEFOC_STUDIO) {
    if (Logging::profile() == Logging::Profile::Idle) motor1.monitor();
  } else if (Communication_object == COMMUNICATION_OBJECT_CONTROL_DUAL_MOTORS) {
    motor2.target = motor1.target;
  }

  motor1.move();
  motor2.move();

  // Torque compensation
  float indexF = sensor1.getMechanicalAngle() / AngleResolutionRatio;
  int index = round(indexF);
  if ((TorqueCompensation == TORQUE_COMPENSATION_ON) && (SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M1) && (SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M2)) {
    motor1.current_sp = motor1.current_sp + Motor1_Current_sp_data[index];
  }

  indexF = sensor2.getMechanicalAngle() / AngleResolutionRatio;
  index = round(indexF);
  if ((TorqueCompensation == TORQUE_COMPENSATION_ON) && (SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M1) && (SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M2)) {
    motor2.current_sp = motor2.current_sp + Motor2_Current_sp_data[index];
  }

  // iterative setting FOC phase voltage
  motor1.loopFOC();
  motor2.loopFOC();


  RightMotorAngle = -sensor1.getPreciseAngle();
  LeftMotorAngle = sensor2.getPreciseAngle();

  // user communication
  command.run();

  ImuUpdate();  // Update IMU data
  CtrlInput();  // BLE or remote control input
  RXsbus();

  ReadTouchDat();

  time_dt = (now_us - now_us1) / 1000000.0f;
  if (time_dt >= 0.001f) {   //1kHz
    controlGateSequence++;
    TouchBiquadFilter();  // Touch screen filter

    RemoteControlFiltering();  // Remote control signal filtering
    ReadVoltage();             // Battery
    print_data();              // Serial port data printing

    Robot_Tumble();  // Machine fall detection
    LED_count++;
    if (LED_count >= LED_dt) {
      LED_count = 0;
      if (LED_HL == 1) {
        digitalWrite(BOARD_PIN_LED, LOW);  // On
        LED_HL = 0;
      } else {
        digitalWrite(BOARD_PIN_LED, HIGH);  // Off
        LED_HL = 1;
      }
    }

    if (Voltage <= 7.4)
      LED_dt = 20;
    else
      LED_dt = 100;

    if (RATE_HZ != RATE_HZ_last) {
      // Initialize second-order low-pass filter
      for (int axis = 0; axis < 6; axis++) {
        biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
      }
      Logging::message(Logging::Level::Info, "RATE_HZ", "%.3f", RATE_HZ);
      RATE_HZ_last = RATE_HZ;
    }
    if (LPF_CUTOFF_FREQ != LPF_CUTOFF_FREQ_last) {
      // Initialize second-order low-pass filter
      for (int axis = 0; axis < 6; axis++) {
        biquadFilterInitLPF(&ImuFilterLPF[axis], (unsigned int)LPF_CUTOFF_FREQ, (unsigned int)RATE_HZ);
      }
      Logging::message(Logging::Level::Info, "LPF_CUTOFF_FREQ", "%.3f", LPF_CUTOFF_FREQ);
      LPF_CUTOFF_FREQ_last = LPF_CUTOFF_FREQ;
    }

    FlashSave((int)CalibrationSelect);  // Save calibration data

    const float wheelVelocityDt = DIAGNOSTIC_LIVE_TUNING_DEFAULTS ? time_dt : 0.01f;
    Motor1_Velocity = (sensor1.getAngle() - Motor1_place_last) / wheelVelocityDt;
    Motor1_Velocity_f = Motor1_Velocity_filter(Motor1_Velocity);
    Motor1_place_last = sensor1.getAngle();

    Motor2_Velocity = -(sensor2.getAngle() - Motor2_place_last) / wheelVelocityDt;
    Motor2_Velocity_f = Motor2_Velocity_filter(Motor2_Velocity);
    Motor2_place_last = sensor2.getAngle();

    if ((SwitchUser == SWITCH_USER_MODE_SAMPLE_TORQUE_M1) && (Slot_calibration_mark == 0)) {
      if (Logging::profile() == Logging::Profile::Idle) Serial.print(" motor1 ");
      CalibrationCurrentSp(-sensor1.getAngle(), Motor1_Velocity_f, &motor1);
    }
    if ((SwitchUser == SWITCH_USER_MODE_SAMPLE_TORQUE_M2) && (Slot_calibration_mark == 0)) {
      if (Logging::profile() == Logging::Profile::Idle) Serial.print(" motor2 ");
      CalibrationCurrentSp(sensor2.getAngle(), Motor2_Velocity_f, &motor2);
    }

    float Rax[2];
    float Lax[2];
    float bodyH = 0.06f;
    float bodyRoll = BodyRoll_f;

    if ((pid_gains_mode == REMOTE_CONTROL_PID_GAINS_MODE_OFF) || (RobotTumble == ROBOT_TUMBLE_YES)) {
      balancePidNeedsPriming = true;
      if (Communication_object == COMMUNICATION_OBJECT_TWO_WHEEL_BALANCE && SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M1 && SwitchUser != SWITCH_USER_MODE_SAMPLE_TORQUE_M2)  //
      {
        motor1.target = 0;
        motor2.target = 0;
      }

      bodyH = 0.06;
      BodyX = 0;
      bodyRoll = 0;
      BodyPitching_f = 0;

      Angle_Pid.integral = 0;
      Speed_Pid.integral = 0;
      Yaw_Pid.integral = 0;
      wheelSpeedFeedbackOutput = 0;
      driveEffectiveSpeed = 0;
      driveSpeedBodyXRaw = 0;

    } else if ((pid_gains_mode_is_enabled(pid_gains_mode)) && (RobotTumble == ROBOT_TUMBLE_NO)) {
      PIDcontroller_posture(time_dt);  // PID controller

      if (roll_mode == REMOTE_CONTROL_ROLL_MODE_AUTO)
        bodyRoll = Roll_Pid.output;

      if (TargetLegLength == 0)
        bodyH = LegLength_f;
      else
        bodyH = TargetLegLength;
    }

    if (RobotTumble == ROBOT_TUMBLE_YES)  // Machine fall
    {
      bodyH = 0.06;
      BodyX = 0;
      bodyRoll = 0;
      BodyPitching_f = 0;
    }

    if (RightInverseKinematics(BarycenterX - BodyX, bodyH - bodyRoll, BodyPitching_f, Rax))
      Logging::message(Logging::Level::Warning, "RightInverseKinematics", "no");

    if (LeftInverseKinematics(BarycenterX - BodyX, bodyH + bodyRoll, BodyPitching_f, Lax))
      Logging::message(Logging::Level::Warning, "LeftInverseKinematics", "no");

    if (posture_or_mark_mode == REMOTE_CONTROL_PM_POSTURE_MODE)  // Posture
    {

      // Set the angle of the four servos
      servoTraceAngle[0] = Lax[0] - zeroBias.servo1;
      servoTraceAngle[1] = Lax[1] - zeroBias.servo2;
      servoTraceAngle[2] = Rax[0] - zeroBias.servo3;
      servoTraceAngle[3] = Rax[1] - zeroBias.servo4;
      servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1, servoTraceAngle[3], 1);
    } else  // Assembly position and calibration
    {
      if ((int)CalibrationSelect == 3)  // Servo calibration
      {
        servoTraceAngle[0] = -zeroBias.servo1;
        servoTraceAngle[1] = -zeroBias.servo2;
        servoTraceAngle[2] = -zeroBias.servo3;
        servoTraceAngle[3] = -zeroBias.servo4;
      } else {
        servoTraceAngle[0] = servoTraceAngle[1] = servoTraceAngle[2] = servoTraceAngle[3] = 0;
      }
      servoControl.setServosAngle(1, servoTraceAngle[0], -1, servoTraceAngle[1], -1, servoTraceAngle[2], 1, servoTraceAngle[3], 1);
    }

    RobotLogCapture::observeServoAngles(servoTraceAngle);

    now_us1 = now_us;
  }
}
