#pragma once

// Host simulation of the balancing robot: the firmware's balance path (ControlTick, BalanceController, imu) against a
// model of the robot. Model and usage: docs/robot/software/simulation.md. SI units, angles in radians.
//
// Signs: forward, forward lean (pitch) and left turn (yaw) are positive. Motor-local values (SimpleFOC) are
// WheelBoundary::kLeftMotorSign / kRightMotorSign times the forward value.

#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "ControlTick.h"
#include "ICM42688.h"
#include "Logging.h"
#include "RobotConfig.h"
#include "signal_table.h"

namespace sim {

// ----- Plant (plant.cpp)

// Measured and fitted values carry their date; values marked placeholder are still estimates (docs/robot/software/simulation.md).
struct PlantParams {
  double bodyMassKg = 0.594;                      // measured 2026-11-01: 680 g total minus both wheels
  double wheelMassKg = 0.043;                     // 2208 gimbal motor (about 39 g, listings) plus 10 % for the tyre
  double wheelInertiaKgM2 = 4.1e-6;               // estimate: rotor bell about 20 g at 12 mm, tyre about 4 g at 17 mm
  double comHeightM = 0.0572;                     // measured 2026-11-01: robot CoM 5 cm above the axle, body share
  double bodyPitchInertiaKgM2 = 3.39e-4;          // measured 2026-11-01: swing period 0.52 s hanging from the axle
  double bodyYawInertiaKgM2 = 1.5e-3;             // placeholder: body about the vertical axis through the axle centre
  double wheelRadiusM = 0.01915;                  // measured 2026-10-24
  double trackWidthM = 0.113;                     // measured 2026-10-26
  double bearingFrictionNmPerRadPerSec = 1.0e-5;  // fitted 2026-11-01 (no measurable effect above this)
  double yawFrictionNmPerRadPerSec = 0.04;       // fitted 2026-11-01: floor scrub when the robot turns, on the yaw rate
  double groundContactPitchRad = 1.2;             // the body touches the ground beyond this |pitch|
  double gravityMps2 = 9.81;
};

struct PlantState {
  double xM = 0.0;                    // axle position
  double vMps = 0.0;                  // axle speed
  double accelMps2 = 0.0;             // axle acceleration of the last step
  double pitchRad = 0.0;
  double pitchRateRadPerSec = 0.0;
  double pitchAccelRadPerSec2 = 0.0;  // of the last step
  double yawRad = 0.0;
  double yawRateRadPerSec = 0.0;
  double leftWheelRad = 0.0;          // wheel rotation in the world frame, forward roll positive
  double rightWheelRad = 0.0;
  double leftWheelRadPerSec = 0.0;
  double rightWheelRadPerSec = 0.0;
  bool lifted = false;                // held in the air: body fixed upright at x, wheels spin free
  bool fallen = false;                // latched once |pitch| reached groundContactPitchRad: everything stops
};

// Motor torques on the wheels, forward positive. The body takes the reaction torque.
struct WheelTorques {
  double leftNm = 0.0;
  double rightNm = 0.0;
};

// Advances the state by dtSec with one RK4 step of the equations in the plan.
void stepPlant(PlantState &state, const PlantParams &params, WheelTorques torques, double dtSec);

// A horizontal impulse at the body centre of mass, forward positive, in N s. Changes vMps, pitchRateRadPerSec and the
// wheel speeds at once. No effect while lifted or fallen.
void applyPush(PlantState &state, const PlantParams &params, double impulseNs);

// ----- Motor (motor.cpp)

// The real motor, as opposed to the values SimpleFOC is configured with.
struct MotorPhysics {
  double phaseResistanceOhm = 5.75;
  double kvRpmPerV = 105.0;         // fitted 2026-11-01 (low-speed KV of the 2026-10-22 sweeps); real back-EMF: Ke = 1 / (KV * sqrt(3) * RPM_TO_RADS) V per rad/s, on voltage.q
  double torquePerKe = 1.0;         // fitted 2026-11-01; torque = torquePerKe * Ke * current.q
};

struct DqVoltage {
  float d;
  float q;
};

// The SimpleFOC BLDCMotor fields that the sketch, ControlTick and signals.def.h use, with SimpleFOC's names, and the
// state of its two filters. One per wheel: motor1 left, motor2 right.
struct SimMotor {
  float phase_resistance = 0.0f;
  float KV_rating = 0.0f;
  float voltage_limit = 0.0f;
  float current_limit = 0.0f;
  float voltage_power_supply = static_cast<float>(kWheelSupplyVoltageV);
  bool enabled = false;
  float target = 0.0f;
  float current_sp = 0.0f;
  float shaft_angle = 0.0f;
  float shaft_velocity = 0.0f;
  DqVoltage voltage{0.0f, 0.0f};
  float currentQ = 0.0f;           // LPF_current_q output (SimpleFOC current.q)
  bool hasPreviousAngle = false;   // velocity estimate state
  double previousAngleRad = 0.0;

  float shaftAngle() const { return shaft_angle; }
  void enable() { enabled = true; }
  // As the sketch's setWheelEnabled(false): disabled, target and outputs zero.
  void disable();
};

// The sketch's configureWheelMotor(): phase resistance, KV_rating = KV * kWheelKvSafetyFactor, voltage_limit
// kWheelVoltageLimitV, current_limit kWheelVoltageLimitV / R.
void configureSimMotor(SimMotor &motor, const WheelMotorParams &params);

// One move() and loopFOC() of SimpleFOC 2.4.0 in torque mode with estimated_current (FOCMotor.cpp), dtSec after the
// previous call. sensorAngleRad is the motor-local shaft angle from the encoder. Updates shaft_angle, shaft_velocity,
// current_sp, currentQ and voltage.q. Does nothing to the outputs while disabled.
void focStep(SimMotor &motor, double sensorAngleRad, double dtSec);

// The q voltage the bridge puts on the motor: voltage.q * busV / voltage_power_supply, clamped to +-busV/sqrt(3);
// 0 while disabled.
double appliedVoltageQ(const SimMotor &motor, double busV);

struct MotorOutput {
  double currentA;  // motor-local
  double torqueNm;  // motor-local
};

// i = (appliedVq - Ke * w) / R and torque = torquePerKe * Ke * i, w the motor-local speed of the rotor relative to
// the stator.
MotorOutput motorOutput(const MotorPhysics &physics, double appliedVq, double relativeSpeedRadPerSec);

// ----- Sensors (sensors.cpp)

struct SensorParams {
  double imuHeightM = 0.03;                    // placeholder: IMU above the axle
  double gyroNoiseRadPerSec = 0.008;           // fitted 2026-11-01, standard deviation per sample
  double gyroBiasRadPerSec[3] = {0.0, 0.0, 0.0};
  double accelNoiseG = 0.008;                  // fitted 2026-11-01
  double imuPitchOffsetRad = 0.0192;           // fitted 2026-11-01: the robot stands at 1.1 deg measured forward lean
  uint32_t imuPeriodUs = 1000;                 // ODR 1 kHz
  int encoderCountsPerTurn = 4096;             // AS5600
  double batteryAdcNoiseCounts = 1.0;
  bool flipGyroX = false;                      // test switch: the tilt-rate axis reads inverted
};

// The ICM-42688 sample for the plant state, in counts at +-8 g and +-2000 deg/s, noise and bias included. Sensor axes:
// x right, y forward, z up, fixed to the body. Specific force at the IMU point (imuHeightM above the axle) and body
// rates; formulas in the plan.
ICM42688RawSample imuSample(const PlantState &state, const PlantParams &plant, const SensorParams &sensors,
                            std::mt19937 &rng);

// The AS5600 reading of a continuous angle: rounded to 2*pi/encoderCountsPerTurn.
double encoderAngle(double angleRad, const SensorParams &sensors);

// The ADC count for a bus voltage (board::kBatteryVoltsPerAdcCount), with noise, clamped to 0..4095.
uint16_t batteryAdc(double busV, const SensorParams &sensors, std::mt19937 &rng);

// ----- Arduino and driver functions on the simulation clock (hal.cpp)

// micros(), millis() and delay() read and advance nowUs. initICM42688() returns imuInitOk; readIMUDataIfReady() hands
// out imuSample once after imuReady was set, as the real driver does after a data-ready event.
struct Hal {
  uint32_t nowUs = 0;
  bool imuInitOk = true;
  bool imuReady = false;
  ICM42688RawSample imuSample{};
};
Hal &hal();

// ----- Log lines (sim_log.cpp)

// LinkPublishLog (link_log.h) is defined by the simulation: every Logging::message line lands in logLines() and, when
// set, in logHook (the live link).
struct LogLine {
  uint32_t tUs;
  Logging::Level level;
  std::string tag;
  std::string text;
};
std::vector<LogLine> &logLines();
extern void (*logHook)(Logging::Level level, const char *tag, const char *text);

// ----- Robot (robot.cpp)

struct BatteryParams {
  double openCircuitV = 7.8;         // placeholder: 2S at mid charge
  double internalResistanceOhm = 0.25;  // placeholder
  double otherLoadA = 0.3;           // placeholder: ESP32, servos holding, IMU
};

struct TimingParams {
  uint32_t passUs = 1770;            // measured 2026-11-01: median control tick 1.84 ms
  uint32_t passJitterUs = 200;       // uniform +-
  uint32_t physicsStepUs = 50;
};

// The receiver: raw S.BUS channels (index 0 is CH1), sent every framePeriodUs while framesOn.
struct RcInput {
  int16_t channels[ControlTick::kRcChannelCount];
  bool framesOn = true;
  bool signalOk = true;              // false: the receiver reports failsafe
  uint32_t framePeriodUs = 9000;
};
// All channels 992 (centre), CH5, CH6 and CH7 at 192 (off).
RcInput defaultRc();

struct Params {
  PlantParams plant;
  MotorPhysics leftMotor;
  MotorPhysics rightMotor;
  SensorParams sensors;
  BatteryParams battery;
  TimingParams timing;
  uint32_t seed = 1;
  bool flipMotorSign = false;        // test switch: both motors mounted reversed
};

// The truth of the last physics step, for the CSV and the tests.
struct Truth {
  double busV = 0.0;
  double leftCurrentA = 0.0;         // motor-local
  double rightCurrentA = 0.0;
};

// The sketch's setup() and loop() for the balance path, on the simulation clock.
class Robot {
 public:
  explicit Robot(const Params &params);

  // setup(): balance config, battery filter 50 Hz at 1000 Hz, legs at kLegSagDegrees, imu::begin(), both motors
  // configured and disabled. The clock starts at 0.
  void boot();

  // One loop() pass at the current time (FOC step, IMU and RC input, a control tick once kControlTimestepMinSec has
  // passed), then the physics up to the start of the next pass. Returns true when the pass ran a control tick.
  bool pass();

  uint32_t nowUs() const;
  PlantState &plant();
  RcInput &rc();
  BatteryParams &battery();
  const Truth &truth() const;
  const Params &params() const;
  void push(double impulseNs);
  void setLifted(bool lifted);

  // The live link (link_server.cpp): called at the start and the end of each control tick, as the sketch's
  // linkPassStart() and linkPassEnd().
  void (*linkPassStart)(uint32_t tickStartUs, bool balanceRequested) = nullptr;
  void (*linkPassEnd)(uint32_t tickStartUs) = nullptr;

 private:
  Params params_;
  PlantState plant_;
  RcInput rc_;
  Truth truth_;
  std::mt19937 rng_;
  uint32_t nextRcFrameUs_ = 0;
  uint32_t nextImuSampleUs_ = 0;
  uint32_t previousControlTimeUs_ = 0;
  uint32_t previousPassUs_ = 0;
  bool hasPreviousPass_ = false;
};

// ----- Scenario and batch runner (scenario.cpp, main.cpp)

// One timed line of a scenario file (format in the plan).
struct ScenarioStep {
  double tSec;
  std::string command;               // rc, rc_frames, rc_failsafe, push, lift, battery, end
  std::vector<std::string> args;
};

// Parses a scenario file. Returns false and sets error ("line N: ...") on a bad line.
bool parseScenario(const std::string &text, std::vector<ScenarioStep> &steps, std::string &error);

// Applies one step to the robot. Returns false and sets error for an unknown command or bad arguments.
bool applyStep(Robot &robot, const ScenarioStep &step, std::string &error);

// Sets one parameter by its dotted name (plant.comHeightM, battery.openCircuitV, timing.passUs, sensors.flipGyroX,
// flipMotorSign, seed, ...; the field names of the structs above). Returns false for an unknown name or bad value.
bool setParam(Params &params, const std::string &name, const std::string &value);

// ----- Live mode (link_server.cpp)

// The expansion of signals.def.h (sim_signals.cpp): the table the endpoint serves, and the health rows refreshed from
// the clock.
const SignalEntry *signalTableEntries();
size_t signalTableCount();
void updateSystemHealth(uint32_t nowUs);

// Runs the robot in real time and serves the NavBot link on one TCP client at listen ("HOST:PORT"). Steps of the
// scenario apply at their times. Returns the process exit code.
int runLive(Robot &robot, const std::vector<ScenarioStep> &steps, const std::string &listen);

}  // namespace sim

// The sketch's globals that signals.def.h names, defined by robot.cpp. They are global, as in the sketch, so the
// sim's expansion of signals.def.h (sim_signals.cpp) reads the same expressions.
extern ControlTick::State control;
extern sim::SimMotor motor1;
extern sim::SimMotor motor2;
extern Config balanceSettings;
extern float m1FilteredVelocityRadPerSec;
extern float m2FilteredVelocityRadPerSec;
extern float controlTimestepSec;
extern uint32_t controlStartUs;
extern uint32_t controlExecutionUs;
extern uint32_t loopEntryIntervalUs;
