#pragma once

namespace board {

constexpr int kStatusLedPin = 35;

constexpr int kBatteryAdcPin = 17;
// Double on purpose: the float product `7.77f / 813.43f` changes the ADC-to-volts result by one ulp.
constexpr double kBatteryVoltsPerAdcCount = 7.77f / 813.43;
// Double on purpose: comparing a float voltage against 7.4f would flip the result at exactly 7.4f.
constexpr double kLowBatteryVolts = 7.4;

constexpr int kLegServoPin[4] = {11, 12, 21, 14};

constexpr int kWheel1PhaseAPin = 15;
constexpr int kWheel1PhaseBPin = 7;
constexpr int kWheel1PhaseCPin = 6;
constexpr int kWheel1EnablePin = 16;
constexpr int kWheel2PhaseAPin = 40;
constexpr int kWheel2PhaseBPin = 39;
constexpr int kWheel2PhaseCPin = 38;
constexpr int kWheel2EnablePin = 37;
constexpr int kWheelMotorPolePairs = 7;

constexpr int kI2cOneSdaPin = 4;
constexpr int kI2cOneSclPin = 5;
constexpr int kI2cTwoSdaPin = 41;
constexpr int kI2cTwoSclPin = 42;

constexpr int kImuSpiSckPin = 3;
constexpr int kImuSpiMosiPin = 9;
constexpr int kImuSpiMisoPin = 10;
constexpr int kImuSpiCsPin = 13;

constexpr int kTouchUartRxPin = 20;
constexpr int kTouchUartTxPin = 19;

constexpr int kSbusRxPin = 1;
constexpr int kSbusTxPin = 2;

}  // namespace board
