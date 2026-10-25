#ifndef LINK_UART_H
#define LINK_UART_H

#include <Arduino.h>
#include "BoardConfig.h"
#include "imu.h"

// UART1 connects the ESP32 to the ros2_pc (NavBot link, see
// docs/robot/software/interfaces/ros2-uart-protocol.md).
// Firmware pin numbers: RX = GPIO20, TX = GPIO19 (opposite of the PCB net names RX1/TX1).
#define LINK_UART_BAUD 500000

// 1 kHz / 20 = 50 Hz
#define LINK_ODOMETRY_DIVIDER 20

void LinkUartInit();

// Sends WheelOdometry, then Imu if the snapshot is valid. Never blocks; a frame that does not fit is dropped.
void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot);

#endif
