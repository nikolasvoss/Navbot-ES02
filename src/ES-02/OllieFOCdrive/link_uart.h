#ifndef LINK_UART_H
#define LINK_UART_H

#include <Arduino.h>
#include "BoardConfig.h"
#include "imu.h"
#include "link_schedule.h"

// UART1 connects the ESP32 to the ros2_pc (NavBot link, see
// docs/integration/software/ros2-uart-protocol.md).
// Firmware pin numbers: RX = GPIO20, TX = GPIO19 (opposite of the PCB net names RX1/TX1).
#define LINK_UART_BAUD 2000000

// WheelOdometry/Imu send period: 50 Hz on average, checked once per control tick (see link_schedule.h).
#define LINK_ODOMETRY_PERIOD_US 20000

void LinkUartInit();

// Starts the USB TX queue and its writer task. Call right after Serial.begin() and before anything writes to the USB port.
void LinkUsbInit();

// Print for text that the control loop or setup produces for the USB port. Never blocks; text that does not fit the
// queue is dropped and counted.
Print& LinkUsbPrint();

// Queues WheelOdometry, then Imu if the snapshot is valid, for UART1. Never blocks and never touches the UART; a frame
// that does not fit is dropped and counted.
void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot);

#endif
