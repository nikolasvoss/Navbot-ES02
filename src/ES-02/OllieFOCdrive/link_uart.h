#ifndef LINK_UART_H
#define LINK_UART_H

#include <Arduino.h>
#include "BoardConfig.h"
#include "imu.h"
#include "link_schedule.h"
#include "signal_table.h"

// UART1 connects the ESP32 to the ros2_pc (NavBot link, see
// docs/integration/software/ros2-uart-protocol.md).
// Firmware pin numbers: RX = GPIO20, TX = GPIO19 (opposite of the PCB net names RX1/TX1).
#define LINK_UART_BAUD 2000000
// The USB port (UART0 and the CH340).
#define LINK_USB_BAUD 576000

// WheelOdometry/Imu send period: 50 Hz on average, checked once per control tick (see link_schedule.h).
#define LINK_ODOMETRY_PERIOD_US 20000
// Measured 4.96 kB/s for WheelOdometry and Imu at 50 Hz each; the subscription budget on UART1 starts after it.
#define LINK_ODOMETRY_RESERVED_BYTES_PER_SEC 5000

// Opens UART1, starts its writer and the receive tasks of both ports, and creates the endpoint that answers the catalog,
// subscription and parameter requests over `table` (build is the firmware build ID, up to 32 characters).
void LinkUartInit(const SignalEntry *table, size_t count, const char *build);

// Control loop, at the start of a tick: applies the requests that the receive tasks queued since the last tick.
// balanceRequested is the sketch's balance state at that moment (see LinkEndpoint::Handle).
void LinkServiceRequests(uint32_t tickStartUs, bool balanceRequested);

// Control loop, at the end of a tick: sends the rows that are due, and drops lapsed leases.
void LinkSampleSignals(uint32_t tickStartUs);

// The letter commands (K, T, M, ...) that arrive on the USB port as printable lines, for the Commander. A frame never
// shows up here.
int LinkUsbCommandAvailable();
int LinkUsbCommandRead();
int LinkUsbCommandPeek();

// Starts the USB TX queue and its writer task. Call right after Serial.begin() and before anything writes to the USB port.
void LinkUsbInit();

// Print for text that the control loop or setup produces for the USB port. Never blocks; text that does not fit the
// queue is dropped and counted.
Print& LinkUsbPrint();

// Queues WheelOdometry, then Imu if the snapshot is valid, for UART1. Never blocks and never touches the UART; a frame
// that does not fit is dropped and counted.
void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot);

#endif
