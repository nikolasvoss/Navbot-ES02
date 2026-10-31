#ifndef LINK_LOG_H
#define LINK_LOG_H

#include "Logging.h"
#include "link_endpoint.h"

// Log frames. LinkSendLog is pure (no Arduino or FreeRTOS, so tests/link host-tests it): it sends one Log frame on each
// port of sink. The endpoint's replies and the firmware's messages both use it.
navbot_link_v1_Log_Level LinkLogLevel(Logging::Level level);
void LinkSendLog(navbot_link_v1_Log_Level level, const char *tag, const char *text, LinkMessageSink &sink);

// Firmware (link_uart.cpp): one Log frame on UART1 and one on USB, for Logging::message. Never blocks; a frame that does
// not fit a port's queue is dropped and counted.
// Call it from the control loop task only: each port's seq counter has one producer.
void LinkPublishLog(Logging::Level level, const char *tag, const char *text);

#endif
