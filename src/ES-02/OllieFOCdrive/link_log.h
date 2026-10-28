#ifndef LINK_LOG_H
#define LINK_LOG_H

#include "Logging.h"

// Sends one Log frame on each port, UART1 and USB. Never blocks; a frame that does not fit a port's queue is dropped and
// counted.
// Call it from the control loop task only: each port's seq counter has one producer.
void LinkPublishLog(Logging::Level level, const char *tag, const char *text);

#endif
