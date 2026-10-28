#ifndef LINK_LOG_H
#define LINK_LOG_H

#include "LogSamples.h"

// Sends one Log frame on UART1, the only link port that carries them until migration step 5. Never blocks; a frame that
// does not fit the queue is dropped and counted.
// Call it from the control loop task only: each port's seq counter has one producer.
void LinkPublishLog(Logging::Level level, const char *tag, const char *text);

#endif
