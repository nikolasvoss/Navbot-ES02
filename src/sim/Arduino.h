#pragma once

// The Arduino functions the shared firmware code calls, for the host simulation. hal.cpp defines them on the
// simulation clock.

#include <cmath>
#include <cstdint>
#include <math.h>

uint32_t micros();
uint32_t millis();
void delay(unsigned long milliseconds);
