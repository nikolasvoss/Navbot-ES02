// The Arduino and IMU driver functions of the shared firmware code, on the simulation clock (sim.h, Hal).

#include <Arduino.h>

#include "ICM42688.h"
#include "sim.h"

namespace sim {

Hal &hal() {
  static Hal instance;
  return instance;
}

}  // namespace sim

uint32_t micros() { return sim::hal().nowUs; }

uint32_t millis() { return sim::hal().nowUs / 1000U; }

void delay(unsigned long milliseconds) { sim::hal().nowUs += static_cast<uint32_t>(milliseconds * 1000UL); }

bool initICM42688(uint16_t) { return sim::hal().imuInitOk; }

bool readIMUDataIfReady(ICM42688RawSample &sample) {
  sim::Hal &clock = sim::hal();
  if (!clock.imuReady) {
    return false;
  }
  clock.imuReady = false;
  sample = clock.imuSample;
  return true;
}
