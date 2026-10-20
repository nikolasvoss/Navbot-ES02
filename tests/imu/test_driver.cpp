#include <Arduino.h>
#include <SPI.h>
#include "ICM42688.h"
#include "imu.h"

#include <array>
#include <cassert>
#include <cstdint>

HostSerial Serial;
namespace {
std::array<uint8_t, 128> registers{};
uint8_t address = 0;
bool reading = false;
bool commandSeen = false;
}

uint32_t micros() { return 0; }
void delay(unsigned long) {}
void pinMode(int, int) {}
void digitalWrite(int, int value) {
  if (value == LOW) commandSeen = false;
}

uint8_t SPIClass::transfer(uint8_t value) {
  if (!commandSeen) {
    commandSeen = true;
    reading = (value & 0x80U) != 0;
    address = value & 0x7fU;
    return 0;
  }
  const uint8_t current = address++;
  if (!reading) {
    registers[current] = value;
    return 0;
  }
  const uint8_t result = registers[current];
  if (current == UB0_REG_INT_STATUS) registers[current] = 0;
  return result;
}

int main() {
  registers[UB0_REG_WHO_AM_I] = WHO_AM_I;
  assert(!initICM42688(999));
  assert(initICM42688(imu::kSampleRateHz));
  assert((registers[UB0_REG_ACCEL_CONFIG0] & 0x0fU) == ODR_1KHZ);
  assert((registers[UB0_REG_GYRO_CONFIG0] & 0x0fU) == ODR_1KHZ);

  ICM42688RawSample sample{1, 2, 3, 4, 5, 6, 7};
  assert(!readIMUDataIfReady(sample));
  assert(sample.accelerationX == 1 && sample.temperature == 7);
  registers[UB0_REG_INT_STATUS] = 1U << 3;
  const int16_t values[7] = {132, 4096, -8192, 123, -234, 345, -456};
  for (int index = 0; index < 7; ++index) {
    registers[UB0_REG_TEMP_DATA1 + index * 2] = static_cast<uint16_t>(values[index]) >> 8;
    registers[UB0_REG_TEMP_DATA1 + index * 2 + 1] = static_cast<uint16_t>(values[index]) & 0xffU;
  }
  assert(readIMUDataIfReady(sample));
  assert(sample.temperature == 132);
  assert(sample.accelerationX == 4096 && sample.accelerationY == -8192 && sample.accelerationZ == 123);
  assert(sample.gyroscopeX == -234 && sample.gyroscopeY == 345 && sample.gyroscopeZ == -456);
  assert(!readIMUDataIfReady(sample));
  return 0;
}
