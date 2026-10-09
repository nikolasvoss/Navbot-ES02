#pragma once
#include <cstdint>

constexpr int HSPI = 2;
constexpr int SPI_MODE3 = 3;
constexpr int SPI_CLOCK_DIV32 = 32;

class SPIClass {
 public:
  explicit SPIClass(int) {}
  void begin(int, int, int, int) {}
  void setDataMode(int) {}
  void setClockDivider(int) {}
  uint8_t transfer(uint8_t value);
};
