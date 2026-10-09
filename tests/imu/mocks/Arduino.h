#pragma once
#include <cstdint>
#include <cmath>
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int OUTPUT = 1;

class HostSerial {
 public:
  void println(const char*) {}
  void print(const char*) {}
  void print(int) {}
  template <typename... Args> void printf(const char*, Args...) {}
};
extern HostSerial Serial;
uint32_t micros();
void delay(unsigned long milliseconds);
void pinMode(int pin, int mode);
void digitalWrite(int pin, int value);
