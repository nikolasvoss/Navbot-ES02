#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

class HostSerial {
 public:
  size_t write(const uint8_t *data, size_t length);
};

extern HostSerial Serial;
void blockSerialWrites();
bool waitForSerialWriteBlocked();
void releaseSerialWrites();
bool waitForSerialLines(size_t count);
std::string serialOutput();
