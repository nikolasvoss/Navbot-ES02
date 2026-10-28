#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string>

class HostSerial {
 public:
  size_t write(const uint8_t *data, size_t length);
};

extern HostSerial Serial;
uint32_t micros();
void setMicrosForTest(uint32_t value);
void limitSerialBaud(uint32_t baud);
void blockSerialWrites();
bool waitForSerialWriteBlocked();
void releaseSerialWrites();
bool waitForSerialLines(size_t count);
void shortNextSerialWrite();
std::string serialOutput();

// Log frames the code under test handed to LinkPublishLog, as "level tag text".
std::string publishedLogs();
