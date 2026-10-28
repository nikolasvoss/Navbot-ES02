#pragma once

#include <stdint.h>

// Firmware text for the NavBot link. Each message goes out as a Log frame on every port (link_uart.cpp).
namespace Logging {

enum class Level : uint8_t { Debug, Info, Warning, Error };

void message(Level level, const char *tag, const char *format, ...);

}  // namespace Logging
