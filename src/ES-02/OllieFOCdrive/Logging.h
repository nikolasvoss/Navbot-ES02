#pragma once

#include <stdint.h>

// Firmware text for the NavBot link. Each message goes out as a Log frame on every port (link_uart.cpp).
namespace Logging {

enum class Level : uint8_t { Debug, Info, Warning, Error };

// message() drops a line below the minimum level before it formats the text. The default is Info.
void setMinLevel(Level level);
Level minLevel();

void message(Level level, const char *tag, const char *format, ...);

}  // namespace Logging
