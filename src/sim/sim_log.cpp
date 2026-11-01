// The firmware's log lines in the simulation: every Logging::message lands in logLines() and, when set, in logHook.

#include "link_log.h"
#include "sim.h"

namespace sim {

void (*logHook)(Logging::Level level, const char *tag, const char *text) = nullptr;

std::vector<LogLine> &logLines() {
  static std::vector<LogLine> lines;
  return lines;
}

}  // namespace sim

// link_log.h: the firmware's link_uart.cpp is replaced by this function.
void LinkPublishLog(Logging::Level level, const char *tag, const char *text) {
  sim::logLines().push_back({sim::hal().nowUs, level, tag, text});
  if (sim::logHook) sim::logHook(level, tag, text);
}
