#pragma once

#include <stdint.h>
#include <string>
#include <vector>

namespace Recording {

struct Event {
  std::string kind;  // "K12", "Trace", "Control", "Balance", "Drive", "due", "ranges"
  uint32_t sequence;  // Trace-family sequence number, otherwise 0
  std::string text;   // Hex dump of the submitted sample, or the call arguments
};

extern bool dueResult;
void reset();
const std::vector<Event>& events();

}  // namespace Recording
