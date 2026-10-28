#include "Logging.h"

#include <stdarg.h>
#include <stdio.h>

#include "link_log.h"
#include "src/navbot_link/navbot_link.pb.h"

namespace {
// The Log text field of navbot_link.options (max_size 97): 96 characters and the terminator. Longer text is cut.
constexpr size_t kTextBytes = sizeof(navbot_link_v1_Log::text);
}  // namespace

namespace Logging {

void message(Level level, const char *tag, const char *format, ...) {
  char text[kTextBytes];
  va_list args;
  va_start(args, format);
  vsnprintf(text, sizeof(text), format, args);
  va_end(args);
  LinkPublishLog(level, tag, text);
}

}  // namespace Logging
