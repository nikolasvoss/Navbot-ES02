#include "Logging.h"

#include <stdarg.h>
#include <stdio.h>

#include "link_log.h"

namespace {
// The Log frame's text field holds 96 bytes; longer text is cut to fit.
constexpr size_t kTextBytes = 96;
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
