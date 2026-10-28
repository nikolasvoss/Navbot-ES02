#include "link_log.h"

#include <string.h>

navbot_link_v1_Log_Level LinkLogLevel(Logging::Level level) {
  switch (level) {
    case Logging::Level::Debug:
      return navbot_link_v1_Log_Level_DEBUG;
    case Logging::Level::Info:
      return navbot_link_v1_Log_Level_INFO;
    case Logging::Level::Warning:
      return navbot_link_v1_Log_Level_WARN;
    case Logging::Level::Error:
      return navbot_link_v1_Log_Level_ERROR;
  }
  return navbot_link_v1_Log_Level_INFO;
}

// init_zero leaves the arrays zeroed, so cutting to size - 1 always leaves the string terminated.
void LinkSendLog(navbot_link_v1_Log_Level level, const char *tag, const char *text, LinkMessageSink &sink) {
  navbot_link_v1_Message message = navbot_link_v1_Message_init_zero;
  message.which_body = navbot_link_v1_Message_log_tag;
  message.body.log.level = level;
  strncpy(message.body.log.tag, tag, sizeof(message.body.log.tag) - 1);
  strncpy(message.body.log.text, text, sizeof(message.body.log.text) - 1);
  for (uint8_t port = 0; port < kLinkPortCount; ++port) sink.Send(port, message);
}
