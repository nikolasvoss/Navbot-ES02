// Logging::message formats one line and hands it to LinkPublishLog. The test stands in for the firmware's
// LinkPublishLog (link_uart.cpp) and records what arrives.

#include "Logging.h"

#include <cstdio>
#include <string>
#include <vector>

namespace {

struct Line {
  Logging::Level level;
  std::string tag;
  std::string text;
};

std::vector<Line> lines;

bool Check(const char* name, bool ok) {
  std::printf("%s: %s\n", name, ok ? "ok" : "FAIL");
  return ok;
}

}  // namespace

void LinkPublishLog(Logging::Level level, const char* tag, const char* text) { lines.push_back({level, tag, text}); }

namespace {

bool TestFormatsTagLevelAndText() {
  lines.clear();
  Logging::message(Logging::Level::Warning, "WHEEL", "peak=%.1frad_s limit=%d", 38.5, 60);
  return Check("format keeps level, tag and text", lines.size() == 1 && lines[0].level == Logging::Level::Warning &&
                                                       lines[0].tag == "WHEEL" && lines[0].text == "peak=38.5rad_s limit=60");
}

bool TestTextOf96CharactersIsKeptWhole() {
  lines.clear();
  const std::string text(96, 'a');
  Logging::message(Logging::Level::Info, "EDGE", "%s", text.c_str());
  return Check("96 characters pass uncut", lines.size() == 1 && lines[0].text == text);
}

bool TestLongTextIsCutTo96Characters() {
  lines.clear();
  // Each character is distinct, so a cut at the wrong place shows.
  std::string text;
  for (int i = 0; i < 200; ++i) text.push_back(static_cast<char>('a' + i % 26));
  Logging::message(Logging::Level::Error, "LONG", "%s", text.c_str());
  return Check("text longer than 96 characters is cut to the first 96",
               lines.size() == 1 && lines[0].text == text.substr(0, 96));
}

}  // namespace

int main() {
  bool ok = true;
  ok &= TestFormatsTagLevelAndText();
  ok &= TestTextOf96CharactersIsKeptWhole();
  ok &= TestLongTextIsCutTo96Characters();
  return ok ? 0 : 1;
}
