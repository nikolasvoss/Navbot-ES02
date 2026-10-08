#pragma once

#include <map>
#include <set>
#include <string>
#include <vector>

class Preferences {
public:
  bool begin(const char* name, bool readOnly);
  void end();
  float getFloat(const char* key, float defaultValue);
  size_t putFloat(const char* key, float value);

  static void reset();

  static std::map<std::string, float> values;
  static std::set<std::string> failedReads;
  static std::set<std::string> failedWrites;
  static bool failBegin;
  static int beginCount;
  static int endCount;
  static std::vector<std::string> namespaces;
  static std::vector<std::string> readKeys;
  static std::vector<std::string> writeKeys;

private:
  bool open_ = false;
};
