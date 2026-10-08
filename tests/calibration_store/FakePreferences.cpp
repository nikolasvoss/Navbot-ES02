#include <Preferences.h>

std::map<std::string, float> Preferences::values;
std::set<std::string> Preferences::failedReads;
std::set<std::string> Preferences::failedWrites;
bool Preferences::failBegin = false;
int Preferences::beginCount = 0;
int Preferences::endCount = 0;
std::vector<std::string> Preferences::namespaces;
std::vector<std::string> Preferences::readKeys;
std::vector<std::string> Preferences::writeKeys;

bool Preferences::begin(const char* name, bool) {
  ++beginCount;
  namespaces.emplace_back(name);
  open_ = !failBegin;
  return open_;
}

void Preferences::end() {
  ++endCount;
  open_ = false;
}

float Preferences::getFloat(const char* key, float defaultValue) {
  readKeys.emplace_back(key);
  if (!open_ || failedReads.count(key) != 0) {
    return defaultValue;
  }
  const auto found = values.find(key);
  return found == values.end() ? defaultValue : found->second;
}

size_t Preferences::putFloat(const char* key, float value) {
  writeKeys.emplace_back(key);
  if (!open_ || failedWrites.count(key) != 0) {
    return 0;
  }
  values[key] = value;
  return sizeof(float);
}

void Preferences::reset() {
  values.clear();
  failedReads.clear();
  failedWrites.clear();
  failBegin = false;
  beginCount = 0;
  endCount = 0;
  namespaces.clear();
  readKeys.clear();
  writeKeys.clear();
}
