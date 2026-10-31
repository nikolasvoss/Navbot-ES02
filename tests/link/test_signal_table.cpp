#include <cstdio>
#include <cstring>

#include "fixture_state.h"
#define SIGNAL_DEF_FILE "fixture_signals.def.h"
#include "signal_table_expand.h"

namespace {

bool failed = false;

void Check(bool condition, const char* what) {
  std::printf("%s: %s\n", what, condition ? "ok" : "FAIL");
  if (!condition) failed = true;
}

const SignalEntry* Find(const char* name) {
  for (size_t i = 0; i < kSignalCount; ++i)
    if (std::strcmp(kSignals[i].name, name) == 0) return &kSignals[i];
  return nullptr;
}

uint32_t Bits(const char* name) { return Find(name)->read(); }

}  // namespace

int main() {
  Check(kSignalCount == 10, "the fixture file expands to 10 rows");

  struct Expected {
    const char* name;
    const char* unit;
    SignalKind kind;
    SignalType type;
  };
  const Expected expected[] = {
      {"speed", "m/s", SignalKind::Signal, SignalType::F32},
      {"count", "", SignalKind::Signal, SignalType::U32},
      {"offset", "", SignalKind::Signal, SignalType::I32},
      {"flag", "", SignalKind::Signal, SignalType::Bool},
      {"mode", "", SignalKind::Signal, SignalType::U32},
      {"battery_min", "V", SignalKind::Min, SignalType::F32},
      {"loop_max", "us", SignalKind::Max, SignalType::U32},
      {"raw_adc", "", SignalKind::Signal, SignalType::U32},
      {"latest_x", "g", SignalKind::Signal, SignalType::F32},
      {"tuning_gain", "", SignalKind::Param, SignalType::F32},
  };
  bool rowsMatch = true;
  for (size_t i = 0; i < kSignalCount; ++i) {
    const SignalEntry& e = kSignals[i];
    rowsMatch &= std::strcmp(e.name, expected[i].name) == 0 && std::strcmp(e.unit, expected[i].unit) == 0 &&
                 e.kind == expected[i].kind && e.type == expected[i].type && e.read != nullptr &&
                 e.args == nullptr;
  }
  Check(rowsMatch, "name, unit, kind and type of every row, in file order");

  fixture::speed = 1.5f;
  fixture::count = 4000000000u;  // above 2^24: a float would round it
  fixture::offset = -3;
  fixture::flag = true;
  fixture::mode = fixture::Mode::Fault;
  fixture::battery = 7.5;  // a double narrows to float
  fixture::loopUs = 1234;
  fixture::adc = 65535;
  fixture::latest().x = -0.25f;

  Check(Bits("speed") == 0x3FC00000u, "float 1.5 is IEEE 754 single");
  Check(Bits("count") == 4000000000u, "u32 keeps every bit");
  Check(Bits("offset") == 0xFFFFFFFDu, "i32 -3 is two's complement");
  Check(Bits("flag") == 1u, "bool true is 1");
  Check(Bits("mode") == 7u, "an enum is its number");
  Check(Bits("battery_min") == 0x40F00000u, "a double is narrowed to float");
  Check(Bits("loop_max") == 1234u, "the running-value kinds read the current value too");
  Check(Bits("raw_adc") == 65535u, "a 16-bit unsigned is widened");
  Check(Bits("latest_x") == 0xBE800000u, "an accessor result is read on each call");
  fixture::latest().x = 2.0f;
  Check(Bits("latest_x") == 0x40000000u, "the next call sees the new state");
  fixture::flag = false;
  Check(Bits("flag") == 0u, "bool false is 0");
  Check(kSignals[9].liveStep == 0.5f && kSignals[0].liveStep == 0.0f, "PARAM_LIVE keeps its step, a SIGNAL has none");
  Check(SignalLiveStepsValid(kSignals, kSignalCount), "every live step is a number that is not negative");

  const SignalEntry same[] = {{"a", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f},
                              {"b", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  const SignalEntry dup[] = {{"a", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f},
                             {"b", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f},
                             {"a", "", SignalKind::Min, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  Check(SignalNamesUnique(same, 2) && !SignalNamesUnique(dup, 3), "a repeated name is detected, whatever the kind");

  const SignalEntry longName[] = {{"abcdefghijklmnopqrstuvwxy", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  const SignalEntry longNameFits[] = {{"abcdefghijklmnopqrstuvwx", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  const SignalEntry longUnit[] = {{"a", "12345678901234567", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  const SignalEntry noName[] = {{"", "", SignalKind::Signal, SignalType::F32, nullptr, nullptr, nullptr, nullptr, 0.0f}};
  Check(SignalStringsFit(longNameFits, 1) && !SignalStringsFit(longName, 1) && !SignalStringsFit(longUnit, 1) &&
            !SignalStringsFit(noName, 1),
        "names up to 24 and units up to 16 characters fit the catalog");

  // A SetParam value is parsed back into its type, and anything the type cannot hold is refused, not wrapped.
  bool flag = false;
  uint16_t cutoff = 0;
  float gain = 0.0f;
  int32_t offset = 0;
  Check(SignalFromBits(1u, flag) == nullptr && flag, "a bool 1 is true");
  Check(SignalFromBits(2u, flag) != nullptr && flag, "a bool 2 is refused and leaves the value");
  Check(SignalFromBits(0x3FC00000u, gain) == nullptr && gain == 1.5f, "a float is read from its IEEE 754 bits");
  Check(SignalFromBits(0x7F800000u, gain) != nullptr && gain == 1.5f, "an infinite float is refused");
  Check(SignalFromBits(0x7FC00000u, gain) != nullptr, "a NaN float is refused");
  Check(SignalFromBits(25u, cutoff) == nullptr && cutoff == 25, "a uint16 that fits is stored");
  Check(SignalFromBits(65536u, cutoff) != nullptr && cutoff == 25, "a uint16 of 65536 is refused, not wrapped to 0");
  Check(SignalFromBits(0xFFFFFFFDu, offset) == nullptr && offset == -3, "an int32 takes two's complement bits");

  return failed ? 1 : 0;
}
