#ifndef SIGNAL_TABLE_H
#define SIGNAL_TABLE_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <cmath>
#include <type_traits>

// The entry table behind the catalog and the sampler. The sketch expands signals.def.h into a constexpr array of
// SignalEntry (OllieFOCdrive.ino); tests expand a fixture the same way. No Arduino dependency.

// The numbers are those of navbot_link.proto CatalogEntry.Kind and CatalogEntry.Type (link_endpoint.cpp checks them).
enum class SignalKind : uint8_t { Signal = 1, Min = 2, Max = 3, Param = 4, Action = 5 };
enum class SignalType : uint8_t { None = 0, F32 = 1, I32 = 2, U32 = 3, Bool = 4 };

// Limits of the catalog strings (navbot_link.options: 24 characters for a name, 16 for a unit or an argument name).
constexpr size_t kSignalMaxNameLen = 24;
constexpr size_t kSignalMaxUnitLen = 16;

struct SignalArg {
  const char* name;
  float min;
  float max;
};

// The argument list of an action. Built by a function, so each list has static storage without a named array.
struct SignalArgs {
  const SignalArg* items;
  uint8_t count;
};

// One row of the table. Which function fields a row sets depends on its kind:
//   SIGNAL, SIGNAL_MIN, SIGNAL_MAX: read.
//   PARAM: read, and write, which applies a value in 4 wire bytes or sets *error and returns false.
//   ACTION: call, which starts the action or sets *error and returns false, and args.
struct SignalEntry {
  const char* name;
  const char* unit;
  SignalKind kind;
  SignalType type;  // None for an action
  uint32_t (*read)();
  bool (*write)(uint32_t bits, const char** error);
  bool (*call)(const float* args, const char** error);
  SignalArgs (*args)();
};

template <class T>
constexpr SignalType SignalTypeOf() {
  using V = std::remove_cv_t<std::remove_reference_t<T>>;
  if constexpr (std::is_same_v<V, bool>) {
    return SignalType::Bool;
  } else if constexpr (std::is_floating_point_v<V>) {
    return SignalType::F32;
  } else if constexpr (std::is_enum_v<V>) {
    return SignalTypeOf<std::underlying_type_t<V>>();
  } else if constexpr (std::is_integral_v<V>) {
    static_assert(sizeof(V) <= 4, "a signal is at most 32 bits wide");
    return std::is_signed_v<V> ? SignalType::I32 : SignalType::U32;
  } else {
    static_assert(sizeof(V) == 0, "a signal is a bool, an integer, an enum or a float");
    return SignalType::None;
  }
}

// The 4 wire bytes of a value, as the host reads them with the catalog type: a float as IEEE 754 single, an integer
// widened to 32 bits, a bool as 0 or 1. Integers are never sent as float, which would lose precision above 2^24.
template <class T>
inline uint32_t SignalBits(T value) {
  constexpr SignalType type = SignalTypeOf<T>();
  if constexpr (type == SignalType::F32) {
    const float single = static_cast<float>(value);
    uint32_t bits;
    memcpy(&bits, &single, sizeof(bits));
    return bits;
  } else if constexpr (type == SignalType::Bool) {
    return value ? 1u : 0u;
  } else {
    return static_cast<uint32_t>(static_cast<std::conditional_t<type == SignalType::I32, int32_t, uint32_t>>(value));
  }
}

// The inverse of SignalBits for a parameter: stores the value the 4 wire bytes carry in out, or returns the reason
// they are not a value of its type. An integer must be exact in its type, so a SetParam cannot wrap a value.
template <class T>
const char* SignalFromBits(uint32_t bits, T& out) {
  using V = std::remove_cv_t<T>;
  if constexpr (std::is_same_v<V, bool>) {
    if (bits > 1) return "a bool takes 0 or 1";
    out = bits == 1;
  } else if constexpr (std::is_floating_point_v<V>) {
    float single;
    memcpy(&single, &bits, sizeof(single));
    if (!std::isfinite(single)) return "the value is not a finite number";
    out = static_cast<V>(single);
  } else {
    static_assert(std::is_integral_v<V> && sizeof(V) <= 4, "a parameter is a bool, a float or an integer up to 32 bits");
    const V value = static_cast<V>(bits);
    if (static_cast<uint32_t>(value) != bits) return "the value is out of range";
    out = value;
  }
  return nullptr;
}

// One table row for a SIGNAL, SIGNAL_MIN or SIGNAL_MAX line. The lambda reads the state each time it is called, so an
// expression such as imu::latest().x needs no pointer that could go stale.
#define SIGNAL_TABLE_ROW(kind, name, unit, expr) \
  {#name, unit, kind, SignalTypeOf<decltype(expr)>(), +[]() -> uint32_t { return SignalBits(expr); }, nullptr, nullptr, nullptr},

constexpr size_t SignalStringLength(const char* text) {
  size_t length = 0;
  while (text[length] != '\0') ++length;
  return length;
}

constexpr bool SignalStringsEqual(const char* a, const char* b) {
  while (*a != '\0' && *a == *b) {
    ++a;
    ++b;
  }
  return *a == *b;
}

constexpr bool SignalNamesUnique(const SignalEntry* table, size_t count) {
  for (size_t i = 0; i < count; ++i)
    for (size_t j = i + 1; j < count; ++j)
      if (SignalStringsEqual(table[i].name, table[j].name)) return false;
  return true;
}

constexpr bool SignalStringsFit(const SignalEntry* table, size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const size_t nameLength = SignalStringLength(table[i].name);
    if (nameLength == 0 || nameLength > kSignalMaxNameLen) return false;
    if (SignalStringLength(table[i].unit) > kSignalMaxUnitLen) return false;
  }
  return true;
}

#endif
