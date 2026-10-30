// Expands the entry file named by SIGNAL_DEF_FILE into kSignals and kSignalCount. Include it once per translation
// unit, at namespace scope, after the state, the settings groups and the action handlers the entries name. The sketch
// names signals.def.h; tests name a fixture. A duplicate name or a name too long for the catalog stops the build here.
#include "signal_table.h"

#ifndef SIGNAL_DEF_FILE
#error "define SIGNAL_DEF_FILE to the entry file before including signal_table_expand.h"
#endif

#define SIGNAL(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Signal, name, unit, expr)
#define SIGNAL_MIN(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Min, name, unit, expr)
#define SIGNAL_MAX(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Max, name, unit, expr)

// A settings group is a type with Struct (the group's struct), current() (a copy of the running struct) and
// apply(candidate, error), which returns false with *error set when it refuses the candidate. A PARAM takes a copy,
// changes one field of it, and hands the copy to apply, so a refused value never reaches the running struct.
// PARAM_LIVE(name, unit, group, field, step) is a PARAM that a SetParam may change while balance is requested, by at
// most step in absolute value (step 0 refuses every change, INFINITY allows any change). PARAM has step 0.
#define PARAM_ROW(name, unit, group, field, step)                                                            \
  {#name, unit, SignalKind::Param, SignalTypeOf<decltype(group::current().field)>(),                         \
   +[]() -> uint32_t { return SignalBits(group::current().field); },                                         \
   +[](uint32_t bits, const char** error) -> bool {                                                          \
     auto candidate = group::current();                                                                      \
     *error = SignalFromBits(bits, candidate.field);                                                         \
     return *error == nullptr && group::apply(candidate, error);                                             \
   },                                                                                                        \
   nullptr, nullptr, static_cast<float>(step)},
#define PARAM(name, unit, group, field) PARAM_ROW(name, unit, group, field, 0.0f)
#define PARAM_LIVE(name, unit, group, field, step) PARAM_ROW(name, unit, group, field, step)

// An action takes its arguments as ARG(name, min, max) and is refused by its handler, which has the safety checks.
// ACTION_NO_ARGS is for an action without arguments: an empty list would not compile.
#define ARG(name, min, max) {#name, static_cast<float>(min), static_cast<float>(max)}
#define ACTION(name, handler, ...)                                                                               \
  {#name, "", SignalKind::Action, SignalType::None, nullptr, nullptr, handler,                                  \
   +[]() -> SignalArgs {                                                                                         \
     static constexpr SignalArg items[] = {__VA_ARGS__};                                                         \
     return {items, sizeof(items) / sizeof(items[0])};                                                           \
   }, 0.0f},
#define ACTION_NO_ARGS(name, handler) \
  {#name, "", SignalKind::Action, SignalType::None, nullptr, nullptr, handler, +[]() -> SignalArgs { return {nullptr, 0}; }, 0.0f},

constexpr SignalEntry kSignals[] = {
#include SIGNAL_DEF_FILE
};

#undef SIGNAL
#undef SIGNAL_MIN
#undef SIGNAL_MAX
#undef PARAM
#undef PARAM_LIVE
#undef PARAM_ROW
#undef ARG
#undef ACTION
#undef ACTION_NO_ARGS

constexpr size_t kSignalCount = sizeof(kSignals) / sizeof(kSignals[0]);

static_assert(SignalNamesUnique(kSignals, kSignalCount), "duplicate signal name in the entry file");
static_assert(SignalLiveStepsValid(kSignals, kSignalCount),
              "a live step is negative or set on an entry that is not a parameter");
static_assert(SignalStringsFit(kSignals, kSignalCount),
              "a signal name is empty or longer than 24 characters, or a unit is longer than 16");
