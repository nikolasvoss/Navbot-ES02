// Expands the entry file named by SIGNAL_DEF_FILE into kSignals and kSignalCount. Include it once per translation
// unit, at namespace scope, after the state the entry expressions read. The sketch names signals.def.h; tests name a
// fixture. A duplicate name or a name too long for the catalog stops the build here.
#include "signal_table.h"

#ifndef SIGNAL_DEF_FILE
#error "define SIGNAL_DEF_FILE to the entry file before including signal_table_expand.h"
#endif

#define SIGNAL(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Signal, name, unit, expr)
#define SIGNAL_MIN(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Min, name, unit, expr)
#define SIGNAL_MAX(name, unit, expr) SIGNAL_TABLE_ROW(SignalKind::Max, name, unit, expr)

constexpr SignalEntry kSignals[] = {
#include SIGNAL_DEF_FILE
};

#undef SIGNAL
#undef SIGNAL_MIN
#undef SIGNAL_MAX

constexpr size_t kSignalCount = sizeof(kSignals) / sizeof(kSignals[0]);

static_assert(SignalNamesUnique(kSignals, kSignalCount), "duplicate signal name in the entry file");
static_assert(SignalStringsFit(kSignals, kSignalCount),
              "a signal name is empty or longer than 24 characters, or a unit is longer than 16");
