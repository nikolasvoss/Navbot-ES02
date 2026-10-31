// Must not compile: run_host_test.sh expands a bad entry file through this file and expects the build to stop.
// SIGNAL_DEF_FILE comes from the command line.
#include "fixture_state.h"
#include "signal_table_expand.h"
