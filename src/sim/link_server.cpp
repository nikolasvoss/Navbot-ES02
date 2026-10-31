// Live mode is not built yet (T5 replaces this stub with the TCP link).

#include <cstdio>

#include "sim.h"

namespace sim {

int runLive(Robot &, const std::vector<ScenarioStep> &, const std::string &) {
  std::fprintf(stderr, "live mode not built\n");
  return 2;
}

}  // namespace sim
