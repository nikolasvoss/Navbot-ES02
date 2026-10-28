#ifndef FIXTURE_STATE_H
#define FIXTURE_STATE_H

#include <stdint.h>

// Stand-in for the sketch state that fixture_signals.def.h reads.
namespace fixture {

enum class Mode : uint8_t { Idle, Run, Fault = 7 };
struct Vector {
  float x;
};

inline float speed = 0.0f;
inline uint32_t count = 0;
inline int32_t offset = 0;
inline bool flag = false;
inline Mode mode = Mode::Idle;
inline double battery = 0.0;
inline uint32_t loopUs = 0;
inline uint16_t adc = 0;
inline uint64_t wide = 0;
inline Vector& latest() {
  static Vector v{0.0f};
  return v;
}

}  // namespace fixture

#endif
