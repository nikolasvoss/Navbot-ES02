#include <cassert>
#include <cstdint>

#include "../../src/ES-02/OllieFOCdrive/rc_mode.h"

namespace {

// The sketch's CH5 handling for one tick: a move into position 1 sets the link flag from the controller state, a move
// out of it clears the flag, and the enable is what the balance controller receives.
struct Sketch {
  uint8_t position = 0;
  bool linkBalance = false;

  bool Tick(int16_t raw, bool controllerDriving) {
    const uint8_t next = RcMode::Ch5Position(raw);
    if (next == 1 && position != 1) linkBalance = RcMode::LinkBalanceOnEntry(controllerDriving);
    if (next != 1 && position == 1) linkBalance = false;
    position = next;
    return RcMode::EnableSwitch(position, linkBalance);
  }
};

constexpr int16_t kPos0 = 100;
constexpr int16_t kPos1 = 1000;
constexpr int16_t kPos2 = 1811;

}  // namespace

int main() {
  using namespace RcMode;

  // CH5 thresholds: 591 and 592 split 0 and 1; 1401 and 1402 split 1 and 2.
  assert(Ch5Position(172) == 0);
  assert(Ch5Position(591) == 0);
  assert(Ch5Position(592) == 1);
  assert(Ch5Position(992) == 1);
  assert(Ch5Position(1401) == 1);
  assert(Ch5Position(1402) == 2);
  assert(Ch5Position(1811) == 2);

  // Balance enable: position 2 always, position 1 only with the link flag, position 0 never.
  assert(!EnableSwitch(0, false));
  assert(!EnableSwitch(0, true));
  assert(!EnableSwitch(1, false));
  assert(EnableSwitch(1, true));
  assert(EnableSwitch(2, false));
  assert(EnableSwitch(2, true));

  // Entering position 1 keeps the link flag only while the controller drives.
  assert(LinkBalanceOnEntry(true));
  assert(!LinkBalanceOnEntry(false));

  // Command freshness: not seen, fresh, exactly at the timeout, one microsecond over.
  assert(!CommandFresh(false, 1000, 1000));
  assert(CommandFresh(true, 1000, 1000));
  assert(CommandFresh(true, 1000 + kLinkCommandTimeoutUs, 1000));
  assert(!CommandFresh(true, 1001 + kLinkCommandTimeoutUs, 1000));

  // Wrap-safe: a Command received just before micros() wraps is still fresh after it.
  {
    const uint32_t received = UINT32_MAX - 100;
    assert(CommandFresh(true, 150, received));  // 251 us old
    assert(!CommandFresh(true, received + kLinkCommandTimeoutUs + 1, received));
  }

  // Expiry latch: a Command turns expired once it is stale, also across the wrap. A latched (cleared) Command stays
  // stale even when the age would wrap back below the timeout after 2^32 us.
  {
    const uint32_t received = UINT32_MAX - 100;
    assert(!CommandExpired(true, received + kLinkCommandTimeoutUs, received));
    assert(CommandExpired(true, received + kLinkCommandTimeoutUs + 1, received));  // wraps past zero
    assert(!CommandExpired(false, received + kLinkCommandTimeoutUs + 1, received));  // nothing stored
    const uint32_t wrappedNow = 10;  // micros() after the wrap: the age is 111 us
    assert(CommandFresh(true, wrappedNow, received));  // why the latch is needed
    assert(!CommandFresh(false, wrappedNow, received));  // cleared: never fresh again
  }

  // 2 -> 1: a balancing robot is handed to the link and keeps balancing.
  {
    Sketch s;
    assert(s.Tick(kPos2, false));
    assert(s.Tick(kPos1, true));
  }

  // 2 -> 0 (passes 1): balance stays on for the transit and ends off.
  {
    Sketch s;
    assert(s.Tick(kPos2, false));
    assert(s.Tick(kPos1, true));  // entered while the controller runs
    assert(!s.Tick(kPos0, true));
  }

  // 0 -> 2 (passes 1): the controller is off on entry, so nothing starts; position 2 gives the enable.
  {
    Sketch s;
    assert(!s.Tick(kPos0, false));
    assert(!s.Tick(kPos1, false));
    assert(s.Tick(kPos2, false));
  }

  // 0 -> 1: nothing starts until the link sets the flag with balance on (the balance action).
  {
    Sketch s;
    assert(!s.Tick(kPos0, false));
    assert(!s.Tick(kPos1, false));
    s.linkBalance = true;  // balance on=1
    assert(RcMode::EnableSwitch(s.position, s.linkBalance));
  }

  return 0;
}
