#pragma once

#include <cstdint>

// CH5 positions and the link Command age, without Arduino so that the host tests can run them. The rules are in
// docs/integration/plans/link-autonomy.md (CH5 positions).
namespace RcMode {

// Raw S.BUS values at which CH5 reaches position 1 (the link) and position 2 (RC).
constexpr int16_t kCh5LinkRaw = 592;
constexpr int16_t kCh5RcRaw = 1402;
// A link Command older than this counts as lost: the demand goes to zero.
constexpr uint32_t kLinkCommandTimeoutUs = 250000;

// 0 below kCh5LinkRaw, 1 below kCh5RcRaw, 2 otherwise.
inline uint8_t Ch5Position(int16_t raw) { return raw < kCh5LinkRaw ? 0 : (raw < kCh5RcRaw ? 1 : 2); }

// The balance enable: position 2, or position 1 while the link holds balance on.
inline bool EnableSwitch(uint8_t position, bool linkBalance) { return position == 2 || (position == 1 && linkBalance); }

// link_balance when CH5 enters position 1: it starts only if the controller is already driving, so the transit through
// position 1 never starts a robot that was off.
inline bool LinkBalanceOnEntry(bool controllerDriving) { return controllerDriving; }

// A Command counts while it is at most kLinkCommandTimeoutUs old. The subtraction wraps, as micros() does.
inline bool CommandFresh(bool seen, uint32_t nowUs, uint32_t receivedUs) {
  return seen && static_cast<uint32_t>(nowUs - receivedUs) <= kLinkCommandTimeoutUs;
}

}  // namespace RcMode
