#include "link_subscriptions.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {

__attribute__((format(printf, 2, 3))) void Refuse(LinkSubscribeResult& result, const char* format, ...) {
  result.accepted = false;
  result.slot = 0;
  va_list args;
  va_start(args, format);
  vsnprintf(result.error, sizeof(result.error), format, args);
  va_end(args);
}

LinkSubscribeResult Accepted(uint8_t slot) {
  LinkSubscribeResult result{};
  result.accepted = true;
  result.slot = slot;
  return result;
}

// The rate a period gives, rounded up and never above the control tick rate.
uint32_t RateHz(uint32_t periodUs) {
  if (periodUs == 0) return kLinkAssumedTickHz;
  const uint32_t rate = static_cast<uint32_t>((1000000ull + periodUs - 1) / periodUs);
  return rate < kLinkAssumedTickHz ? rate : kLinkAssumedTickHz;
}

float AsFloat(uint32_t bits) {
  float value;
  memcpy(&value, &bits, sizeof(value));
  return value;
}

bool Less(SignalType type, uint32_t a, uint32_t b) {
  switch (type) {
    case SignalType::F32:
      return AsFloat(a) < AsFloat(b);
    case SignalType::I32:
      return static_cast<int32_t>(a) < static_cast<int32_t>(b);
    default:
      return a < b;
  }
}

}  // namespace

uint32_t LinkSubscriptionLoad(size_t idCount, uint32_t periodUs) {
  const uint64_t rowBytes = 4 + 4 * idCount;
  return static_cast<uint32_t>((rowBytes + kLinkFrameOverheadBytes) * RateHz(periodUs));
}

uint32_t LinkPortAllowance(const LinkPortBudget& budget) {
  const uint32_t share = static_cast<uint32_t>(static_cast<uint64_t>(budget.lineBytesPerSec) * kLinkBudgetPercent / 100);
  return share > budget.reservedBytesPerSec ? share - budget.reservedBytesPerSec : 0;
}

LinkSubscriptions::LinkSubscriptions(const SignalEntry* table, size_t count,
                                     const LinkPortBudget (&budgets)[kLinkPortCount])
    : table_(table), count_(count) {
  for (size_t i = 0; i < kLinkPortCount; ++i) budgets_[i] = budgets[i];
}

uint32_t LinkSubscriptions::PortLoadExcluding(uint8_t port, size_t excludedSlotIndex) const {
  uint32_t load = 0;
  for (size_t i = 0; i < kLinkMaxSlots; ++i) {
    const Slot& slot = slots_[i];
    if (slot.active && slot.port == port && i != excludedSlotIndex) load += LinkSubscriptionLoad(slot.idCount, slot.periodUs);
  }
  return load;
}

LinkSubscribeResult LinkSubscriptions::Subscribe(uint8_t port, uint32_t slotNumber, const uint16_t* ids, size_t idCount,
                                                 uint32_t periodUs, uint32_t nowUs) {
  LinkSubscribeResult result{};
  if (port >= kLinkPortCount) {
    Refuse(result, "unknown port");
    return result;
  }
  for (size_t i = 0; i < idCount; ++i) {
    if (ids[i] >= count_ || table_[ids[i]].kind == SignalKind::Action) {
      Refuse(result, "not a signal: %u", static_cast<unsigned>(ids[i]));
      return result;
    }
  }
  if (idCount == 0 || idCount > kLinkMaxIds) {
    Refuse(result, "need 1 to %u entries, got %u", static_cast<unsigned>(kLinkMaxIds), static_cast<unsigned>(idCount));
    return result;
  }
  if (periodUs > kLinkMaxPeriodUs) {
    Refuse(result, "period_us must be at most %u", static_cast<unsigned>(kLinkMaxPeriodUs));
    return result;
  }

  size_t index = 0;
  if (slotNumber == 0) {
    while (index < kLinkMaxSlots && slots_[index].active) ++index;
    if (index == kLinkMaxSlots) {
      Refuse(result, "all %u slots are in use", static_cast<unsigned>(kLinkMaxSlots));
      return result;
    }
  } else {
    if (slotNumber > kLinkMaxSlots || !slots_[slotNumber - 1].active) {
      Refuse(result, "no subscription in slot %u", static_cast<unsigned>(slotNumber));
      return result;
    }
    index = slotNumber - 1;
    if (slots_[index].port != port) {
      Refuse(result, "slot in use");
      return result;
    }
  }

  // A replaced slot does not count against itself.
  const uint32_t load = PortLoadExcluding(port, index) + LinkSubscriptionLoad(idCount, periodUs);
  const uint32_t allowance = LinkPortAllowance(budgets_[port]);
  if (load > allowance) {
    // Tenths of a kB/s, rounded.
    const unsigned loadTenths = static_cast<unsigned>((load + 50) / 100);
    const unsigned allowanceTenths = static_cast<unsigned>((allowance + 50) / 100);
    Refuse(result, "over budget: %u.%u kB/s of %u.%u kB/s allowed", loadTenths / 10, loadTenths % 10,
           allowanceTenths / 10, allowanceTenths % 10);
    return result;
  }

  Slot& slot = slots_[index];
  const bool same = slot.active && slot.idCount == idCount && slot.periodUs == periodUs &&
                    memcmp(slot.ids, ids, idCount * sizeof(ids[0])) == 0;
  if (!same) {
    slot = Slot();
    slot.active = true;
    slot.port = port;
    slot.idCount = static_cast<uint8_t>(idCount);
    memcpy(slot.ids, ids, idCount * sizeof(ids[0]));
    slot.periodUs = periodUs;
  }
  slot.renewedUs = nowUs;
  return Accepted(static_cast<uint8_t>(index + 1));
}

LinkSubscribeResult LinkSubscriptions::Unsubscribe(uint8_t port, uint32_t slotNumber) {
  LinkSubscribeResult result{};
  if (slotNumber >= 1 && slotNumber <= kLinkMaxSlots && slots_[slotNumber - 1].active) {
    Slot& slot = slots_[slotNumber - 1];
    if (slot.port != port) {
      Refuse(result, "slot in use");
      return result;
    }
    slot = Slot();
  }
  return Accepted(static_cast<uint8_t>(slotNumber));
}

void LinkSubscriptions::Observe(Slot& slot) const {
  for (size_t i = 0; i < slot.idCount; ++i) {
    const SignalEntry& entry = table_[slot.ids[i]];
    if (entry.kind != SignalKind::Min && entry.kind != SignalKind::Max) continue;
    const uint32_t value = entry.read();
    const uint64_t bit = 1ull << i;
    const bool better = entry.kind == SignalKind::Min ? Less(entry.type, value, slot.running[i])
                                                       : Less(entry.type, slot.running[i], value);
    if (!(slot.runningValid & bit) || better) slot.running[i] = value;
    slot.runningValid |= bit;
  }
}

void LinkSubscriptions::Sample(uint32_t nowUs, LinkRowSink& sink) {
  for (size_t index = 0; index < kLinkMaxSlots; ++index) {
    Slot& slot = slots_[index];
    if (!slot.active) continue;
    const uint8_t slotNumber = static_cast<uint8_t>(index + 1);
    if (static_cast<uint32_t>(nowUs - slot.renewedUs) > kLinkLeaseUs) {
      const uint8_t port = slot.port;
      slot = Slot();
      sink.OnLeaseExpired(port, slotNumber);
      continue;
    }
    Observe(slot);
    if (!LinkScheduleDue(slot.schedule, nowUs, slot.periodUs)) continue;

    uint8_t row[kLinkMaxRowLen];
    for (int shift = 0; shift < 4; ++shift) row[shift] = static_cast<uint8_t>(nowUs >> (8 * shift));
    for (size_t i = 0; i < slot.idCount; ++i) {
      const SignalEntry& entry = table_[slot.ids[i]];
      const bool ranged = entry.kind == SignalKind::Min || entry.kind == SignalKind::Max;
      const uint32_t value = ranged ? slot.running[i] : entry.read();
      for (int shift = 0; shift < 4; ++shift) row[4 + 4 * i + shift] = static_cast<uint8_t>(value >> (8 * shift));
    }
    slot.runningValid = 0;
    sink.OnRow(slot.port, slotNumber, row, 4 + 4 * slot.idCount);
  }
}

size_t LinkSubscriptions::activeCount() const {
  size_t active = 0;
  for (const Slot& slot : slots_) active += slot.active ? 1 : 0;
  return active;
}
