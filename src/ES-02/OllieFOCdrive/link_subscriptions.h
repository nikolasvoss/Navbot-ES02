#ifndef LINK_SUBSCRIPTIONS_H
#define LINK_SUBSCRIPTIONS_H

#include <stddef.h>
#include <stdint.h>

#include "link_schedule.h"
#include "signal_table.h"

// The robot's subscription slots: which port owns each, which entries it streams, when the next row is due, and
// whether the port can carry it. Pure logic; the control loop calls it at tick start (Subscribe, Unsubscribe) and at
// tick end (Sample).

constexpr size_t kLinkMaxSlots = 8;
constexpr size_t kLinkMaxIds = 48;  // one row of 48 values (196 bytes) fits one frame
constexpr size_t kLinkMaxRowLen = 4 + 4 * kLinkMaxIds;
constexpr uint32_t kLinkLeaseUs = 5000000;
// Longest period a subscription may ask for. LinkScheduleDue compares times across the micros() wrap only for gaps under
// 2^31 us (about 35 minutes), so a longer period is refused; a period of a minute already means one row a minute.
constexpr uint32_t kLinkMaxPeriodUs = 60000000;

constexpr size_t kLinkPortCount = 2;
constexpr uint8_t kLinkPortUart = 0;  // UART1, the ros2_pc
constexpr uint8_t kLinkPortUsb = 1;   // UART0, the CH340

// A port may carry 70 % of its line rate. The rest stays free for Log and Ack frames.
constexpr uint32_t kLinkBudgetPercent = 70;
// Bytes a row adds to its frame: header, CRC, COBS and delimiter. The PC encoder's SignalBatch adds 20 to 25 bytes beyond
// its row: 25 at 31 to 48 entries once seq and t_ms are near 2^32, so 26 covers it.
constexpr uint32_t kLinkFrameOverheadBytes = 26;
// A row every tick is budgeted at this rate. Measured control rates were 424, 521 and about 710 passes per second as
// the code changed; the tick gate allows at most 1000.
constexpr uint32_t kLinkAssumedTickHz = 710;

struct LinkPortBudget {
  uint32_t lineBytesPerSec;      // baud / 10
  uint32_t reservedBytesPerSec;  // taken before any subscription: odometry and IMU on UART1
};

// What a subscription of idCount entries adds to its port, in bytes per second, including frame overhead.
uint32_t LinkSubscriptionLoad(size_t idCount, uint32_t periodUs);
// What all subscriptions on the port may add together.
uint32_t LinkPortAllowance(const LinkPortBudget& budget);

struct LinkSubscribeResult {
  bool accepted;
  uint8_t slot;     // the slot that now holds the subscription
  char error[65];  // the reason when refused; fits Ack.error
};

class LinkRowSink {
 public:
  virtual ~LinkRowSink() {}
  // One row: [t_us u32][4 bytes per subscribed entry], little-endian.
  virtual void OnRow(uint8_t port, uint8_t slot, const uint8_t* row, size_t len) = 0;
  virtual void OnLeaseExpired(uint8_t port, uint8_t slot) { (void)port; (void)slot; }
};

class LinkSubscriptions {
 public:
  LinkSubscriptions(const SignalEntry* table, size_t count, const LinkPortBudget (&budgets)[kLinkPortCount]);

  // slot 0 asks for a free slot. A nonzero slot renews the lease, or replaces the list, of a slot the same port owns.
  // A renewal that changes neither the entries nor the period keeps the schedule and the running values.
  LinkSubscribeResult Subscribe(uint8_t port, uint32_t slot, const uint16_t* ids, size_t idCount, uint32_t periodUs,
                                uint32_t nowUs);
  // A free slot is accepted. Another port's slot is refused.
  LinkSubscribeResult Unsubscribe(uint8_t port, uint32_t slot);

  // End of a control tick, with nowUs the micros() value at its start. Drops expired leases, folds this tick into the
  // running minimum and maximum, and emits one row for each subscription that is due.
  void Sample(uint32_t nowUs, LinkRowSink& sink);

  size_t activeCount() const;

 private:
  struct Slot {
    bool active = false;
    uint8_t port = 0;
    uint8_t idCount = 0;
    uint16_t ids[kLinkMaxIds] = {};
    uint32_t periodUs = 0;
    uint32_t renewedUs = 0;
    LinkSchedule schedule;
    uint32_t running[kLinkMaxIds] = {};  // minimum or maximum since the previous row, for Min and Max entries
    uint64_t runningValid = 0;           // bit i: running[i] holds a value
  };

  uint32_t PortLoadExcluding(uint8_t port, size_t excludedSlotIndex) const;
  void Observe(Slot& slot) const;

  const SignalEntry* table_;
  size_t count_;
  LinkPortBudget budgets_[kLinkPortCount];
  Slot slots_[kLinkMaxSlots];
};

#endif
