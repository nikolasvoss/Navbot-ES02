#include "link_subscriptions.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace {

uint32_t gPlain = 0;
float gMin = 0.0f;
int32_t gMax = 0;

uint32_t ReadPlain() { return gPlain; }
uint32_t ReadMin() {
  uint32_t bits;
  std::memcpy(&bits, &gMin, sizeof(bits));
  return bits;
}
uint32_t ReadMax() { return static_cast<uint32_t>(gMax); }

// Ids: 0 plain u32, 1 running minimum f32, 2 running maximum i32, 3 an action.
const SignalEntry kTable[] = {
    {"plain", "", SignalKind::Signal, SignalType::U32, ReadPlain, nullptr, nullptr, nullptr, 0.0f},
    {"low", "V", SignalKind::Min, SignalType::F32, ReadMin, nullptr, nullptr, nullptr, 0.0f},
    {"high", "us", SignalKind::Max, SignalType::I32, ReadMax, nullptr, nullptr, nullptr, 0.0f},
    {"go", "", SignalKind::Action, SignalType::None, nullptr, nullptr, nullptr, nullptr, 0.0f},
};
constexpr size_t kTableCount = sizeof(kTable) / sizeof(kTable[0]);

// UART1 at 2000000 baud with 5 kB/s taken by odometry and IMU; USB at 576000 baud.
const LinkPortBudget kBudgets[kLinkPortCount] = {{200000, 5000}, {57600, 0}};

struct Rows : LinkRowSink {
  struct Row {
    uint8_t port;
    uint8_t slot;
    std::vector<uint8_t> bytes;
  };
  std::vector<Row> rows;
  std::vector<std::pair<int, int>> expired;
  void OnRow(uint8_t port, uint8_t slot, const uint8_t* row, size_t len) override {
    rows.push_back({port, slot, std::vector<uint8_t>(row, row + len)});
  }
  void OnLeaseExpired(uint8_t port, uint8_t slot) override { expired.push_back({port, slot}); }
};

bool failed = false;

void Check(bool condition, const char* what) {
  std::printf("%s: %s\n", what, condition ? "ok" : "FAIL");
  if (!condition) failed = true;
}

bool Refused(const LinkSubscribeResult& r, const char* error) { return !r.accepted && std::strcmp(r.error, error) == 0; }

std::vector<uint16_t> Ids(size_t count, uint16_t id = 0) { return std::vector<uint16_t>(count, id); }

LinkSubscribeResult Sub(LinkSubscriptions& s, uint8_t port, uint32_t slot, size_t count, uint32_t periodUs,
                        uint32_t nowUs = 0) {
  const std::vector<uint16_t> ids = Ids(count);
  return s.Subscribe(port, slot, ids.data(), ids.size(), periodUs, nowUs);
}

void TestBudget() {
  Check(LinkPortAllowance(kBudgets[0]) == 135000, "UART1 allows 70 % of 200 kB/s minus 5 kB/s");
  Check(LinkPortAllowance(kBudgets[1]) == 40320, "USB allows 70 % of 57.6 kB/s");
  // Each subscription is charged (4 + 4 x entries + 24 frame bytes) x rate; see link_subscriptions.h.
  Check(LinkSubscriptionLoad(1, 0) == 24140, "1 entry every tick: (8 + 26) B x 710 Hz");
  Check(LinkSubscriptionLoad(8, 0) == 44020, "8 entries every tick: (36 + 26) B x 710 Hz");
  Check(LinkSubscriptionLoad(40, 100000) == 1900, "40 entries at 10 Hz: (164 + 26) B x 10 Hz");
  Check(LinkSubscriptionLoad(48, 0) == 157620, "48 entries every tick: (196 + 26) B x 710 Hz");

  {
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    Check(Sub(s, kLinkPortUart, 0, 40, 100000).accepted && Sub(s, kLinkPortUsb, 0, 40, 100000).accepted,
          "40 entries at 10 Hz fit both ports");
  }
  {
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    Check(Sub(s, kLinkPortUart, 0, 8, 0).accepted, "8 entries every tick fit UART1");
    Check(Refused(Sub(s, kLinkPortUsb, 0, 8, 0), "over budget: 44.0 kB/s of 40.3 kB/s allowed"),
          "8 entries every tick are refused on USB, with the numbers");
  }
  {
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    Check(Sub(s, kLinkPortUart, 0, 20, 0).accepted, "20 entries every tick fit UART1");
    const LinkSubscribeResult usb = Sub(s, kLinkPortUsb, 0, 20, 0);
    Check(Refused(usb, "over budget: 78.1 kB/s of 40.3 kB/s allowed"), "20 entries every tick are refused on USB, with the numbers");
    Check(Sub(s, kLinkPortUsb, 0, 20, 3000).accepted, "20 entries at a 3 ms period fit USB");
  }
  {
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    const LinkSubscribeResult uart = Sub(s, kLinkPortUart, 0, 48, 0);
    Check(Refused(uart, "over budget: 157.6 kB/s of 135.0 kB/s allowed"), "48 entries every tick are refused on UART1, with the numbers");
    Check(Sub(s, kLinkPortUart, 0, 48, 2000).accepted, "48 entries at a 2 ms period fit UART1");
  }
  {
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    Check(Sub(s, kLinkPortUart, 0, 48, 2000).accepted, "slot 1: 111.0 kB/s on UART1");
    const LinkSubscribeResult second = Sub(s, kLinkPortUart, 0, 20, 0);
    Check(Refused(second, "over budget: 189.1 kB/s of 135.0 kB/s allowed"), "a second subscription adds to the port's load");
    Check(Sub(s, kLinkPortUsb, 0, 4, 0).accepted, "the other port's budget is separate");
    Check(Sub(s, kLinkPortUart, 1, 48, 1500).accepted == false, "replacing slot 1 by a faster list is checked on its own");
    Check(Sub(s, kLinkPortUart, 1, 48, 2000).accepted, "renewing slot 1 does not count it twice");
  }
  {
    // Two 2-entry subscriptions at period 0 already exceed USB, so four slots of them cannot all be granted.
    LinkSubscriptions s(kTable, kTableCount, kBudgets);
    Check(Sub(s, kLinkPortUsb, 0, 2, 0).accepted, "one 2-entry subscription at period 0 fits USB");
    Check(Refused(Sub(s, kLinkPortUsb, 0, 2, 0), "over budget: 54.0 kB/s of 40.3 kB/s allowed"),
          "a second 2-entry subscription at period 0 is refused on USB, so four slots of them cannot fit");
    Check(s.activeCount() == 1, "only the first subscription holds a slot");
  }
}

void TestSlotsAndOwnership() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  const LinkSubscribeResult first = Sub(s, kLinkPortUart, 0, 2, 10000);
  Check(first.accepted && first.slot == 1, "slot 0 asks for the lowest free slot");
  const LinkSubscribeResult usb = Sub(s, kLinkPortUsb, 0, 2, 10000);
  Check(usb.accepted && usb.slot == 2, "the other port gets the next slot");

  Check(Refused(Sub(s, kLinkPortUsb, 1, 2, 10000), "slot in use"), "USB cannot renew UART1's slot");
  Check(Refused(s.Unsubscribe(kLinkPortUsb, 1), "slot in use"), "USB cannot unsubscribe UART1's slot");
  Check(Sub(s, kLinkPortUart, 1, 2, 20000).accepted, "the owner replaces its list");
  Check(Refused(Sub(s, kLinkPortUart, 3, 2, 10000), "no subscription in slot 3"), "a free slot cannot be renewed");
  Check(Refused(Sub(s, kLinkPortUart, 9, 2, 10000), "no subscription in slot 9"), "a slot number past the table is refused");

  for (uint32_t expected = 3; expected <= kLinkMaxSlots; ++expected)
    Check(Sub(s, expected % 2 ? kLinkPortUart : kLinkPortUsb, 0, 1, 10000).slot == expected, "the slots fill up in order");
  Check(Refused(Sub(s, kLinkPortUart, 0, 1, 10000), "all 8 slots are in use"), "a ninth subscription is refused");
  Check(s.activeCount() == kLinkMaxSlots, "every slot is active");

  Check(s.Unsubscribe(kLinkPortUart, 1).accepted && s.activeCount() == kLinkMaxSlots - 1, "the owner unsubscribes");
  Check(s.Unsubscribe(kLinkPortUart, 1).accepted, "unsubscribing a free slot is accepted");
  Check(Sub(s, kLinkPortUsb, 0, 1, 10000).slot == 1, "a freed slot is handed out again");

  const uint16_t action[] = {3};
  Check(Refused(s.Subscribe(kLinkPortUart, 0, action, 1, 0, 0), "not a signal: 3"), "an action cannot be streamed");
  const uint16_t pastEnd[] = {0, 4};
  Check(Refused(s.Subscribe(kLinkPortUart, 0, pastEnd, 2, 0, 0), "not a signal: 4"), "an id past the table is refused");
  Check(Refused(s.Subscribe(kLinkPortUart, 0, pastEnd, 0, 0, 0), "need 1 to 48 entries, got 0"), "an empty list is refused");
}

void TestLease() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  Rows rows;
  Sub(s, kLinkPortUart, 0, 1, 1000000, 1000);
  s.Sample(1000 + kLinkLeaseUs, rows);
  Check(rows.expired.empty() && s.activeCount() == 1, "alive exactly 5 s after the request");
  s.Sample(1000 + kLinkLeaseUs + 1, rows);
  Check(rows.expired.size() == 1 && rows.expired[0] == std::make_pair(0, 1) && s.activeCount() == 0,
        "expired one microsecond later, and reported with its port and slot");

  rows = Rows();
  Sub(s, kLinkPortUsb, 0, 1, 1000000, 100);
  Sub(s, kLinkPortUsb, 1, 1, 1000000, 3000000);  // renewal
  s.Sample(7999999, rows);
  Check(rows.expired.empty(), "a renewal restarts the 5 s");
  s.Sample(8000001, rows);
  Check(rows.expired.size() == 1 && s.activeCount() == 0, "and the slot ends 5 s after the renewal");

  rows = Rows();
  Sub(s, kLinkPortUart, 0, 1, 1000000, 0xFFFFFF00u);  // micros() wraps 256 us later
  s.Sample(0xFFFFFF00u + kLinkLeaseUs, rows);
  Check(rows.expired.empty(), "the lease check survives the micros() wrap");
  s.Sample(0xFFFFFF00u + kLinkLeaseUs + 1, rows);
  Check(rows.expired.size() == 1, "and ends across it");
}

void TestSchedule() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  Rows rows;
  Sub(s, kLinkPortUart, 0, 1, 2500, 1000);
  std::vector<uint32_t> times;
  for (uint32_t t = 1000; t <= 11000; t += 1000) {
    s.Sample(t, rows);
  }
  for (const Rows::Row& r : rows.rows) times.push_back(r.bytes[0] | (r.bytes[1] << 8));
  const std::vector<uint32_t> expected = {1000, 4000, 6000, 9000, 11000};
  Check(times == expected, "a 2.5 ms period on a 1 ms tick: first tick, then the first tick at or after each deadline");

  LinkSubscriptions every(kTable, kTableCount, kBudgets);
  Rows rowsEvery;
  Sub(every, kLinkPortUart, 0, 1, 0, 1000);
  for (uint32_t t = 1000; t <= 3000; t += 1000) every.Sample(t, rowsEvery);
  Check(rowsEvery.rows.size() == 3, "period 0 gives a row on every tick");

  LinkSubscriptions two(kTable, kTableCount, kBudgets);
  Rows rowsTwo;
  Sub(two, kLinkPortUart, 0, 1, 0, 0);
  Sub(two, kLinkPortUsb, 0, 1, 0, 0);
  two.Sample(5000, rowsTwo);
  Check(rowsTwo.rows.size() == 2 && rowsTwo.rows[0].port == 0 && rowsTwo.rows[1].port == 1 &&
            rowsTwo.rows[0].bytes == rowsTwo.rows[1].bytes,
        "rows of two slots from the same tick carry the same t_us, each goes to its owner");
}

void TestRowLayoutAndRunningValues() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  Rows rows;
  const uint16_t ids[] = {0, 1, 2};
  s.Subscribe(kLinkPortUart, 0, ids, 3, 3000, 0);

  struct Tick {
    uint32_t t;
    float low;
    int32_t high;
  };
  const Tick ticks[] = {{1000, 5.0f, -2}, {2000, 3.0f, 10}, {3000, 4.0f, 7}, {4000, 6.0f, 1},
                        {5000, 9.0f, 0},  {6000, 8.0f, 100}, {7000, 9.0f, 3}};
  gPlain = 0x11223344u;
  for (const Tick& tick : ticks) {
    gMin = tick.low;
    gMax = tick.high;
    s.Sample(tick.t, rows);
  }
  Check(rows.rows.size() == 3, "rows at 1000, 4000 and 7000 us");

  // [t_us][plain][min][max], little-endian.
  const uint8_t first[] = {0xE8, 0x03, 0x00, 0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x00, 0xA0, 0x40, 0xFE, 0xFF, 0xFF, 0xFF};
  const uint8_t second[] = {0xA0, 0x0F, 0x00, 0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x00, 0x40, 0x40, 0x0A, 0x00, 0x00, 0x00};
  const uint8_t third[] = {0x58, 0x1B, 0x00, 0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x00, 0x00, 0x41, 0x64, 0x00, 0x00, 0x00};
  auto is = [&](size_t i, const uint8_t* expected) {
    return rows.rows[i].bytes.size() == 16 && std::memcmp(rows.rows[i].bytes.data(), expected, 16) == 0;
  };
  Check(is(0, first), "row 1: min 5.0 and max -2 of the one tick so far");
  Check(is(1, second), "row 2: min 3.0 and max 10 over ticks 2 to 4");
  Check(is(2, third), "row 3: min 8.0 and max 100 over ticks 5 to 7, so row 2 reset both");

  rows = Rows();
  LinkSubscriptions negative(kTable, kTableCount, kBudgets);
  negative.Subscribe(kLinkPortUart, 0, ids, 3, 2000, 0);
  gMin = -1.0f;
  gMax = -5;
  negative.Sample(1000, rows);
  gMin = -4.0f;
  gMax = -9;
  negative.Sample(2000, rows);
  gMin = -2.0f;
  gMax = -7;
  negative.Sample(3000, rows);
  // Second row: min over ticks 2 and 3 is -4.0 (0xC0800000), max is -7 (0xFFFFFFF9).
  const uint8_t signedRow[] = {0xB8, 0x0B, 0x00, 0x00, 0x44, 0x33, 0x22, 0x11, 0x00, 0x00, 0x80, 0xC0, 0xF9, 0xFF, 0xFF, 0xFF};
  Check(rows.rows.size() == 2 && rows.rows[1].bytes.size() == 16 &&
            std::memcmp(rows.rows[1].bytes.data(), signedRow, 16) == 0,
        "min and max compare floats and signed integers by value");
}

void TestRenewalKeepsState() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  Rows rows;
  const uint16_t ids[] = {1};
  s.Subscribe(kLinkPortUart, 0, ids, 1, 3000, 0);
  gMin = 5.0f;
  s.Sample(1000, rows);  // row 1, window restarts
  gMin = 2.0f;
  s.Sample(2000, rows);
  s.Subscribe(kLinkPortUart, 1, ids, 1, 3000, 2500);  // same list and period: a renewal
  gMin = 4.0f;
  s.Sample(4000, rows);
  Check(rows.rows.size() == 2 && rows.rows[1].bytes[4] == 0x00 && rows.rows[1].bytes[7] == 0x40,
        "a renewal keeps the schedule and the running minimum (2.0 = 0x40000000)");
  s.Subscribe(kLinkPortUart, 1, ids, 1, 4000, 4100);  // another period: a fresh start
  gMin = 7.0f;
  s.Sample(5000, rows);
  Check(rows.rows.size() == 3 && rows.rows[2].bytes[7] == 0x40 && rows.rows[2].bytes[6] == 0xE0,
        "a changed period starts the schedule over, so a row is due at once (7.0 = 0x40E00000)");
}

void TestPeriodLimit() {
  LinkSubscriptions s(kTable, kTableCount, kBudgets);
  Check(Sub(s, kLinkPortUart, 0, 1, 60000000).accepted, "a period of 60 s is accepted");
  Check(Refused(Sub(s, kLinkPortUart, 0, 1, 60000001), "period_us must be at most 60000000"),
        "a period just above 60 s is refused");
  Check(Refused(Sub(s, kLinkPortUart, 0, 1, 0xFFFFFFFF), "period_us must be at most 60000000"),
        "the largest period is refused with the same text");
  Check(s.activeCount() == 1, "refused periods hold no slot");
}

}  // namespace

int main() {
  TestBudget();
  TestSlotsAndOwnership();
  TestLease();
  TestSchedule();
  TestRowLayoutAndRunningValues();
  TestRenewalKeepsState();
  TestPeriodLimit();
  return failed ? 1 : 0;
}
