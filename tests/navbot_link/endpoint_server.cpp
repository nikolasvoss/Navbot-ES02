// The firmware's link code behind stdin and stdout, for tests/navbot_link/test_firmware_endpoint.py: the PC library
// talks to the same LinkEndpoint that runs on the robot. One port, a table of counters that advance on every tick, and
// a real-time tick of about 1 ms.
#include <fcntl.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>

#include <cstring>

#include "link_endpoint.h"
#include "link_tx.h"

namespace {

uint32_t gTick = 0;
float gGain = 0.25f;

uint32_t Bits(float value) {
  uint32_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}

uint32_t ReadTick() { return gTick; }
uint32_t ReadRamp() { return Bits(0.5f * static_cast<float>(gTick)); }
uint32_t ReadNegative() { return static_cast<uint32_t>(-static_cast<int32_t>(gTick)); }
uint32_t ReadOdd() { return gTick % 2; }
uint32_t ReadFalling() { return Bits(100.0f - static_cast<float>(gTick)); }
// A spike on one tick in seven: a sampled stream sees it rarely, a running maximum never misses it.
uint32_t ReadSpike() { return gTick % 7 == 3 ? 1000u : 1u; }
uint32_t ReadGain() { return Bits(gGain); }

const SignalArg kGoArgs[] = {{"left_a", -2.0f, 2.0f}, {"lease_ms", 1.0f, 1000.0f}};
const SignalEntry kTable[] = {
    {"tick", "", SignalKind::Signal, SignalType::U32, ReadTick, nullptr, 0},
    {"ramp", "m", SignalKind::Signal, SignalType::F32, ReadRamp, nullptr, 0},
    {"negative", "", SignalKind::Signal, SignalType::I32, ReadNegative, nullptr, 0},
    {"odd", "", SignalKind::Signal, SignalType::Bool, ReadOdd, nullptr, 0},
    {"falling_min", "V", SignalKind::Min, SignalType::F32, ReadFalling, nullptr, 0},
    {"spike_max", "", SignalKind::Max, SignalType::U32, ReadSpike, nullptr, 0},
    {"gain", "A/rad", SignalKind::Param, SignalType::F32, ReadGain, nullptr, 0},
    {"go", "", SignalKind::Action, SignalType::None, nullptr, kGoArgs, 2},
};
const LinkPortBudget kBudgets[kLinkPortCount] = {{200000, 5000}, {57600, 0}};

uint32_t NowUs() {
  timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint32_t>(now.tv_sec * 1000000ull + now.tv_nsec / 1000);
}

// Sends each message on the one port and writes the frame to stdout at once.
class StdoutSink : public LinkMessageSink {
 public:
  StdoutSink() : port_(storage_, sizeof(storage_), false) {}
  void Send(uint8_t, navbot_link_v1_Message& msg) override {
    LinkPortSend(port_, msg, NowUs() / 1000);
    uint8_t tag;
    uint8_t frame[LINK_MAX_ITEM_LEN];
    size_t len;
    while (port_.queue.Pop(tag, frame, len)) {
      for (size_t written = 0; written < len;) {
        const ssize_t n = write(STDOUT_FILENO, frame + written, len - written);
        if (n > 0) written += static_cast<size_t>(n);
      }
    }
  }

 private:
  uint8_t storage_[2048];
  LinkPort port_;
};

struct Requests : LinkRxSink {
  LinkEndpoint* endpoint = nullptr;
  uint32_t nowUs = 0;
  void OnPayload(const uint8_t* payload, size_t len) override { endpoint->Handle(kLinkPortUart, payload, len, nowUs); }
};

}  // namespace

int main() {
  StdoutSink sink;
  LinkEndpoint endpoint(kTable, sizeof(kTable) / sizeof(kTable[0]), "server-1", kBudgets, sink);
  LinkRxDecoder decoder(false);
  Requests requests;
  requests.endpoint = &endpoint;
  for (;;) {
    pollfd input = {STDIN_FILENO, POLLIN, 0};
    if (poll(&input, 1, 1) > 0) {
      uint8_t bytes[256];
      const ssize_t n = read(STDIN_FILENO, bytes, sizeof(bytes));
      if (n <= 0) return 0;  // the test closed the pipe
      requests.nowUs = NowUs();
      decoder.Feed(bytes, static_cast<size_t>(n), requests);
    }
    ++gTick;
    endpoint.Sample(NowUs());
  }
}
