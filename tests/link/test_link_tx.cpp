#include "link_tx.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// Odometry frames for t_ms = 2, t_us = 1000, left 1.5, right -0.25 with seq 1..4, built with
// navbot_link.framing.encode_frame from the python protobuf Message.
namespace {

const uint8_t kOdomSeq1[] = {0x0B, 0x08, 0x01, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
                             0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0xC8, 0xD4, 0x00};
const uint8_t kOdomSeq2[] = {0x0B, 0x08, 0x02, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
                             0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0x4A, 0x30, 0x00};
const uint8_t kOdomSeq4[] = {0x0B, 0x08, 0x04, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
                             0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0x6F, 0xE9, 0x00};

typedef std::vector<uint8_t> Bytes;

Bytes Of(const uint8_t* data, size_t len) { return Bytes(data, data + len); }

Bytes Concat(const Bytes& a, const Bytes& b) {
  Bytes out = a;
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

std::string Hex(const Bytes& bytes) {
  std::string out;
  char buf[4];
  for (size_t i = 0; i < bytes.size(); ++i) {
    std::snprintf(buf, sizeof(buf), "%02X ", bytes[i]);
    out += buf;
  }
  return out;
}

bool Check(const char* name, bool ok) {
  std::printf("%s: %s\n", name, ok ? "ok" : "FAIL");
  return ok;
}

bool CheckBytes(const char* name, const Bytes& actual, const Bytes& expected) {
  const bool ok = actual == expected;
  std::printf("%s: %s\n", name, ok ? "ok" : "FAIL");
  if (!ok) std::printf("  actual   %s\n  expected %s\n", Hex(actual).c_str(), Hex(expected).c_str());
  return ok;
}

// Accepts at most `available` bytes per AvailableForWrite() and records every Write call.
class FakeSink : public LinkByteSink {
 public:
  int available = 0;
  Bytes written;
  std::vector<size_t> writeSizes;

  int AvailableForWrite() override { return available; }
  void Write(const uint8_t* data, size_t len) override {
    written.insert(written.end(), data, data + len);
    writeSizes.push_back(len);
    available -= static_cast<int>(len);
  }
};

navbot_link_v1_Message Odometry() {
  navbot_link_v1_Message msg = navbot_link_v1_Message_init_zero;
  msg.which_body = navbot_link_v1_Message_odom_tag;
  msg.body.odom.t_us = 1000;
  msg.body.odom.left_angle_rad = 1.5;
  msg.body.odom.right_angle_rad = -0.25;
  return msg;
}

bool Send(LinkPort& port) {
  navbot_link_v1_Message msg = Odometry();
  return LinkPortSend(port, msg, 2);
}

bool TestWholeFrameUnderPartialAvailability() {
  uint8_t storage[256];
  LinkPort port(storage, sizeof(storage), false);
  Send(port);
  Send(port);
  FakeSink sink;
  bool ok = true;

  sink.available = 30;  // one byte short of a 31-byte frame
  ok &= Check("30 free: nothing written", port.writer.Pump(port.queue, sink) == 0 && sink.written.empty());

  sink.available = 31;
  ok &= Check("31 free: first frame written", port.writer.Pump(port.queue, sink) == 31);
  ok &= CheckBytes("first frame bytes", sink.written, Of(kOdomSeq1, sizeof(kOdomSeq1)));

  sink.available = 60;
  port.writer.Pump(port.queue, sink);
  ok &= CheckBytes("second frame follows whole", sink.written,
                   Concat(Of(kOdomSeq1, sizeof(kOdomSeq1)), Of(kOdomSeq2, sizeof(kOdomSeq2))));
  ok &= Check("each frame was one Write", sink.writeSizes.size() == 2 && sink.writeSizes[0] == 31 && sink.writeSizes[1] == 31);
  return ok;
}

bool TestDropCountedOnFullQueue() {
  uint8_t storage[70];  // two 33-byte items (2-byte header + 31-byte frame) fit, a third does not
  LinkPort port(storage, sizeof(storage), false);
  bool ok = true;
  ok &= Check("first accepted", Send(port));
  ok &= Check("second accepted", Send(port));
  ok &= Check("third dropped", !Send(port));
  ok &= Check("drop counted once", port.queue.dropped() == 1);

  FakeSink sink;
  sink.available = 1000;
  port.writer.Pump(port.queue, sink);
  ok &= CheckBytes("only the first two frames go out", sink.written,
                   Concat(Of(kOdomSeq1, sizeof(kOdomSeq1)), Of(kOdomSeq2, sizeof(kOdomSeq2))));

  ok &= Check("fourth accepted after drain", Send(port));
  sink.written.clear();
  port.writer.Pump(port.queue, sink);
  ok &= CheckBytes("seq 3 was spent by the drop, so the receiver sees a gap", sink.written, Of(kOdomSeq4, sizeof(kOdomSeq4)));
  return ok;
}

bool TestSeqIsPerPort() {
  uint8_t storageA[256];
  uint8_t storageB[256];
  LinkPort uart(storageA, sizeof(storageA), false);
  LinkPort usb(storageB, sizeof(storageB), false);
  Send(uart);
  Send(uart);
  Send(usb);

  FakeSink uartSink;
  FakeSink usbSink;
  uartSink.available = 1000;
  usbSink.available = 1000;
  uart.writer.Pump(uart.queue, uartSink);
  usb.writer.Pump(usb.queue, usbSink);
  bool ok = true;
  ok &= CheckBytes("uart carries seq 1 then 2", uartSink.written,
                   Concat(Of(kOdomSeq1, sizeof(kOdomSeq1)), Of(kOdomSeq2, sizeof(kOdomSeq2))));
  ok &= CheckBytes("usb starts at seq 1", usbSink.written, Of(kOdomSeq1, sizeof(kOdomSeq1)));
  return ok;
}

bool TestUsbPrefixWaitsForRoom() {
  uint8_t storage[256];
  LinkPort usb(storage, sizeof(storage), true);
  Send(usb);
  Send(usb);
  FakeSink sink;
  bool ok = true;

  sink.available = sizeof(kOdomSeq1);  // one byte short of the 0x00 and the frame
  usb.writer.Pump(usb.queue, sink);
  ok &= CheckBytes("the frame waits for room, not split", sink.written, Bytes{});

  sink.available = 100;
  usb.writer.Pump(usb.queue, sink);
  Bytes expected{0x00};
  expected = Concat(expected, Of(kOdomSeq1, sizeof(kOdomSeq1)));
  expected.push_back(0x00);
  expected = Concat(expected, Of(kOdomSeq2, sizeof(kOdomSeq2)));
  ok &= CheckBytes("each frame goes out behind its 0x00, in push order", sink.written, expected);
  return ok;
}

bool TestRequestQueueKeepsPortAndPayload() {
  uint8_t storage[16];
  LinkItemQueue requests(storage, sizeof(storage));
  const uint8_t a[] = {1, 2, 3, 4, 5, 6};
  const uint8_t b[] = {7, 8, 9, 10, 11, 12};
  bool ok = true;
  ok &= Check("push port 0", requests.Push(0, a, sizeof(a)));
  ok &= Check("push port 1", requests.Push(1, b, sizeof(b)));
  ok &= Check("full queue refuses and counts", !requests.PushOrDrop(1, a, sizeof(a)) && requests.dropped() == 1);
  ok &= Check("plain Push refuses without counting", !requests.Push(1, a, sizeof(a)) && requests.dropped() == 1);

  uint8_t tag = 0xFF;
  uint8_t out[LINK_MAX_ITEM_LEN];
  size_t len = 0;
  ok &= Check("pop first", requests.Pop(tag, out, len) && tag == 0 && CheckBytes("first payload", Bytes(out, out + len), Bytes(a, a + 6)));
  ok &= Check("pop second wraps the ring", requests.Push(0, a, sizeof(a)) && requests.Pop(tag, out, len) && tag == 1 &&
                                               CheckBytes("second payload", Bytes(out, out + len), Bytes(b, b + 6)));
  ok &= Check("pop third", requests.Pop(tag, out, len) && tag == 0 && len == 6);
  ok &= Check("empty", !requests.Pop(tag, out, len));
  return ok;
}

bool TestServesTwoRequestsPerTick() {
  uint8_t storage[64];
  LinkItemQueue requests(storage, sizeof(storage));
  const uint8_t one[] = {1};
  const uint8_t two[] = {2};
  const uint8_t three[] = {3};
  bool ok = true;
  ok &= Check("request cap is 2", LINK_REQUESTS_PER_TICK == 2);
  ok &= Check("push three requests", requests.Push(0, one, 1) && requests.Push(1, two, 1) && requests.Push(0, three, 1));

  struct Served {
    uint8_t port;
    uint8_t first;
  };
  std::vector<Served> served;
  uint8_t scratch[LINK_MAX_ITEM_LEN];
  auto handle = [&served](uint8_t port, const uint8_t* payload, size_t) { served.push_back({port, payload[0]}); };

  const size_t tick1 = LinkServeQueued(requests, 2, scratch, handle);
  ok &= Check("tick 1 serves the two oldest", tick1 == 2 && served.size() == 2 && served[0].port == 0 &&
                                                  served[0].first == 1 && served[1].port == 1 && served[1].first == 2);
  const size_t tick2 = LinkServeQueued(requests, LINK_REQUESTS_PER_TICK, scratch, handle);
  ok &= Check("tick 2 serves the third", tick2 == 1 && served.size() == 3 && served[2].port == 0 && served[2].first == 3);
  ok &= Check("tick 3 has nothing left", LinkServeQueued(requests, LINK_REQUESTS_PER_TICK, scratch, handle) == 0);
  return ok;
}

}  // namespace

int main() {
  bool ok = true;
  ok &= TestWholeFrameUnderPartialAvailability();
  ok &= TestDropCountedOnFullQueue();
  ok &= TestSeqIsPerPort();
  ok &= TestUsbPrefixWaitsForRoom();
  ok &= TestRequestQueueKeepsPortAndPayload();
  ok &= TestServesTwoRequestsPerTick();
  return ok ? 0 : 1;
}
