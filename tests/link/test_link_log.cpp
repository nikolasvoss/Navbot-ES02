// Logging::message through the Log frames that reach the two ports. This file stands in for the firmware's
// LinkPublishLog (link_uart.cpp): the same LinkSendLog sends to two LinkPort queues, one for UART1 (no prefix) and one for
// USB (a 0x00 before each frame). Each port's bytes are then read back the way the ros2_pc reads them.

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "Logging.h"
#include "link_frame.h"
#include "link_log.h"
#include "link_tx.h"

namespace {

typedef std::vector<uint8_t> Bytes;

bool Check(const char* name, bool ok) {
  std::printf("%s: %s\n", name, ok ? "ok" : "FAIL");
  return ok;
}

// Collects what a port's writer puts on the wire.
class BytesSink : public LinkByteSink {
 public:
  int AvailableForWrite() override { return 1 << 20; }
  void Write(const uint8_t* data, size_t len) override { bytes.insert(bytes.end(), data, data + len); }
  Bytes bytes;
};

struct Ports {
  Ports() : uart(uartStorage, sizeof(uartStorage), false), usb(usbStorage, sizeof(usbStorage), true) {}
  uint8_t uartStorage[1024];
  uint8_t usbStorage[2560];
  LinkPort uart;
  LinkPort usb;
};

class PortsSink : public LinkMessageSink {
 public:
  explicit PortsSink(Ports& ports) : ports_(ports) {}
  void Send(uint8_t port, navbot_link_v1_Message& msg) override {
    LinkPortSend(port == kLinkPortUart ? ports_.uart : ports_.usb, msg, 0);
  }

 private:
  Ports& ports_;
};

PortsSink* activeSink = nullptr;

}  // namespace

// The firmware's LinkPublishLog, with the test's ports in place of UART1 and USB.
void LinkPublishLog(Logging::Level level, const char* tag, const char* text) {
  LinkSendLog(LinkLogLevel(level), tag, text, *activeSink);
}

namespace {

// Writes everything queued on a port, as the port's writer task does, and returns the bytes.
Bytes Drain(LinkPort& port) {
  BytesSink sink;
  port.writer.Pump(port.queue, sink);
  return sink.bytes;
}

class Collector : public LinkRxSink {
 public:
  void OnPayload(const uint8_t* payload, size_t len) override {
    navbot_link_v1_Message message = navbot_link_v1_Message_init_zero;
    if (LinkDecodeMessage(payload, len, message)) messages.push_back(message);
  }
  std::vector<navbot_link_v1_Message> messages;
};

std::vector<navbot_link_v1_Message> ReadFrames(const Bytes& bytes) {
  Collector collector;
  LinkRxDecoder decoder;
  decoder.Feed(bytes.data(), bytes.size(), collector);
  return collector.messages;
}

bool IsLog(const navbot_link_v1_Message& message, navbot_link_v1_Log_Level level, const char* tag, const char* text) {
  return message.which_body == navbot_link_v1_Message_log_tag && message.body.log.level == level &&
         std::strcmp(message.body.log.tag, tag) == 0 && std::strcmp(message.body.log.text, text) == 0;
}

bool TestBothPortsGetTheSameLogAndUsbPrefixesEachFrame() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  Logging::message(Logging::Level::Warning, "WHEEL", "peak=%.1frad_s limit=%d", 38.5, 60);
  const Bytes uart = Drain(ports.uart);
  const Bytes usb = Drain(ports.usb);
  const std::vector<navbot_link_v1_Message> uartFrames = ReadFrames(uart);
  const std::vector<navbot_link_v1_Message> usbFrames = ReadFrames(usb);
  const bool content = uartFrames.size() == 1 && usbFrames.size() == 1 &&
                       IsLog(uartFrames[0], navbot_link_v1_Log_Level_WARN, "WHEEL", "peak=38.5rad_s limit=60") &&
                       IsLog(usbFrames[0], navbot_link_v1_Log_Level_WARN, "WHEEL", "peak=38.5rad_s limit=60");
  // The USB bytes are the UART bytes with one 0x00 in front: the receiver starts a new frame at each 0x00.
  const bool prefix = !uart.empty() && usb.size() == uart.size() + 1 && usb[0] == 0x00 &&
                      std::memcmp(usb.data() + 1, uart.data(), uart.size()) == 0 && uart[0] != 0x00;
  return Check("both ports get the level, tag and text; USB has one 0x00 before the frame",
               content && prefix);
}

bool TestEveryLevelMapsToItsWireLevel() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  Logging::message(Logging::Level::Debug, "L", "d");
  Logging::message(Logging::Level::Info, "L", "i");
  Logging::message(Logging::Level::Warning, "L", "w");
  Logging::message(Logging::Level::Error, "L", "e");
  const std::vector<navbot_link_v1_Message> frames = ReadFrames(Drain(ports.uart));
  return Check("levels map to DEBUG, INFO, WARN and ERROR",
               frames.size() == 4 && IsLog(frames[0], navbot_link_v1_Log_Level_DEBUG, "L", "d") &&
                   IsLog(frames[1], navbot_link_v1_Log_Level_INFO, "L", "i") &&
                   IsLog(frames[2], navbot_link_v1_Log_Level_WARN, "L", "w") &&
                   IsLog(frames[3], navbot_link_v1_Log_Level_ERROR, "L", "e"));
}

bool TestTextOf96CharactersIsKeptOnBothPorts() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  const std::string text(96, 'a');
  Logging::message(Logging::Level::Info, "EDGE", "%s", text.c_str());
  const std::vector<navbot_link_v1_Message> uartFrames = ReadFrames(Drain(ports.uart));
  const std::vector<navbot_link_v1_Message> usbFrames = ReadFrames(Drain(ports.usb));
  return Check("96 characters pass uncut on both ports", uartFrames.size() == 1 && usbFrames.size() == 1 &&
                                                             IsLog(uartFrames[0], navbot_link_v1_Log_Level_INFO, "EDGE", text.c_str()) &&
                                                             IsLog(usbFrames[0], navbot_link_v1_Log_Level_INFO, "EDGE", text.c_str()));
}

bool TestLongTextIsCutTo96Characters() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  // Each character is distinct, so a cut at the wrong place shows.
  std::string text;
  for (int i = 0; i < 200; ++i) text.push_back(static_cast<char>('a' + i % 26));
  Logging::message(Logging::Level::Error, "LONG", "%s", text.c_str());
  const std::vector<navbot_link_v1_Message> frames = ReadFrames(Drain(ports.uart));
  return Check("text longer than 96 characters is cut to the first 96",
               frames.size() == 1 && IsLog(frames[0], navbot_link_v1_Log_Level_ERROR, "LONG", text.substr(0, 96).c_str()));
}

bool TestTagIsCutTo16Characters() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  Logging::message(Logging::Level::Info, "abcdefghijklmnopqrstuvwxyz", "tag");
  const std::vector<navbot_link_v1_Message> frames = ReadFrames(Drain(ports.uart));
  return Check("tag is cut to its 16 characters", frames.size() == 1 && IsLog(frames[0], navbot_link_v1_Log_Level_INFO,
                                                                              "abcdefghijklmnop", "tag"));
}

bool TestSeqCountsPerPort() {
  Ports ports;
  PortsSink sink(ports);
  activeSink = &sink;
  Logging::message(Logging::Level::Info, "S", "first");
  Logging::message(Logging::Level::Info, "S", "second");
  const std::vector<navbot_link_v1_Message> uartFrames = ReadFrames(Drain(ports.uart));
  const std::vector<navbot_link_v1_Message> usbFrames = ReadFrames(Drain(ports.usb));
  return Check("each port numbers its own frames from 1",
               uartFrames.size() == 2 && usbFrames.size() == 2 && uartFrames[0].seq == 1 && uartFrames[1].seq == 2 &&
                   usbFrames[0].seq == 1 && usbFrames[1].seq == 2);
}

}  // namespace

int main() {
  bool ok = true;
  ok &= TestBothPortsGetTheSameLogAndUsbPrefixesEachFrame();
  ok &= TestEveryLevelMapsToItsWireLevel();
  ok &= TestTextOf96CharactersIsKeptOnBothPorts();
  ok &= TestLongTextIsCutTo96Characters();
  ok &= TestTagIsCutTo16Characters();
  ok &= TestSeqCountsPerPort();
  return ok ? 0 : 1;
}
