#include "link_uart.h"

#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/usb_serial_jtag_struct.h>

#include "Logging.h"
#include "link_endpoint.h"
#include "link_frame.h"
#include "link_log.h"
#include "link_tx.h"
#include "usb_out.h"

namespace {

constexpr float kStandardGravityMps2 = 9.80665f;
constexpr uint32_t kLinkStatsEverySends = 100;  // 2 s at 50 Hz
constexpr size_t kUartQueueBytes = 1024;
// Holds a 576-byte logger line plus the frames that arrive while the CH340 drains it.
constexpr size_t kUsbQueueBytes = 2560;
constexpr uint32_t kWriterStackBytes = 4096;
constexpr UBaseType_t kWriterPriority = 2;  // above the logger sender (1) so a long text line does not hold up frames
constexpr uint32_t kReaderStackBytes = 4096;
constexpr UBaseType_t kReaderPriority = 2;
constexpr size_t kRequestQueueBytes = 1536;  // a few requests of up to 252 bytes
constexpr size_t kCommandQueueBytes = 256;   // typed letter commands are short
constexpr size_t kUartRxBufferBytes = 1024;  // about 5 ms of line at 2000000 baud; the reader polls every tick (1 ms)
constexpr TickType_t kBlockingPushTimeoutTicks = 200;

class SpinLock : public LinkLock {
 public:
  void Lock() override { portENTER_CRITICAL(&mux_); }
  void Unlock() override { portEXIT_CRITICAL(&mux_); }

 private:
  portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;
};

class PrintSink : public LinkByteSink {
 public:
  explicit PrintSink(Print& print) : print_(print) {}
  int AvailableForWrite() override { return print_.availableForWrite(); }
  void Write(const uint8_t* data, size_t len) override { print_.write(data, len); }

 private:
  Print& print_;
};

// The lock is declared first because the port keeps a pointer to it.
struct PortRuntime {
  PortRuntime(uint8_t* storage, size_t capacity, bool prefixFrames, Print& print)
      : port(storage, capacity, prefixFrames, &lock), sink(print) {}
  SpinLock lock;
  LinkPort port;
  PrintSink sink;
  TaskHandle_t task = nullptr;
};

uint8_t uartStorage[kUartQueueBytes];
uint8_t usbStorage[kUsbQueueBytes];
PortRuntime uart(uartStorage, sizeof(uartStorage), false, Serial1);
PortRuntime usb(usbStorage, sizeof(usbStorage), true, Serial);

PortRuntime& runtimeOf(uint8_t port) { return port == kLinkPortUart ? uart : usb; }

void wake(PortRuntime& runtime) {
  if (runtime.task) xTaskNotifyGive(runtime.task);
}

// Requests from both ports, tagged with the port, for the control loop to apply at the start of a tick.
SpinLock requestLock;
uint8_t requestStorage[kRequestQueueBytes];
LinkItemQueue requests(requestStorage, sizeof(requestStorage), &requestLock);

// Printable lines from the USB port for the Commander, so the letter commands keep working next to frames.
SpinLock commandLock;
uint8_t commandStorage[kCommandQueueBytes];
LinkItemQueue commandLines(commandStorage, sizeof(commandStorage), &commandLock);

// One per port: decodes what the port receives and queues it. Runs in its own task and touches no robot state.
struct Reader : LinkRxSink {
  Reader(uint8_t portNumber, Stream& serialPort, bool passTextLines)
      : port(portNumber), serial(serialPort), decoder(passTextLines) {}
  void OnPayload(const uint8_t* payload, size_t len) override { requests.PushOrDrop(port, payload, len); }
  void OnTextLine(const uint8_t* line, size_t len) override { commandLines.PushOrDrop(0, line, len); }
  uint8_t port;
  Stream& serial;
  LinkRxDecoder decoder;
};

Reader uartReader(kLinkPortUart, Serial1, false);
Reader usbReader(kLinkPortUsb, Serial, true);

void readerTask(void* arg) {
  Reader& reader = *static_cast<Reader*>(arg);
  uint8_t bytes[64];
  for (;;) {
    int available = reader.serial.available();
    if (available <= 0) {
      vTaskDelay(1);
      continue;
    }
    const size_t n = reader.serial.readBytes(bytes, available < static_cast<int>(sizeof(bytes)) ? available : sizeof(bytes));
    reader.decoder.Feed(bytes, n, reader);
  }
}

void startReader(Reader& reader, const char* name) {
  xTaskCreatePinnedToCore(readerTask, name, kReaderStackBytes, &reader, kReaderPriority, nullptr, 0);
}

// The only code that writes to a UART. It sleeps one tick when the queue is empty or the UART buffer is too full for
// the next frame.
void writerTask(void* arg) {
  PortRuntime& runtime = *static_cast<PortRuntime*>(arg);
  for (;;) {
    if (runtime.port.writer.Pump(runtime.port.queue, runtime.sink) == 0) ulTaskNotifyTake(pdTRUE, 1);
  }
}

void startWriter(PortRuntime& runtime, const char* name) {
  xTaskCreatePinnedToCore(writerTask, name, kWriterStackBytes, &runtime, kWriterPriority, &runtime.task, 0);
}

class UsbPrint : public Print {
 public:
  size_t write(uint8_t value) override { return write(&value, 1); }
  size_t write(const uint8_t* data, size_t len) override {
    for (size_t offset = 0; offset < len; offset += LINK_MAX_ITEM_LEN) {
      const size_t chunk = len - offset < LINK_MAX_ITEM_LEN ? len - offset : LINK_MAX_ITEM_LEN;
      usb.port.queue.PushOrDrop(kLinkItemRaw, data + offset, chunk);
    }
    wake(usb);
    return len;  // a drop is counted, not retried: the callers are the control loop and setup
  }
} usbPrint;

bool sendOn(PortRuntime& runtime, navbot_link_v1_Message& msg) {
  const bool queued = LinkPortSend(runtime.port, msg, millis());
  if (queued) wake(runtime);
  return queued;
}

class PortSink : public LinkMessageSink {
 public:
  void Send(uint8_t port, navbot_link_v1_Message& msg) override { sendOn(runtimeOf(port), msg); }
} portSink;

const LinkPortBudget portBudgets[kLinkPortCount] = {
    {LINK_UART_BAUD / 10, LINK_ODOMETRY_RESERVED_BYTES_PER_SEC},
    {LINK_USB_BAUD / 10, 0},
};

LinkEndpoint* endpoint = nullptr;

navbot_link_v1_Log_Level linkLevel(Logging::Level level) {
  switch (level) {
    case Logging::Level::Debug:
      return navbot_link_v1_Log_Level_DEBUG;
    case Logging::Level::Info:
      return navbot_link_v1_Log_Level_INFO;
    case Logging::Level::Warning:
      return navbot_link_v1_Log_Level_WARN;
    case Logging::Level::Error:
      return navbot_link_v1_Log_Level_ERROR;
  }
  return navbot_link_v1_Log_Level_INFO;
}

// The command queue holds one line per item; the Commander reads it a byte at a time.
uint8_t commandLine[LINK_MAX_TEXT_LINE];
size_t commandLineLen = 0;
size_t commandLinePos = 0;

bool commandBytesLeft() {
  if (commandLinePos < commandLineLen) return true;
  uint8_t tag = 0;
  commandLinePos = 0;
  commandLineLen = 0;
  size_t len = 0;
  if (!commandLines.Pop(tag, commandLine, len)) return false;
  commandLineLen = len;
  return true;
}

}  // namespace

Print& LinkUsbPrint() { return usbPrint; }

void LinkUsbInit() { startWriter(usb, "usbTx"); }

size_t UsbWriteBlocking(const uint8_t* data, size_t len) {
  for (TickType_t tries = 0; tries < kBlockingPushTimeoutTicks; ++tries) {
    if (usb.port.queue.Push(kLinkItemRaw, data, len)) {
      wake(usb);
      return len;
    }
    vTaskDelay(1);
  }
  usb.port.queue.NoteDrop();
  return 0;
}

void LinkPublishLog(Logging::Level level, const char* tag, const char* text) {
  navbot_link_v1_Message message = navbot_link_v1_Message_init_zero;
  message.which_body = navbot_link_v1_Message_log_tag;
  message.body.log.level = linkLevel(level);
  strncpy(message.body.log.tag, tag, sizeof(message.body.log.tag) - 1);
  strncpy(message.body.log.text, text, sizeof(message.body.log.text) - 1);
  // USB stays free of Log frames: between K CSV rows they corrupt the capture until step 5 deletes K.
  for (uint8_t port = 0; port < kLinkPortCount; ++port)
    if (LinkLogReachesPort(port)) sendOn(runtimeOf(port), message);
}

int LinkUsbCommandAvailable() { return commandBytesLeft() ? static_cast<int>(commandLineLen - commandLinePos) : 0; }

int LinkUsbCommandRead() { return commandBytesLeft() ? commandLine[commandLinePos++] : -1; }

int LinkUsbCommandPeek() { return commandBytesLeft() ? commandLine[commandLinePos] : -1; }

void LinkServiceRequests(uint32_t tickStartUs, bool balanceRequested) {
  if (!endpoint) return;
  static uint8_t payload[LINK_MAX_ITEM_LEN];
  auto handle = [tickStartUs, balanceRequested](uint8_t port, const uint8_t* data, size_t len) {
    endpoint->Handle(port, data, len, tickStartUs, balanceRequested);
  };
  LinkServeQueued(requests, LINK_REQUESTS_PER_TICK, payload, handle);
}

void LinkSampleSignals(uint32_t tickStartUs) {
  if (endpoint) endpoint->Sample(tickStartUs);
}

void LinkUartInit(const SignalEntry* table, size_t count, const char* build) {
  // GPIO19 and GPIO20 are the USB Serial/JTAG D- and D+ pads. They power up in USB mode (pad function and D+ pull-up
  // on), which keeps the UART signals off the pins. Nothing here uses native USB, so release the pads first.
  USB_SERIAL_JTAG.conf0.usb_pad_enable = 0;
  USB_SERIAL_JTAG.conf0.dp_pullup = 0;
  // Pin check before UART1 takes the pin: drive the TX pin low for 50 ms and read it back. A pad that does not follow
  // shows up in the log and as a missing low pulse on a logic analyzer.
  pinMode(board::kLinkUartTxPin, OUTPUT);
  digitalWrite(board::kLinkUartTxPin, LOW);
  delay(50);
  const int lowReadback = digitalRead(board::kLinkUartTxPin);
  digitalWrite(board::kLinkUartTxPin, HIGH);
  delay(1);
  const int highReadback = digitalRead(board::kLinkUartTxPin);
  Logging::message(Logging::Level::Info, "Link", "GPIO%d drive low reads %d, high reads %d", board::kLinkUartTxPin,
                   lowReadback, highReadback);
  Serial1.setTxBufferSize(512);
  Serial1.setRxBufferSize(kUartRxBufferBytes);
  Serial1.begin(LINK_UART_BAUD, SERIAL_8N1, board::kLinkUartRxPin, board::kLinkUartTxPin);
  startWriter(uart, "uart1Tx");
  static LinkEndpoint instance(table, count, build, portBudgets, portSink);
  endpoint = &instance;
  startReader(uartReader, "uart1Rx");
  startReader(usbReader, "usbRx");
  Logging::message(Logging::Level::Info, "Link", "UART1 %d baud RX=%d TX=%d usbPad=%d %u entries", LINK_UART_BAUD,
                   board::kLinkUartRxPin, board::kLinkUartTxPin, static_cast<int>(USB_SERIAL_JTAG.conf0.usb_pad_enable),
                   static_cast<unsigned>(count));
}

void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot) {
  static uint32_t publishCount = 0;
  if (++publishCount % kLinkStatsEverySends == 0)
    Logging::message(Logging::Level::Info, "Link", "seq=%u dropped=%u txFree=%d rxBad=%u/%u reqDropped=%u undecoded=%u",
                     static_cast<unsigned>(uart.port.seq), static_cast<unsigned>(uart.port.queue.dropped()),
                     Serial1.availableForWrite(), static_cast<unsigned>(uartReader.decoder.badFrames()),
                     static_cast<unsigned>(usbReader.decoder.badFrames()), static_cast<unsigned>(requests.dropped()),
                     static_cast<unsigned>(endpoint ? endpoint->undecodable() : 0));
  navbot_link_v1_Message odom = navbot_link_v1_Message_init_zero;
  odom.which_body = navbot_link_v1_Message_odom_tag;
  odom.body.odom.t_us = sampleTimeUs;
  odom.body.odom.left_angle_rad = leftWheelAngleRad;
  odom.body.odom.right_angle_rad = rightWheelAngleRad;
  sendOn(uart, odom);

  if (!imuSnapshot.valid) return;
  navbot_link_v1_Message sample = navbot_link_v1_Message_init_zero;
  sample.which_body = navbot_link_v1_Message_imu_tag;
  sample.body.imu.ax = imuSnapshot.filteredAccelerationG.x * kStandardGravityMps2;
  sample.body.imu.ay = imuSnapshot.filteredAccelerationG.y * kStandardGravityMps2;
  sample.body.imu.az = imuSnapshot.filteredAccelerationG.z * kStandardGravityMps2;
  sample.body.imu.gx = imuSnapshot.filteredAngularVelocityRadPerSec.x;
  sample.body.imu.gy = imuSnapshot.filteredAngularVelocityRadPerSec.y;
  sample.body.imu.gz = imuSnapshot.filteredAngularVelocityRadPerSec.z;
  sample.body.imu.roll_rad = imuSnapshot.attitudeDegrees.roll * DEG_TO_RAD;
  sample.body.imu.pitch_rad = imuSnapshot.attitudeDegrees.pitch * DEG_TO_RAD;
  sample.body.imu.yaw_rad = imuSnapshot.attitudeDegrees.yaw * DEG_TO_RAD;
  sample.body.imu.t_us = imuSnapshot.readTimeUs;
  sendOn(uart, sample);
}
