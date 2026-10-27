#include "link_uart.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <soc/usb_serial_jtag_struct.h>

#include "Logging.h"
#include "link_frame.h"
#include "link_tx.h"
#include "usb_out.h"

namespace {

constexpr float kStandardGravityMps2 = 9.80665f;
constexpr uint32_t kLinkStatsEverySends = 100;  // 2 s at 50 Hz
constexpr size_t kUartQueueBytes = 1024;
// Holds a 576-byte logger line plus the frames that arrive while the CH340 drains it.
constexpr size_t kUsbQueueBytes = 2560;
constexpr uint32_t kWriterStackBytes = 3072;
constexpr UBaseType_t kWriterPriority = 2;  // above the logger sender (1) so a long text line does not hold up frames
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

void wake(PortRuntime& runtime) {
  if (runtime.task) xTaskNotifyGive(runtime.task);
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

bool LinkSend(navbot_link_v1_Message& msg) {
  const bool queued = LinkPortSend(uart.port, msg, millis());
  if (queued) wake(uart);
  return queued;
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

void LinkUartInit() {
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
  Serial1.begin(LINK_UART_BAUD, SERIAL_8N1, board::kLinkUartRxPin, board::kLinkUartTxPin);
  startWriter(uart, "uart1Tx");
  Logging::message(Logging::Level::Info, "Link", "UART1 %d baud RX=%d TX=%d usbPad=%d", LINK_UART_BAUD,
                   board::kLinkUartRxPin, board::kLinkUartTxPin, static_cast<int>(USB_SERIAL_JTAG.conf0.usb_pad_enable));
}

void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot) {
  static uint32_t publishCount = 0;
  if (++publishCount % kLinkStatsEverySends == 0)
    Logging::message(Logging::Level::Info, "Link", "seq=%u dropped=%u txFree=%d", static_cast<unsigned>(uart.port.seq),
                     static_cast<unsigned>(uart.port.queue.dropped()), Serial1.availableForWrite());
  navbot_link_v1_Message odom = navbot_link_v1_Message_init_zero;
  odom.which_body = navbot_link_v1_Message_odom_tag;
  odom.body.odom.t_us = sampleTimeUs;
  odom.body.odom.left_angle_rad = leftWheelAngleRad;
  odom.body.odom.right_angle_rad = rightWheelAngleRad;
  LinkSend(odom);

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
  LinkSend(sample);
}
