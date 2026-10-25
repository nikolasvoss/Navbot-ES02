#include "link_uart.h"

#include <soc/usb_serial_jtag_struct.h>

#include "Logging.h"
#include "link_frame.h"

namespace {

constexpr float kStandardGravityMps2 = 9.80665f;

uint32_t linkSequence = 0;
uint32_t linkDroppedFrames = 0;
uint32_t linkSentFrames = 0;
constexpr uint32_t kLinkStatsEverySends = 100;  // 2 s at 50 Hz

void LinkSend(navbot_link_v1_Message& msg) {
  msg.seq = ++linkSequence;
  msg.t_ms = millis();

  uint8_t frame[LINK_MAX_FRAME_LEN];
  const size_t frameLen = LinkEncodeFrame(msg, frame, sizeof(frame));
  if (frameLen == 0 || Serial1.availableForWrite() < static_cast<int>(frameLen)) {
    ++linkDroppedFrames;
    return;
  }
  Serial1.write(frame, frameLen);
  ++linkSentFrames;
}

}  // namespace

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
  Logging::message(Logging::Level::Info, "Link", "UART1 %d baud RX=%d TX=%d usbPad=%d", LINK_UART_BAUD,
                   board::kLinkUartRxPin, board::kLinkUartTxPin, static_cast<int>(USB_SERIAL_JTAG.conf0.usb_pad_enable));
}

void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot) {
  static uint32_t publishCount = 0;
  if (++publishCount % kLinkStatsEverySends == 0)
    Logging::message(Logging::Level::Info, "Link", "sent=%u dropped=%u txFree=%d", static_cast<unsigned>(linkSentFrames),
                     static_cast<unsigned>(linkDroppedFrames), Serial1.availableForWrite());
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
  LinkSend(sample);
}
