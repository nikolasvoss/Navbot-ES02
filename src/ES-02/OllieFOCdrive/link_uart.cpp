#include "link_uart.h"

#include "link_frame.h"

namespace {

constexpr float kStandardGravityMps2 = 9.80665f;

uint32_t linkSequence = 0;
uint32_t linkDroppedFrames = 0;

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
}

}  // namespace

void LinkUartInit() {
  Serial1.setTxBufferSize(512);
  Serial1.begin(LINK_UART_BAUD, SERIAL_8N1, LINK_UART_RX_PIN, LINK_UART_TX_PIN);
}

void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imu) {
  navbot_link_v1_Message odom = navbot_link_v1_Message_init_zero;
  odom.which_body = navbot_link_v1_Message_odom_tag;
  odom.body.odom.t_us = sampleTimeUs;
  odom.body.odom.left_angle_rad = leftWheelAngleRad;
  odom.body.odom.right_angle_rad = rightWheelAngleRad;
  LinkSend(odom);

  if (!imu.valid) return;
  navbot_link_v1_Message sample = navbot_link_v1_Message_init_zero;
  sample.which_body = navbot_link_v1_Message_imu_tag;
  sample.body.imu.ax = imu.filteredAccelerationG.x * kStandardGravityMps2;
  sample.body.imu.ay = imu.filteredAccelerationG.y * kStandardGravityMps2;
  sample.body.imu.az = imu.filteredAccelerationG.z * kStandardGravityMps2;
  sample.body.imu.gx = imu.filteredAngularVelocityRadPerSec.x;
  sample.body.imu.gy = imu.filteredAngularVelocityRadPerSec.y;
  sample.body.imu.gz = imu.filteredAngularVelocityRadPerSec.z;
  sample.body.imu.roll_rad = imu.attitudeDegrees.roll * DEG_TO_RAD;
  sample.body.imu.pitch_rad = imu.attitudeDegrees.pitch * DEG_TO_RAD;
  sample.body.imu.yaw_rad = imu.attitudeDegrees.yaw * DEG_TO_RAD;
  LinkSend(sample);
}

uint32_t LinkDroppedFrames() {
  return linkDroppedFrames;
}
