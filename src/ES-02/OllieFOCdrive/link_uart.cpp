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

void LinkPublishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad, const imu::Snapshot& imuSnapshot) {
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
