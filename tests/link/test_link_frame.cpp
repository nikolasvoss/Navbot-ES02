#include "link_frame.h"

#include <cstdio>
#include <cstring>

// Expected frames were derived independently: protobuf (python) serializes the
// same Message, navbot_link_bridge.framing.encode_frame frames it, and
// navbot_link_bridge.framing.FrameDecoder decodes the result back to the same payload.
namespace {

const uint8_t kOdometryFrame[] = {
    0x0B, 0x08, 0x01, 0x10, 0x02, 0x72, 0x15, 0x08, 0xE8, 0x07, 0x11, 0x01, 0x01, 0x01, 0x01, 0x01,
    0x04, 0xF8, 0x3F, 0x19, 0x01, 0x01, 0x01, 0x01, 0x01, 0x05, 0xD0, 0xBF, 0xC8, 0xD4, 0x00,
};

const uint8_t kImuFrame[] = {
    0x0D, 0x08, 0x03, 0x10, 0x04, 0x52, 0x0A, 0x0D, 0x0A, 0xE8, 0x1C, 0x41, 0x35, 0x01, 0x01, 0x04,
    0x3F, 0xD2, 0x1E, 0x00,
};

bool CheckFrame(const char* name, const uint8_t* actual, size_t actualLen,
                const uint8_t* expected, size_t expectedLen) {
  if (actualLen != expectedLen || std::memcmp(actual, expected, expectedLen) != 0) {
    std::printf("%s: FAIL (length %zu, expected %zu)\n", name, actualLen, expectedLen);
    return false;
  }
  std::printf("%s: ok (%zu bytes)\n", name, actualLen);
  return true;
}

}  // namespace

int main() {
  bool ok = true;
  uint8_t frame[LINK_MAX_FRAME_LEN];

  navbot_link_v1_Message odometry = navbot_link_v1_Message_init_zero;
  odometry.seq = 1;
  odometry.t_ms = 2;
  odometry.which_body = navbot_link_v1_Message_odom_tag;
  odometry.body.odom.t_us = 1000;
  odometry.body.odom.left_angle_rad = 1.5;
  odometry.body.odom.right_angle_rad = -0.25;

  size_t len = LinkEncodeFrame(odometry, frame, sizeof(frame));
  ok &= CheckFrame("odometry", frame, len, kOdometryFrame, sizeof(kOdometryFrame));

  navbot_link_v1_Message imu = navbot_link_v1_Message_init_zero;
  imu.seq = 3;
  imu.t_ms = 4;
  imu.which_body = navbot_link_v1_Message_imu_tag;
  imu.body.imu.ax = 9.80665f;
  imu.body.imu.gz = 0.5f;

  len = LinkEncodeFrame(imu, frame, sizeof(frame));
  ok &= CheckFrame("imu", frame, len, kImuFrame, sizeof(kImuFrame));

  const size_t tooSmall = LinkEncodeFrame(odometry, frame, sizeof(kOdometryFrame) - 1);
  if (tooSmall != 0) {
    std::printf("capacity too small: FAIL (returned %zu)\n", tooSmall);
    ok = false;
  } else {
    std::printf("capacity too small: ok (returned 0)\n");
  }

  return ok ? 0 : 1;
}
