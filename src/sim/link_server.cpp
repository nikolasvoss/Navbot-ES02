// The live mode of navbot_sim: the NavBot link served to one TCP client while the robot runs on the wall clock (plan,
// Model: Command line; T5). The firmware's link code runs unchanged (LinkEndpoint, its subscriptions, frames and log
// frames). This file replaces link_uart.cpp: a socket for the UART and the requests and rows follow the control tick
// as the sketch's linkPassStart and linkPassEnd do.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <memory>
#include <string>
#include <thread>

#include <Arduino.h>

#include "ControlTick.h"
#include "MathConstants.h"
#include "WheelBoundary.h"
#include "imu.h"
#include "link_endpoint.h"
#include "link_frame.h"
#include "link_log.h"
#include "link_schedule.h"
#include "link_tx.h"
#include "link_uart.h"
#include "sim.h"


namespace sim {
namespace {

constexpr uint8_t kClientPort = kLinkPortUart;  // the client takes the port of UART1, the ros2_pc's
constexpr const char kBuildId[] = "sim";
constexpr double kStandardGravityMps2 = 9.80665;
constexpr size_t kPortQueueBytes = 2048;
constexpr size_t kRequestQueueBytes = 1536;  // a few requests of up to 252 bytes, as on the robot
constexpr size_t kSocketRoomBytes = 4096;    // the socket takes a whole frame at once
constexpr time_t kSendTimeoutSec = 1;        // a client that stops reading ends its connection, not the simulation

const LinkPortBudget kBudgets[kLinkPortCount] = {
    {LINK_UART_BAUD / 10, LINK_ODOMETRY_RESERVED_BYTES_PER_SEC},
    {LINK_USB_BAUD / 10, 0},
};

const char *acceptCommand(float speedMps, float turnRadPerSec) {
  return ControlTick::acceptLinkCommand(control, speedMps, turnRadPerSec, hal().nowUs);
}

// Writes frames to the client's socket. After a failed or stalled send the client is broken: nothing more is written
// to it, and the main loop closes it between passes, since the pass that failed is still running.
class SocketBytes : public LinkByteSink {
 public:
  explicit SocketBytes(int fd) : fd_(fd) {}

  int AvailableForWrite() override { return broken_ ? 0 : static_cast<int>(kSocketRoomBytes); }

  void Write(const uint8_t *data, size_t len) override {
    while (!broken_ && len > 0) {
      const ssize_t n = send(fd_, data, len, MSG_NOSIGNAL);
      if (n < 0 && errno == EINTR) continue;
      if (n <= 0) {
        broken_ = true;
        return;
      }
      data += n;
      len -= static_cast<size_t>(n);
    }
  }

  bool broken() const { return broken_; }

 private:
  int fd_;
  bool broken_ = false;
};

// One connected client with its own endpoint, so its subscriptions end with it.
class Client : public LinkMessageSink, public LinkRxSink {
 public:
  explicit Client(int fd)
      : fd_(fd),
        bytes_(fd),
        port_(portStorage_, sizeof(portStorage_), false),
        requests_(requestStorage_, sizeof(requestStorage_)),
        endpoint_(signalTableEntries(), signalTableCount(), kBuildId, kBudgets, *this) {
    endpoint_.SetCommandHandler(acceptCommand);
  }

  ~Client() { close(fd_); }
  Client(const Client &) = delete;
  Client &operator=(const Client &) = delete;

  int fd() const { return fd_; }
  bool broken() const { return bytes_.broken(); }

  // The client's bytes: whole frames go to OnPayload.
  void feed(const uint8_t *data, size_t len) { decoder_.Feed(data, len, *this); }

  // LinkMessageSink: replies, rows and log frames. The robot's link has two ports; this one is the client's.
  void Send(uint8_t port, navbot_link_v1_Message &msg) override {
    if (port != kClientPort) return;
    LinkPortSend(port_, msg, millis());
    port_.writer.Pump(port_.queue, bytes_);
  }

  // LinkRxSink: a request, answered at the next control tick.
  void OnPayload(const uint8_t *payload, size_t len) override { requests_.PushOrDrop(kClientPort, payload, len); }

  // Control tick start: the requests queued since the last tick (LinkServiceRequests on the robot).
  void serveRequests(uint32_t tickStartUs, bool balanceRequested) {
    auto handle = [this, tickStartUs, balanceRequested](uint8_t port, const uint8_t *data, size_t len) {
      endpoint_.Handle(port, data, len, tickStartUs, balanceRequested);
    };
    LinkServeQueued(requests_, LINK_REQUESTS_PER_TICK, scratch_, handle);
  }

  // Control tick end: the odometry when it is due, then the rows that are due (LinkPublishOdometry and
  // LinkSampleSignals on the robot).
  void endTick(uint32_t tickStartUs) {
    if (control.wheelsValid && LinkScheduleDue(odometrySchedule_, tickStartUs, LINK_ODOMETRY_PERIOD_US)) {
      publishOdometry(control.wheelReadTimeUs, WheelBoundary::kLeftMotorSign * motor1.shaftAngle(),
                      WheelBoundary::kRightMotorSign * motor2.shaftAngle(), imu::latest());
    }
    endpoint_.Sample(tickStartUs);
  }

 private:
  // The WheelOdometry and Imu frames, as link_uart.cpp sends them.
  void publishOdometry(uint32_t sampleTimeUs, double leftWheelAngleRad, double rightWheelAngleRad,
                       const imu::Snapshot &imuSnapshot) {
    navbot_link_v1_Message odom = navbot_link_v1_Message_init_zero;
    odom.which_body = navbot_link_v1_Message_odom_tag;
    odom.body.odom.t_us = sampleTimeUs;
    odom.body.odom.left_angle_rad = leftWheelAngleRad;
    odom.body.odom.right_angle_rad = rightWheelAngleRad;
    Send(kClientPort, odom);

    if (!imuSnapshot.valid) return;
    navbot_link_v1_Message sample = navbot_link_v1_Message_init_zero;
    sample.which_body = navbot_link_v1_Message_imu_tag;
    sample.body.imu.ax = imuSnapshot.filteredAccelerationG.x * kStandardGravityMps2;
    sample.body.imu.ay = imuSnapshot.filteredAccelerationG.y * kStandardGravityMps2;
    sample.body.imu.az = imuSnapshot.filteredAccelerationG.z * kStandardGravityMps2;
    sample.body.imu.gx = imuSnapshot.filteredAngularVelocityRadPerSec.x;
    sample.body.imu.gy = imuSnapshot.filteredAngularVelocityRadPerSec.y;
    sample.body.imu.gz = imuSnapshot.filteredAngularVelocityRadPerSec.z;
    sample.body.imu.roll_rad = imuSnapshot.attitudeDegrees.roll * (kPi / 180.0);
    sample.body.imu.pitch_rad = imuSnapshot.attitudeDegrees.pitch * (kPi / 180.0);
    sample.body.imu.yaw_rad = imuSnapshot.attitudeDegrees.yaw * (kPi / 180.0);
    sample.body.imu.t_us = imuSnapshot.readTimeUs;
    Send(kClientPort, sample);
  }

  int fd_;
  SocketBytes bytes_;
  uint8_t portStorage_[kPortQueueBytes];
  LinkPort port_;
  uint8_t requestStorage_[kRequestQueueBytes];
  LinkItemQueue requests_;
  uint8_t scratch_[LINK_MAX_ITEM_LEN];
  LinkRxDecoder decoder_;
  LinkSchedule odometrySchedule_;
  LinkEndpoint endpoint_;
};

std::unique_ptr<Client> gClient;  // the connected client, if any

// The robot's pass hooks. With no client they do nothing: the robot runs on, as it does without a PC.
void passStart(uint32_t tickStartUs, bool balanceRequested) {
  if (gClient) gClient->serveRequests(tickStartUs, balanceRequested);
}

void passEnd(uint32_t tickStartUs) {
  if (gClient) gClient->endTick(tickStartUs);
}

void publishLog(Logging::Level level, const char *tag, const char *text) {
  if (gClient) LinkSendLog(LinkLogLevel(level), tag, text, *gClient);
}

// Parses HOST:PORT. The host is an IPv4 address, so that the port is never open beyond the address given.
bool parseListen(const std::string &text, std::string &host, uint16_t &port) {
  const size_t colon = text.rfind(':');
  if (colon == std::string::npos || colon == 0) return false;
  host = text.substr(0, colon);
  const std::string number = text.substr(colon + 1);
  if (number.empty() || number.size() > 5 || number.find_first_not_of("0123456789") != std::string::npos) return false;
  const unsigned long value = std::stoul(number);
  if (value == 0 || value > 65535) return false;
  port = static_cast<uint16_t>(value);
  in_addr address{};
  return inet_pton(AF_INET, host.c_str(), &address) == 1;
}

// A listening socket on host:port, or -1 with the reason on stderr.
int listenOn(const std::string &host, uint16_t port) {
  const int fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    std::fprintf(stderr, "navbot_sim: socket: %s\n", std::strerror(errno));
    return -1;
  }
  const int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  inet_pton(AF_INET, host.c_str(), &address.sin_addr);
  if (bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(fd, 1) != 0) {
    std::fprintf(stderr, "navbot_sim: cannot listen on %s:%u: %s\n", host.c_str(), port, std::strerror(errno));
    close(fd);
    return -1;
  }
  return fd;
}

// Takes a waiting client when none is connected, reads what the connected one sent, and drops it once it is closed
// or broken. Never blocks.
void serveSockets(int listener) {
  if (gClient && gClient->broken()) gClient.reset();
  if (!gClient) {
    pollfd waiting{listener, POLLIN, 0};
    if (poll(&waiting, 1, 0) > 0) {
      const int fd = accept(listener, nullptr, nullptr);
      if (fd >= 0) {
        const int one = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        const timeval sendTimeout{kSendTimeoutSec, 0};
        setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &sendTimeout, sizeof(sendTimeout));
        gClient = std::make_unique<Client>(fd);
      }
    }
  }
  if (!gClient) return;

  uint8_t bytes[256];
  for (;;) {
    const ssize_t n = recv(gClient->fd(), bytes, sizeof(bytes), MSG_DONTWAIT);
    if (n > 0) {
      gClient->feed(bytes, static_cast<size_t>(n));
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) return;
    gClient.reset();  // the client closed the connection, or the socket failed
    return;
  }
}

}  // namespace

int runLive(Robot &robot, const std::vector<ScenarioStep> &steps, const std::string &listen) {
  std::string host;
  uint16_t port = 0;
  if (!parseListen(listen, host, port)) {
    std::fprintf(stderr, "navbot_sim: --listen takes an IPv4 address and a port, as HOST:PORT\n");
    return 2;
  }
  const int listener = listenOn(host, port);
  if (listener < 0) return 2;

  robot.linkPassStart = passStart;
  robot.linkPassEnd = passEnd;
  logHook = publishLog;

  // The simulation clock follows the wall clock from where the robot stands now. The scenario steps apply at their
  // times on that clock, as in batch mode; an end line does nothing here, so the robot runs until it is stopped.
  using Clock = std::chrono::steady_clock;
  const Clock::time_point wallStart = Clock::now();
  const uint32_t simStartUs = robot.nowUs();
  size_t nextStep = 0;
  for (;;) {
    serveSockets(listener);
    updateSystemHealth(robot.nowUs());
    const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - wallStart).count();
    const uint32_t wallUs = simStartUs + static_cast<uint32_t>(elapsed);
    while (static_cast<int32_t>(wallUs - robot.nowUs()) > 0) {
      while (nextStep < steps.size() &&
             static_cast<uint32_t>(std::llround(steps[nextStep].tSec * 1e6)) <= robot.nowUs()) {
        std::string error;
        if (!applyStep(robot, steps[nextStep], error)) {
          std::fprintf(stderr, "navbot_sim: %s\n", error.c_str());
          close(listener);
          return 2;
        }
        ++nextStep;
      }
      robot.pass();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

}  // namespace sim
