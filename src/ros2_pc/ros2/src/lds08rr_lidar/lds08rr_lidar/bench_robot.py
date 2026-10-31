"""Fake robot of the navigation bench: a healthy navbot_firmware status, a battery, and the start pose.

Run by bench_launch.py as `python3 -m lds08rr_lidar.bench_robot` with the parameters x, y and yaw.
It publishes nothing to the robot and reads no serial port.
"""

import math

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

BATTERY_V = 7.6
FIRMWARE_VALUES = {"balance_state": "Run", "ch5_position": "1", "rc_valid": "1"}


class BenchRobot(Node):
    def __init__(self) -> None:
        super().__init__("bench_robot")
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.pose_sent = False
        self.create_timer(1.0, self.tick)

    def tick(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.diag_pub.publish(self.diagnostics(stamp))
        battery = BatteryState()
        battery.header.stamp = stamp
        battery.voltage = BATTERY_V
        self.battery_pub.publish(battery)
        # AMCL and the loopback simulator both subscribe to /initialpose; send the start pose once they do.
        if not self.pose_sent and self.pose_pub.get_subscription_count() >= 2:
            self.pose_pub.publish(self.initial_pose(stamp))
            self.pose_sent = True

    def diagnostics(self, stamp) -> DiagnosticArray:
        status = DiagnosticStatus(name="navbot_firmware", level=DiagnosticStatus.OK, message="bench")
        status.values = [KeyValue(key=key, value=value) for key, value in FIRMWARE_VALUES.items()]
        array = DiagnosticArray()
        array.header.stamp = stamp
        array.status = [status]
        return array

    def initial_pose(self, stamp) -> PoseWithCovarianceStamped:
        yaw = self.get_parameter("yaw").value
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(self.get_parameter("x").value)
        msg.pose.pose.position.y = float(self.get_parameter("y").value)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.covariance[0] = 0.05  # x
        msg.pose.covariance[7] = 0.05  # y
        msg.pose.covariance[35] = 0.03  # yaw
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BenchRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
