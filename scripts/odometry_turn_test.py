#!/usr/bin/env python3
"""Odometry turn test: spin the robot in place a known number of turns and compare wheel and gyro yaw.

Records /odom (wheel yaw from navbot_link_bridge in wheel heading mode) and /imu/data (gz) until
Ctrl-C or --duration, then prints the summary as JSON. One counter-clockwise spin seen from above
gives the track width, the left/right wheel mapping, and gyro_yaw_sign. It never commands the robot.
"""

import argparse
import json
import math
import sys
import time

COMMANDED_SEPARATION_M = 0.113


def unwrap_step(previous: float, current: float) -> float:
    """Smallest signed angle from previous to current."""
    return math.atan2(math.sin(current - previous), math.cos(current - previous))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def summarize(turns_ccw: float, wheel_yaw_rad: float, gyro_yaw_rad: float, separation_m: float) -> dict:
    """turns_ccw: true turns counted against a floor mark, positive counter-clockwise seen from above."""
    true_yaw = turns_ccw * math.tau
    result = {
        "true_yaw_deg": round(math.degrees(true_yaw), 1),
        "wheel_yaw_deg": round(math.degrees(wheel_yaw_rad), 1),
        "gyro_yaw_deg": round(math.degrees(gyro_yaw_rad), 1),
        "left_right_mapping": "ok" if wheel_yaw_rad * true_yaw > 0 else "swapped",
        "gyro_yaw_sign": 1 if gyro_yaw_rad * true_yaw > 0 else -1,
    }
    # Wheel yaw scales as 1/separation, so the separation that makes wheel yaw match the true yaw is:
    result["track_width_m"] = round(separation_m * abs(wheel_yaw_rad / true_yaw), 4)
    result["gyro_scale"] = round(abs(gyro_yaw_rad / true_yaw), 4)
    return result


def record(duration_s: float | None):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu

    state = {"wheel": 0.0, "gyro": 0.0, "prev_yaw": None, "prev_imu": None, "odom": 0, "imu": 0}

    def on_odom(msg):
        q = msg.pose.pose.orientation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        if state["prev_yaw"] is not None:
            state["wheel"] += unwrap_step(state["prev_yaw"], yaw)
        state["prev_yaw"] = yaw
        state["odom"] += 1

    def on_imu(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gz = msg.angular_velocity.z
        if state["prev_imu"] is not None:
            prev_t, prev_gz = state["prev_imu"]
            if 0.0 < t - prev_t < 0.1:
                state["gyro"] += (prev_gz + gz) / 2.0 * (t - prev_t)
        state["prev_imu"] = (t, gz)
        state["imu"] += 1

    rclpy.init()
    node = rclpy.create_node("odometry_turn_test")
    node.create_subscription(Odometry, "/odom", on_odom, 50)
    node.create_subscription(Imu, "/imu/data", on_imu, qos_profile_sensor_data)
    start = time.monotonic()
    last_print = 0.0
    try:
        while duration_s is None or time.monotonic() - start < duration_s:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_print >= 1.0:
                last_print = time.monotonic()
                print(f"wheel {math.degrees(state['wheel']):8.1f} deg   gyro (+gz) {math.degrees(state['gyro']):8.1f} deg",
                      file=sys.stderr, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return state


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--turns", type=float, required=True,
                        help="true turns against a floor mark, positive counter-clockwise seen from above")
    parser.add_argument("--duration", type=float, help="seconds to record (default: until Ctrl-C)")
    parser.add_argument("--separation", type=float, default=COMMANDED_SEPARATION_M,
                        help="wheel_separation_m the bridge runs with")
    args = parser.parse_args()
    if args.turns == 0:
        parser.error("--turns must not be 0")
    state = record(args.duration)
    if state["odom"] < 2 or state["imu"] < 2:
        print(json.dumps({"error": "no /odom or /imu/data received", "odom": state["odom"], "imu": state["imu"]}))
        return 1
    summary = summarize(args.turns, state["wheel"], state["gyro"], args.separation)
    summary.update(odom_messages=state["odom"], imu_messages=state["imu"])
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
