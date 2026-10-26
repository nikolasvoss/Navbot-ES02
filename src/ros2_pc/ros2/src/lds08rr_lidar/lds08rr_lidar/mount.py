"""Where the LiDAR sits on the robot, in base_link. Shared by the ROS TF and the LiDAR page."""

import math

# The sensor sits at the rotation center. A point straight ahead of the robot reads 240 degrees clockwise
# in the sensor's own bearings (user observation, 2026-10-26). to_laserscan negates bearings, so ahead is
# +120 degrees in the lds08rr frame and the frame is rotated -120 degrees against base_link.
SENSOR_MOUNT_X_M = 0.0
SENSOR_MOUNT_Y_M = 0.0
SENSOR_MOUNT_YAW_RAD = -math.radians(120.0)


def robot_bearing_deg(sensor_bearing_deg: float) -> float:
    """Clockwise bearing from the robot's front for a native clockwise sensor bearing."""
    return (sensor_bearing_deg + math.degrees(-SENSOR_MOUNT_YAW_RAD)) % 360.0
