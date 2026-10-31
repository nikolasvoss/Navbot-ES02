import os
import sys
from pathlib import Path

# The bridge tests drive the same in-memory firmware as the navbot_link tests.
sys.path.insert(0, str(Path(__file__).resolve().parents[6] / "tests" / "navbot_link"))

# The bridge runs on this machine and publishes odom and imu/data. Node tests get their own DDS
# domain and stay on the loopback interface, so they never join the live graph.
os.environ["ROS_DOMAIN_ID"] = "77"
os.environ["ROS_LOCALHOST_ONLY"] = "1"
