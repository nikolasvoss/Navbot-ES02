# Roadmap

## 1. Show the HMMD radar as a 20 × 16 range-Doppler map in ROS 2

Status: Software implemented; live acceptance still open.

The ros2_pc sensor node, the ROS topics, the browser display, and the shared local or remote start are implemented. The available evidence includes real HMMD frames and browser access on the ros2_pc. Full acceptance is still pending.

Before closing, confirm the current ROS 2 build on the ros2_pc, the physical UART and voltage check, recordings of an empty measurement area and of a still and a moving person, and playback without a live sensor. The [HMMD ROS 2 developer guide](../software/hmmd-ros2.md) contains the current commands and limits.

## 2. Integrate Roborock LiDAR in ROS 2

Status: Real browser scans and ROS 2 topic `/scan` confirmed on 2026-10-15; RViz, calibration, and bag playback still open. Second ROS 2 milestone.

The `lds08rr_lidar` package provides a demo without radar, an independent browser status
page, and a ROS 2 adapter. It uses the confirmed Camsense driver from
`lds2d[viz]==0.7.0` and publishes complete scans on `/scan` and the status on
`/lidar/status`. No host PWM output is activated. The hardware and protocol limits are
described in the [LDS08RR hardware documentation](../hardware/lds08rr-lidar.md).
The [software guide](../software/lds08rr-lidar.md) contains build, demo, and start
commands.

The first visible hardware result should be a live laser scan in RViz. The first
test is stationary, outside the Navbot. Software alone confirms neither wiring nor
sensor protocol, orientation, range, or scan rate.

### Basis and open questions

The TX capture `acq0003.csv` confirms the LDS08RR Camsense format at 115200 baud,
8N1. A direct capture on `/dev/ttyAMA0` produced 450 valid packets without checksum
errors; the browser and ROS receive real scans at about 5 Hz. Angle zero point,
rotation direction, measurement accuracy, and electrical acceptance remain open.

- [x] After protocol confirmation and valid scans, removed the unused Delta connection,
  protocol selection, and the related tests and guides.

### Planned scope

- Identify the module and document the connection data used.
- Stationary operation with a suitable power supply and the required drive of the scan
  motor, if the module needs it externally.
- Connection to the ros2_pc and publication as `sensor_msgs/msg/LaserScan` on topic
  `/scan`. The software uses a fixed angular grid and positive infinity for measurements
  without a return.
- Document angular direction, angular zero point, distance units, timestamps, and
  invalid measurements.
- Display in RViz with a separate sensor coordinate system.
- Recording and playback with rosbag2, and display of missing scans.
- Prepare mounting on the Navbot, including weight, power supply, a clear field of view,
  and the transform to the robot coordinate system.

### Done when

- Real scans appear continuously in RViz.
- Distances and angles match at several known positions in the test setup within the
  tolerances defined for the identified module.
- A recording can be displayed again without a connected sensor.
- An interrupted connection is detected; old scans do not appear as current.
- Sensor designation, connection data, driver, and observed scan rate are documented.

Before use while the robot is driving, check the effects of tilt, balance movements,
additional weight, and power draw. SLAM and autonomous navigation are follow-on projects.

## 3. Telemetry and logging via the ros2_pc

Status: Planned, not started yet.

High-rate telemetry and recordings from the ESP32 will in future run over the UART connection to the ros2_pc and be recorded there, for example with rosbag2. The ESP32 does not run its own Wi-Fi logging service; the earlier Wi-Fi logging proposals have been removed. Format, rate, and recording scope are still to be defined.
