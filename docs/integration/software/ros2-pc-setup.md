# ros2_pc setup for the robot link

Everything the ros2_pc needs for the [NavBot link](ros2-uart-protocol.md) that the repository checkout does not provide: operating system and packages, the boot configuration for the link UART, serial permissions, and checks. Run the commands on the ros2_pc. Nothing here has been run against the robot yet; the UART path was checked on 2026-10-26 (`/dev/ttyAMA3` carried the robot's frames at 2000000 baud), but the bridge itself has not run against the robot.

## 1. Operating system and ROS 2

- ROS 2 Jazzy on Ubuntu 24.04, Python 3.12. This is the setup the bridge was built and tested on (19 October 2026, pseudo-terminal only). Install ROS 2 from the packages described in the ROS 2 Jazzy installation guide; the repository does not install it.
- The system clock is the only time source for the ROS stamps. Keep it correct (network time or an RTC); see [Time stamps](ros2-uart-protocol.md#time-stamps).

## 2. System packages

The bridge package declares these. `rosdep install` pulls them in, or install them directly:

```bash
sudo apt install python3-serial python3-protobuf
```

`python3-protobuf` must be version 3.21 (the Ubuntu 24.04 package). The generated `navbot_link_pb2.py` needs that runtime; a newer protobuf from pip breaks it.

Only a development machine that regenerates the schema needs `protobuf-compiler` (3.21.x) and `pip install nanopb==0.4.9.1`. The ros2_pc does not.

## 3. Link UART in the boot configuration

The ros2_pc side is on GPIO8 (TX, J8 pin 24) and GPIO9 (RX, J8 pin 21), see the [J8 allocation](../../ros2_pc/hardware/system-overview.md#j8-pin-allocation). The carrier datasheet calls this pair UART3.

1. Add the overlay to `/boot/firmware/config.txt`, as the HMMD guide does for UART2:

   ```text
   dtoverlay=uart3-pi5
   ```

   Check that the overlay exists on the installed image before rebooting: `dtoverlay -h uart3-pi5`.
2. Reboot.
3. Check the pin functions: `pinctrl get 8-9` must show the UART3 TX and RX functions on GPIO8 and GPIO9.
4. Find the device path. Do not infer it from the UART number: list `/dev/ttyAMA*`, and compare with `dmesg | grep -i tty`. `/dev/ttyAMA3` is confirmed on the current image (2026-10-26). Note the path, since it is the link multiplexer's `--port`.
5. Make sure no serial console, getty or other service uses the port, and that the link multiplexer is the only reader. UART0 (`/dev/ttyAMA0`) belongs to the LiDAR and UART2 to the HMMD radar; do not reuse them.

Wiring and levels are physical and cannot be set in software: cross TX and RX, join the grounds, and keep both ends at 3.3 V logic. The robot side is GPIO20 (RX) and GPIO19 (TX), opposite to the PCB net names RX1/TX1. Verify the pins and the voltage on the boards before connecting.

## 4. Serial permissions

The user that runs the multiplexer needs access to the port:

```bash
sudo usermod -aG dialout "$USER"
```

Log in again afterwards. The [multiplexer unit](../../../src/ros2_pc/systemd/navbot-link-mux.service.in) sets `SupplementaryGroups=dialout`, as the [LiDAR unit](../../../src/ros2_pc/systemd/navbot-lidar.service.in) does, so the service itself does not depend on this group.

## 5. Build and run

Build the bridge as described in the [bridge page](navbot-link-bridge.md). Start the multiplexer by hand with the device path from step 3 (`python3 -m navbot_link.mux --port <path> --baud 2000000 --listen 127.0.0.1:7650`), then the bridge with the measured wheel radius and track width. Or install the services (`python3 scripts/install_link_bridge_service.py`), which start the multiplexer on `/dev/ttyAMA3` and the bridge at boot, with a 0.01915 m wheel radius and a 0.113 m track width.

## 6. Checks

With the robot powered, balance disabled and motor outputs safe:

1. The multiplexer opens the port at 2000000 baud. A failing open (wrong path or missing group) shows in `journalctl -u navbot-link-mux`.
2. `ros2 topic hz /odom` and `ros2 topic hz /imu/data` show about 50 Hz (see the [protocol rules](ros2-uart-protocol.md#rules)).
3. The bridge reports no `bad_frames`, `decode_errors` or `lost_frames` growth. A burst at start is normal while it resynchronises.
4. Roll the robot forward by hand: `/odom` twist must be positive.

## Not covered here

- The robot firmware build and flash: see the [USB, serial and flashing guide](../../robot/software/usb-serial-flashing.md).
- The LiDAR and HMMD radar setup and their services: see the [LiDAR software](../../ros2_pc/software/lds08rr-lidar.md) and the [HMMD guide](../../ros2_pc/software/hmmd-ros2.md).
