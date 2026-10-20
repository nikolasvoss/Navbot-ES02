# LDS08RR photo evidence, 2026-10-13

Status: open

## SLAM systemd autostart, 2026-10-15

Observation: The user explicitly requested SLAM autostart like LiDAR, superseding the earlier exclusion of a SLAM systemd unit. `scripts/install_slam_service.py` installed and enabled `/etc/systemd/system/navbot-slam.service` for UID 1000 and GID 1003. It sources Jazzy and this checkout's built workspace and launches the existing `lds08rr_lidar slam.launch.py`. The unit wants and starts after `navbot-lidar.service`, with failure restart after three seconds and normal cgroup ownership of the launch children. The existing LiDAR unit was not modified. Manual SLAM leftovers were stopped during handover so only one mapper remained.

Evidence: `systemd-analyze verify` accepted the installed unit. Both LiDAR and SLAM report enabled; the SLAM symlink exists under `multi-user.target.wants`. A clean `systemctl restart navbot-slam.service` returned to lifecycle `active`; a separate ROS subscriber received 40 scans at 5.012 Hz during an eight-second window, plus a current `/map` and a resolved `map` to `lds08rr` transform. Killing only the service's main process with SIGKILL changed MainPID from 5844 to 6172, incremented NRestarts from 0 to 1, and automatically restored active mapping. A second independent subscriber again received 40 real scans at 5.012 Hz with map and TF. The requested existing pytest suite passed all 24 tests in 15.25 s on isolated domain 83.

Practical consequence: SLAM now starts through systemd when this host boots and can be managed with `systemctl` and `journalctl` using `navbot-slam.service`. Boot enablement and runtime restart/recovery were checked without rebooting the CM5. Each service start still creates a fresh in-memory map; this task adds neither map persistence nor automatic map loading. Stop the service before starting a manual mapper.

## ROS scan handedness correction, 2026-10-15

Observation: Following the user's request to fix the reversed blue pose arrow, `to_laserscan` now negates native bearings before assigning the CCW-positive ROS bins. The previous conversion copied the native angle sign unchanged. Native UART decoding, scan assembly, sensor-relative LiDAR rendering, and SLAM rendering remain unchanged. This fixes the angular convention at the ROS boundary rather than compensating in TF or the display.

Evidence: Four signed-bearing regression cases for native +90°, -90°, +361°, and -361° failed before the correction with `Obtained: inf; Expected: 1.2` at the required ROS bin. They pass after correction, as do fixed-bin conversion and nearest-return/intensity pairing across the 0° wrap. The selected package rebuilt successfully; the requested full suite passed 24 tests in 18.13 s on isolated ROS domain 83. A focused rerun after adding the same optional-ROS import skips as the existing test passed all six conversion cases.

Runtime: LiDAR and SLAM were restarted with the corrected conversion and a fresh map. An initial relaunch overlapped the old background SLAM process because it ignored SIGINT; both launch instances were explicitly terminated and a single fresh instance activated successfully. A later CM5 reboot stopped manual SLAM again; after manual relaunch, lifecycle state was `active [3]`, `/map.json` reported `mapping` with fresh pose, and Chromium showed Mapping with a 106 × 174-cell map. This does not add automatic SLAM startup. After relaunch, the user slowly turned the sensor in both directions and answered "Both directions correct" to the explicit blue-arrow check. This is user-assisted live confirmation that the reversed heading is corrected. An independent left/right landmark check, physical angle-zero calibration and surveyed trajectory accuracy remain pending.

## User-reported reversed SLAM heading, 2026-10-15

Observation: After the motion check, the user reported that clockwise LiDAR rotation produces opposite rotation in the display, with the same reversal in the other direction, and described the SLAM map as inverted. Asked to distinguish the SLAM pose arrow from sensor-relative wall motion, the user answered "Blue pose arrow turns opposite". This is user evidence of a heading-direction defect. The earlier live check established changing pose and map growth, not physically correct heading or handedness.

Evidence: Source inspection at the time of the report showed the LiDAR canvas uses `x = centerX + sin(angle) * radius` and `y = centerY - cos(angle) * radius`, so increasing native angles appear clockwise from the top. The pre-fix `ros_node.to_laserscan` placed the same native `point.angle_deg` into increasing, CCW-positive ROS bins without a handedness conversion. The SLAM canvas uses `ctx.rotate(-pose.yaw)` to account for screen coordinates. These are code observations; the native packet angles' physical direction has not yet been independently calibrated.

Practical consequence: Prioritize a controlled physical rotation and left/right landmark check against native scan angles, `/scan`, TF yaw and both browser views before changing angle conversion. Opposite movement of stationary walls in a sensor-fixed plot is expected, but a SLAM pose arrow turning opposite to the sensor is not. A missing native-to-ROS handedness conversion is a hypothesis, not a verified fix. The roadmap records this bug and persistence work. At the time of this report no runtime correction had been applied. The subsequent correction is recorded above; maps from before that correction should not be treated as orientation-validated.

## Live lidar-only SLAM verification, 2026-10-15

Observation: On the CM5 running ROS 2 Jazzy, `feat/slam` at `d8226de` built successfully with the `navbot-lidar` venv Python and `colcon --packages-select lds08rr_lidar`. The installed package contains `share/lds08rr_lidar/launch/slam.launch.py` and `config/slam_toolbox.yaml`. Apt upgraded `ros-jazzy-slam-toolbox` to `2.8.5-1noble.20260916.104732` and `ros-jazzy-tf2-ros` to `0.36.23-1noble.20260916.080959`.

Evidence: After stopping `navbot-lidar.service`, the manual launcher `python3 scripts/start_lidar.py --device /dev/ttyAMA0` used ROS auto and was the sole UART reader. Messages have 360 bins, frame `lds08rr`, positive 1-degree angle increments, and about 0.2 s scan time. The 30 m message `range_max` is a software filter, not a measured sensor rating. The SLAM 8 m cap remains provisional; returns slightly beyond it do not establish calibrated useful range.

Observation: `ros2 launch lds08rr_lidar slam.launch.py` configured and activated `slam_toolbox` automatically. `ros2 lifecycle get /slam_toolbox` returned `active [3]`. The existing static publisher arguments produced identity `odom` to `lds08rr`; `tf2_echo map lds08rr` resolved after initial discovery. The included Jazzy launch declares the supplied `slam_params_file` and `use_sim_time` arguments, and the custom parameter names match its shipped `mapper_params_online_async.yaml`.

Observation: During the final motion check the user moved the sensor and then reported "done". A 60.063 s ROS probe received 293 scans at 5.003 Hz between first and last arrivals, with finite ranges of 0.122–8.731 m, plus 30 map messages and 96 poses. Map size grew from 93 × 162 to 122 × 216 cells, and known cells grew from 2,863 to 6,393. Pose x spanned -0.960 to 0.020 m, y spanned -0.730 to 0.350 m, and yaw spanned -2.251 to 1.790 rad. The final pose was about 0.010 m, -0.030 m, -0.05584 rad. Browser snapshots before and after movement showed the arrow at the corresponding positions and headings, with Mapping displayed in both snapshots. This establishes response to real movement; no surveyed trajectory or full-room coverage was provided. The observed 8.731 m return exceeds the provisional 8 m mapping cap, but its accuracy is unverified, so no range tuning was applied.

Observation: Headless Chromium opened `/slam`, `/lidar`, and `/radar/`, each with HTTP 200 and all hub tabs. `/slam` displayed `Mapping`, a rendered occupancy grid and blue pose arrow; `/lidar` displayed `LIVE` at about 5 Hz; the absent radar displayed `Reconnecting` and unavailable sensor status. There were no JavaScript page errors. The manual launcher initially bound loopback as documented; restarting it with `--host 0.0.0.0` made `http://192.168.2.230:8081/slam` reachable on the LAN.

Evidence: The captured SLAM launch output contains configuration, activation, and sensor registration, with no dropped-scan, queue-full, or transform-error messages during the observed windows. An exploratory `ros2 param dump` triggered a warning that optional `stack_size_to_use` was uninitialized; mapping continued. No map serialization was tested. Two host reboots interrupted manual SLAM and motion recording; their cause was not established. LiDAR restarted through its existing service, while SLAM required manual relaunch as expected.

Verification: The requested pytest command initially gave 18 passed and 1 failed while live LiDAR was publishing in the same ROS domain. The demo acceptance subscriber received a real scan with `scan_time=0.2003338933` instead of its expected demo value of 0.2. With an unused `ROS_DOMAIN_ID=83`, the same command passed all 19 tests in 16.49 s. Demo tests must run with live mapping stopped or on an isolated domain so synthetic scans cannot enter the live map.

Practical consequence: No demonstrated fault justified changing the static transform, SLAM parameter names, laser range, or scan direction. Record live acceptance and test isolation in the README. Physical left/right handedness remains unverified; positive LaserScan increments alone do not prove the physical direction. No mirroring correction was guessed. Movement and map growth were observed during the user-assisted check; quantitative trajectory accuracy, complete room coverage, and a physical landmark comparison remain unverified.

## Capture and live UART confirmation, 2026-10-15

Observation: The user supplied `acq0003.csv`, a 16,384-sample logic capture at 800 kHz. UART decoding at 115200 baud, 8N1, non-inverted gives three complete LDS08RR Camsense packets. All three 60-byte packets have header `55 AA 07 0C` and valid checksums (`0x5FBF`, `0x40B4`, `0x6D09`). The first spans 175.328125°–186.25°, with 193–288 mm returns and a reported 300.8125 RPM. `0x8000` occurs as the missing-return marker. The first packet is retained literally in `test_module_tx_capture_decodes_literal_measurements`.

Evidence: User attachment `acq0003.csv`, supplied in this chat on 2026-10-15. The user then stated "do that now, lidar connected". Direct UART capture on `/dev/ttyAMA0` at 115200 received 27,005 bytes in 3.01 seconds, containing 450 checksum-valid Camsense packets and zero failed checksum candidates. Raw [UART capture](captures/2026-10-15-lds08rr-uart0.bin) and [summary](captures/2026-10-15-lds08rr-uart0.json) preserve the live evidence.

Observation: The current working tree assigns LiDAR UART0 on J8 pin 10 / GPIO15 RX and reserves UART2 for HMMD. On the CM5 host, `sudo pinctrl get 14-15` confirmed TXD0/RXD0; `/sys/class/tty/ttyAMA0/device/of_node` resolved to RP1 `serial@30000`. No UART console or other reader occupied the port. No boot configuration or GPIO was changed and no reboot was needed.

Evidence: The existing launcher ran with ROS enabled. An independent subscriber received 26 real `/scan` messages in 5.10 seconds, each with 360 bins. The final observed message had 249 finite bins, 0.207–1.745 m returns, and a reported 0.1987 s scan period. HTTP `/status.json` reported `live`, `serial`, `/dev/ttyAMA0`, Camsense, about 5 Hz, and no receive error; ROS diagnostics had no publish error. Headless Chromium rendered the live polar canvas with no page errors. `/dev/ttyAMA0` had one reader.

Practical consequence: Camsense is confirmed for this module. Remove the project Delta branch, inert motor adapter, protocol selector, and Delta tests; serial startup takes only `--device /dev/ttyAMA0`. Keep demo mode and fixed Camsense status reporting. The user's reported 3 V TX peak remains user evidence; voltage margins, supply wiring, motor circuit, physical angle zero, direction, and distance calibration were not independently verified. Earlier unverified-protocol/dual-driver notes below are historical.

## TX peak clarification, 2026-10-14

Observation: The user reports LiDAR TX at 3 V peak. The earlier 1.8 V DMM reading is retained as a separate measurement.

Evidence: User statement: "lidar tx is 3V peak. document." Instrument and measurement conditions were not specified. No independent agent measurement was performed.

Practical consequence: Record 3 V as the reported TX peak in the canonical hardware reference and browser notes. This supersedes the earlier absence of a reported peak voltage. Low level, timing, protocol, and installed UART operation remain unverified; this documentation update does not establish live hardware acceptance.

## User voltage and installation clarification, 2026-10-14

Observation: The user reports `M−` at 5 V idle and a 1.8 V DMM reading on LiDAR TX. The LiDAR is not connected to the ros2_pc. At the time of this earlier report, TX's peak voltage and protocol were unconfirmed; the later 3 V peak report is recorded above.

Evidence: User answers: "m- is 5V idle and lidar tx is 1.8V (measured with dmm, not sure if it averages from 3v3 or so)" and "not connected atm. implement both, but add todo to remove later again". These are user measurements and installation statements, not independent agent measurements.

Practical consequence: Use an external transistor sink for `M−`; its 5 V idle signal must not reach a GPIO directly. That DMM reading alone did not establish a 1.8 V logic interface. Keep both implemented LDS08RR decoders temporarily. Once this module's capture identifies the protocol and valid scans confirm the selected driver, remove the unused project variant, its selector, tests, and documentation. The roadmap records this removal TODO. Motor output remains inert.

## ros2_pc UART2 inspection, 2026-10-14

Observation: The software host identifies as `Raspberry Pi Compute Module 5 Rev 1.0` running Ubuntu 24.04.5. `/dev/ttyAMA0` and `/dev/ttyAMA10` exist; `/dev/ttyAMA2` does not. `/boot/firmware/config.txt` has no UART2 overlay. `pinctrl get 4-5` reports GPIO4 and GPIO5 with no alternate function, while GPIO14 and GPIO15 carry UART0. The installed overlay README documents `dtoverlay=uart2-pi5` for UART2 on GPIO4 and GPIO5.

Evidence: Direct inspection of `/proc/device-tree/model`, `/etc/os-release`, `/dev`, `/boot/firmware/config.txt`, `pinctrl get`, and `/boot/firmware/overlays/README` on the ros2_pc on 2026-10-14. No boot configuration was changed and no sensor wiring was performed.

Practical consequence: Reserve J8 pin 7 / GPIO4 for UART2 TX and pin 29 / GPIO5 for UART2 RX. After electrical checks, add the documented overlay and reboot, then inspect the resulting device mapping. Do not assume `/dev/ttyAMA2` until the active image confirms it. Reserve GPIO17 on J8 pin 11 for a future external `M−` sink only; do not wire it directly to the module.

## LiDAR software status, 2026-10-14

Observation: The `lds08rr_lidar` package uses the released `lds2d[viz]==0.7.0` drivers with explicit Delta or Camsense selection. One reader feeds the upstream plot and ROS 2 output. The web status page starts without a UART or the HMMD radar. Delta opens with an inert motor-output adapter.

Evidence: The implementation and hardware-free protocol fixtures in the `feat/lds08rr-lidar` worktree. Upstream driver fixtures exercise both protocol parsers; they do not identify the installed module's protocol.

Practical consequence: Use demo mode for browser checks. Keep protocol selection provisional until a raw capture from this module identifies it. The ROS angle zero, direction, range limits, and scan-start timestamp remain software assumptions or estimates until hardware measurements establish them.

## Connector M− measurement, 2026-10-14

Observation: The LiDAR connector's `M−` pin is active low and draws 1 mA when pulled to GND.

Evidence: User report on 2026-10-14: "the lidar m- pin on the connector is low active and draws 1mA when pulled to GND". No independent measurement was performed by the agent.

Practical consequence: Activate by pulling the connector's `M−` pin low; the controlling sink must handle the reported 1 mA. This is connector-pin current, not total module or motor current. Open-circuit voltage, logic thresholds, and PWM behaviour are still unverified. This updates the electrical evidence beyond the photographs described below.

## Photo evidence

Observation: The user-provided module label reads LDS08RR, 5VDC/1.5W, Class 1 Laser Product, IEC 60825-1:2014, and EN 60825-1:2014+A11:2021. A separate label reads HHCD2SAA1204104. The PCB has a six-contact connector whose upright photographed left-to-right labels are NC, M−, RX, TX, GND, VIN. A separate two-contact connector has red and black wires. Four outer mounting ears are visible.

Evidence: Original user photographs 59725.jpg and 59723.jpg supplied in this chat on 2026-10-13, preserved as [label](../../docs/ros2_pc/assets/lds08rr-label.jpg) and [PCB](../../docs/ros2_pc/assets/lds08rr-pcb.jpg). Canonical reference: [LDS08RR hardware](../../docs/ros2_pc/hardware/lds08rr-lidar.md).

Practical consequence: The model and nominal supply rating can be documented as label evidence. The photo position order is not manufacturer pin numbering. No continuity, UART voltage, current, motor-control function, protocol revision, or installation has been verified. Do not derive wiring from M− or wire colours. The two protocol variants documented upstream cannot be distinguished from these photographs alone.

# LDS08RR mounting sketch, 2026-10-13

Observation: A user-supplied mounting sketch gives four Ø 3.1 mm holes at 54.0 mm horizontal and 44.0 mm vertical centre spacing. A side-view sketch gives 38.0 mm overall height, 59.0 mm lower-housing width, and 24.5 mm motor width. Its 17.6 mm and 28.0 mm height references have unconfirmed upper reference surfaces.

Evidence: Transcribed in [dimension record](../../cad/ros2_pc/lds08rr-dimensions.json), with [drawing](../../docs/ros2_pc/assets/lds08rr-hole-pattern.svg), [mounting sketch](../../docs/ros2_pc/assets/lds08rr-mounting-sketch.jpg), and [side sketch](../../docs/ros2_pc/assets/lds08rr-side-sketch.jpg). The record marks every value as transcribed from a sketch, not measured.

Practical consequence: Use these values only as unverified planning inputs. Measure the module before fixing a bracket. Do not infer plate thickness from the 17.6 mm or 28.0 mm references. The optical-axis and motor-centre offsets from the hole pattern are not established.

# LDS08RR heights and simplified model, 2026-10-13

Observation: A second user sketch (side and top view) and the user's chat answers establish the following. The left height reads 17.8 mm, not the 17.6 mm transcribed earlier, and runs from the housing bottom to the underside of the plastic plate. The 28.0 mm runs to the plate top, which carries the PCB, so the plate is 10.2 mm thick. The mounting ears are 4.3 mm thick and flush with the plate top. The rotor centre coincides with the hole-pattern centre. The sketch also gives a Ø 59.6 mm plate, a 50.0 mm distance from the motor's outer edge to the rotor centre, an 80.9 mm overall length, and a 69.4 mm hole diagonal.

Evidence: [combined sketch](../../docs/ros2_pc/assets/lds08rr-dimension-sketch.jpg), the user's answers in chat on 2026-10-13, the updated [dimension record](../../cad/ros2_pc/lds08rr-dimensions.json), and the [simplified STEP model](../../cad/ros2_pc/lds08rr-simple.step) built from [lds08rr_simple.py](../../cad/ros2_pc/lds08rr_simple.py).

Practical consequence: The earlier warning against subtracting 17.6 from 28.0 no longer applies, because the reference surfaces are now confirmed. The values are still sketch values, not measurements. The 59.0 mm housing width and the 80.9 mm overall length disagree by about 1.4 mm with a housing concentric to the rotor. Measure this before relying on the clearance on the housing side opposite the motor.
