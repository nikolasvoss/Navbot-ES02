# HMMD setup and observations

Status: open

## Established state

| Date | Observation | Evidence | Practical consequence |
| --- | --- | --- | --- |
| 2026-10-04 | ROS 2 is installed on a ros2_pc. | User instruction in the planning session. | Do not plan a new ROS 2 installation. Record the distribution and working environment before building packages. |
| 2026-10-04 | The running ros2_pc uses Ubuntu 24.04.5 LTS, kernel `6.8.0-1064-raspi`, ROS 2 Jazzy, and Python 3.12.3. `/dev/ttyAMA0` was present; the process `hmmd_sensor` read this port and published `/hmmd/rdmap` as best effort and `/hmmd/status`. | `uname -a`, `/etc/os-release`, `echo $ROS_DISTRO`, `ros2 topic info /hmmd/rdmap --verbose`, `ros2 topic echo /hmmd/status --once`, `/dev/ttyAMA*`, and process list on the ros2_pc on 2026-10-04. | Do not reinstall ROS. For browser access, install the matching rosbridge for Jazzy, and do not start the existing `hmmd_sensor` in parallel. |
| 2026-10-04 | A real HMMD data status reported `connected=true`, `stale=false`, `frames_received=15508`, `frame_rate_hz=5.000`, `last_frame_age_sec=0.140`, and no I/O errors. | Output of `ros2 topic echo /hmmd/status --once` on the ros2_pc; afterwards, both `roslibpy` 2.1.0 and a ROS-vs-WebSocket comparison received real `/hmmd/rdmap` messages. | The UART is not merely open: complete HMMD frames were received. The nominal browser limit of 10 Hz is a display maximum, not a sensor frequency. |
| 2026-10-04 | The active Wi-Fi network was `192.168.178.28/24`; a PC browser with source address `192.168.178.22` loaded HTML, CSS, and JavaScript from the static server successfully. | `ip -br addr`; HTTP server requests for `/`, `/style.css`, `/app.mjs`, `/topic_registry.mjs`, `/model.mjs`, and `/rosbridge_client.mjs` returned HTTP 200 on 2026-10-04. | Browser URL at the time of the check: `http://192.168.178.28:8080/`. The HMMD WebSocket display from the PC is reachable only after a local SSH tunnel on the PC. |
| 2026-10-04 | The user reports that the HMMD sensor is connected to J8 and names "GPIO10, GPIO8 etc."; whether GPIO numbers or physical J8 pin numbers are meant, and which module wires are there, is still unresolved. | User report in the guide correction. No visual check or electrical measurement exists. | Before power-on, check physical J8 pin numbers, module TX/RX, ground, and 3.3 V supply against the actual markings. GPIO8/GPIO10 are not a UART3 TX/RX pair. |
| 2026-10-04 | Roadmap item 1 requires a stationary HMMD test with heatmap and bag playback. | `docs/ros2_pc/software/roadmap.md`, section 1. | Operate the radar first, separately from the drive control. |
| 2026-10-05 | The installed HMMD reported firmware v1.6.1. Parameter ID 1 read Maximum Distance Gate 12; parameter ID 4 read Target Disappearance Delay 30 seconds. Unchanged write operations for both values were confirmed with ACK, matching readback, and Save/Exit ACK. | Direct serial capture via ros2_pc `/dev/ttyAMA0` at 115200 baud on 2026-10-05; the captured request and response bytes are in `protocol.md`, section "HMMD configuration, checked on the module". | The driver may use only the checked IDs, response formats, and ranges. The check after power-off and power-on is missing; do not report the Save/Exit ACK as power-cycle persistence. During the capture, `hmmd_sensor` was paused and then restarted as the sole UART owner. |
| 2026-10-06 | With hand movements in front of the installed radar, the current debug map typically showed Doppler bins 10–11 at standstill, lower bins (about 5) when moving toward the radar, and higher bins (about 15) when moving away from it; the exact values varied with the movement. | User report from visual observation of the running Doppler map; no raw capture and no movement at a known speed. | For this device, the direction is observed qualitatively: lower = approaching, higher = receding, middle around 10–11 = standstill. No m/s value without a known bin-frequency mapping and calibration; this is only the radial movement toward or away from the radar. |

## Not yet observed

- The specific HMMD module variant; firmware v1.6.1 is observed for the installed unit.
- ros2_pc carrier board, actual J8 wiring, supply, and signal levels.
- Behavior after power-off and power-on, and persistence of the radar values.
- Calibration of the Doppler bins to radial speed in m/s, and behavior at known speeds.
- Behavior under longer or higher bridge load.

## Schema for new findings

Each new entry contains the date, the specific observation, the evidence file or measurement log, and the practical consequence. Mark manufacturer statements and assumptions separately. A missing test is not evidence of sensor behavior.

## Software check on 2026-10-04

The packages `hmmd_interfaces` and `hmmd_radar` are implemented. The check in the development worktree used only synthetic frames and simulated serial ports. 15 tests passed; one ROS graph test was skipped because no local ROS runtime was available. The actual renderer produced the raw, log, and stale views, including the color scale and bin axes. Evidence: the tests under `src/ros2_pc/ros2/src/hmmd_radar/test/` and the executed Matplotlib render checks.

## ros2_pc browser access on 2026-10-04

In the separate worktree at commit `f93ab3f34ee273914629e4e4c0d851866f819ae2`, `hmmd_interfaces` and `hmmd_radar` were built with ROS 2 Jazzy. `ros-jazzy-rosbridge-server` and `ros-jazzy-rosbridge-library` version 2.7.1 were added. The server listened on `127.0.0.1:9090`; the static browser file was served on `0.0.0.0:8080`. Filter strings had to be passed with inner quotation marks because of the ROS string parameters. See the HOWTO and reference for the verified filter semantics; service and action capabilities are not a pure receive limit.

A roslibpy client without ROS or generated message types received 20×16, 320 raw values, header/timestamp, and status. A direct ROS subscriber and a WebSocket client saw an identical frame, including the timestamp. The PC browser loaded the page over Wi-Fi; the user started the required SSH tunnel and answered the question about the running access with "running". Afterwards, `sudo ss -tnp` showed an SSH forward to `127.0.0.1:9090`; the bridge log showed the subscriptions to both HMMD topics and their reconnection after the restart. Real HMMD reception on the ros2_pc was not replaced by synthetic data.

The local Chromium check against the real rosbridge passed for the raw/log view, 20×16/320 values including the `uint32` maximum, topic selection, and all error states. For these phases, PID 3606 (`hmmd_sensor`) was briefly paused; the explicitly synthetic ROS source did not access the UART. Afterwards, the same sensor node was resumed and real frames were observed again. `sudo fuser -v /dev/ttyAMA0` then still showed only PID 3606. `sudo pinctrl get 14,15` showed GPIO14=TXD0 and GPIO15=RXD0; this confirms the software mux, not the physical wiring. The [check log](captures/2026-10-04-hmmd-browser-integration.json) contains the individual states. All 16 Python tests, including the isolated ROS graph test, and all six node tests passed; the Jazzy test needed a minimal allowance for `numbers.Integral`, not just Python `int`, in the Matplotlib display model.
