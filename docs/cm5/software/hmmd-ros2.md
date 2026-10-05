# HMMD ROS 2 developer guide

Use this guide to build, start, inspect, and record the HMMD ROS 2 software. The implementation lives in `src/cm5/ros2/`. The launcher lives at `scripts/start_hmmd.py`.

The software has a sensor node on the CM5, a loopback-only rosbridge server, a static browser page, and a launcher that can start the services locally or over SSH. In remote mode, the launcher opens a tunnel to the browser and rosbridge ports. The CM5 services stay up after the tunnel ends.

## Check the sensor connection

The documented default uses UART0 on physical CM5IO J8 pins 8 and 10. Those are GPIO14/TX and GPIO15/RX. The module TX connects to J8 pin 10, and module RX connects to J8 pin 8. Connect ground to pin 6 and 3.3 V to pin 1 or 17.

Use the pin 1 marker to identify the header orientation. The HMMD vendor documents a 3.0–3.6 V supply and 0–3.3 V UART signals. Check the actual module, carrier, wiring, supply, and GPIO reference voltage before applying power. Do not connect the module to 5 V. The project has software evidence for UART0, but the physical board and current wiring have not been verified. See the [CM5IO pin reference](../hardware/compute-module-5-io-board.md) and [HMMD observations](../../../agent_notes/cm5/sensors/hmmd/observations.md).

Do not infer a UART from a device name alone. The J8 UART device exists whether or not the sensor is connected. For UART0, check `/dev/ttyAMA0`, pin functions, permissions, and console ownership. If you use another UART or a USB adapter, confirm its wiring and device path before starting the node. The sensor node must be the only reader of its UART.

## Build and check the workspace

Run these commands on the CM5 from the project checkout. The verified setup used ROS 2 Jazzy and Python 3.12.3. Check the installed ROS distribution before building.

```bash
cd ~/Navbot-ES02-cm5-hmmd
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/cm5/ros2/src --ignore-src -r -y --skip-keys=ament_python
colcon --log-base src/cm5/ros2/log build --base-paths src/cm5/ros2/src --build-base src/cm5/ros2/build --install-base src/cm5/ros2/install
source src/cm5/ros2/install/setup.bash
```

Source both setup files in each new terminal. The `ament_python` build type comes from the ROS installation; `rosdep` has no system key for it, so the command skips that key.

Run the software checks from the project root with the ROS environment active:

```bash
PYTHONPATH="src/cm5/ros2/src/hmmd_radar${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s src/cm5/ros2/src/hmmd_radar/test -v
python3 scripts/test_start_hmmd.py -v
node --test src/cm5/ros2/src/hmmd_radar/test/web/client.test.mjs
```

The Python unit tests use synthetic frames and a mock serial port. They do not prove live sensor reception. A ROS build and live sensor check still require the CM5 and sensor.

## Start the services

On the CM5, run the launcher from the checkout:

```bash
python3 scripts/start_hmmd.py
```

From a PC, run the same script with an SSH alias or `user@host`:

```bash
python3 scripts/start_hmmd.py --ssh cm5
```

The remote default workspace is `$HOME/Navbot-ES02-cm5-hmmd`. The local default is the checkout containing the script. Use `--workspace PATH` for another workspace. Use `--device PATH` for a confirmed UART path, `--baud-rate N` for another baud rate, or `--startup-timeout N` to change the service wait. The defaults are `/dev/ttyAMA0`, `115200`, and `30` seconds.

The launcher starts or reuses the sensor node, rosbridge, and static web server. It refuses an unknown UART owner or a service with the wrong identity or configuration. It does not stop existing processes. Do not start a second `hmmd_sensor` manually.

In remote mode, the launcher checks that local ports 8080 and 9090 are available, starts or checks the CM5 services, then forwards both ports to local loopback. Keep the terminal open while you use `http://127.0.0.1:8080/`. Press Ctrl+C to end the tunnel. The CM5 services keep running. Add `--no-browser` to suppress opening a browser window; the tunnel still stays open.

Newly started service logs are `/tmp/navbot-hmmd-sensor.log`, `/tmp/navbot-hmmd-rosbridge.log`, and `/tmp/navbot-hmmd-web.log` on the CM5. The launcher uses `ws://127.0.0.1:9090` for rosbridge and binds rosbridge to loopback. Do not expose port 9090 directly to the LAN. The installed rosbridge version still offers operations beyond subscriptions, so the browser's read-only behavior does not make the server read-only.

For manual diagnosis, first check that no process owns the sensor UART. With ROS and the workspace sourced, run:

```bash
ros2 run hmmd_radar hmmd_sensor --ros-args -p port:=/dev/ttyAMA0 -p baud_rate:=115200
```

In another terminal, inspect status and map rate:

```bash
ros2 topic echo /hmmd/status
ros2 topic hz /hmmd/rdmap
```

The launcher is for live-sensor startup. Do not use it for bag replay because it starts the sensor node.

## ROS topics and parameters

| Topic | Type | Meaning |
| --- | --- | --- |
| `/hmmd/rdmap` | `hmmd_interfaces/msg/RangeDopplerMap` | Raw 20 by 16 matrix and host receive timestamp. |
| `/hmmd/status` | `diagnostic_msgs/msg/DiagnosticArray` | UART state, frame rate, counters, and last-frame age. |

`RangeDopplerMap` carries `doppler_bins`, `range_gates`, and 320 `uint32` values in `amplitude_squared`. The flat index is `doppler_bin * 16 + range_gate`. The header timestamp is the ROS host receive time, not a sensor measurement time. The display uses bin indices; physical range, speed, orientation, and Doppler sign have not been established.

| Sensor parameter | Default | Meaning |
| --- | --- | --- |
| `port` | empty | Empty disables serial access. |
| `baud_rate` | `115200` | UART baud rate. |
| `poll_period_sec` | `0.01` | Maximum interval between bounded input polls. |
| `stale_timeout_sec` | `1.0` | Age after which a frame or partial frame is stale. |
| `diagnostics_period_sec` | `1.0` | Interval between status messages. |

The node publishes `/hmmd/rdmap` with Best Effort QoS and depth 5. It publishes `/hmmd/status` with Reliable QoS. Parameters are read-only after startup. Status fields include `connected`, `stale`, `last_frame_age_sec`, `frames_received`, `frame_rate_hz`, `malformed_candidates`, `discarded_bytes`, `reconnects`, `input_backlog_overflows`, and `last_io_error`.

`connected=true` means the host opened the UART; it does not prove that the sensor sent a frame. `stale=true` means the last complete frame is older than `stale_timeout_sec`. `last_frame_age_sec` uses monotonic host time. The frame and error counters cover decoded frames, malformed candidates, discarded bytes, reconnects, and input backlog overflows. `last_io_error` contains the latest serial error.

The browser reads endpoint declarations from `web/endpoint-manifest.json`. The launcher reads the same file and derives rosbridge topic and service filters with `scripts/sensor_channel_config.py`. Topic records use `name`, `type`, `label`, `direction`, `reliability`, `throttleMs`, `staleAfterMs`, and `pinned`; `defaultPayload` is optional. Set `direction` to `subscribe`, `publish`, or `both`. Service records use `name`, `type`, `label`, and `timeoutMs`, with an optional `defaultPayload`. The current manifest subscribes to the HMMD topics and exposes `/hmmd_sensor/get_parameters` as a read-only `rcl_interfaces/srv/GetParameters` request. That service reads node parameters. It does not configure HMMD hardware.

The browser keeps the latest message for each active receive topic and applies the manifest freshness limit. HMMD validation stays in `model.mjs` and is injected into the generic `RosbridgeClient`; other topic JSON remains opaque. Reconnects resubscribe active receive topics but do not replay publications or service calls. A publish status means that the browser submitted the JSON message to rosbridge. ROS topic publication has no delivery acknowledgement. Service responses use request IDs and fail on bridge rejection, malformed response, disconnect, stop, or timeout.

The generic controls list manifest-declared publish topics and services. Select an endpoint, edit its JSON object, and submit it. A receive-only endpoint cannot be published, and a publish-only endpoint is not subscribed. The production manifest has no publish endpoint because the HMMD driver has no corresponding command interface.

### Sensor-channel interface baseline

This is the initial comparison baseline for future sensor-channel changes. It describes the public contract at the manifest and browser-client boundary; rosbridge remains the transport. A later change should state which rows it extends or changes and keep the existing HMMD behavior unless an intentional migration is documented.

| Surface | Current contract |
| --- | --- |
| Stream endpoint | ROS topic declaration with exact `name`, message `type`, UI `label`, `direction`, `reliability`, `throttleMs`, `staleAfterMs`, and `pinned` fields. Optional `defaultPayload` is a JSON object. |
| Request endpoint | ROS service declaration with exact `name`, service `type`, UI `label`, and `timeoutMs`. Optional `defaultPayload` is a JSON object. |
| Receive API | `subscribe(name)` and `unsubscribe(name)` operate only on declared receive-capable topics. `snapshot()` returns the connection generation, endpoint errors and each topic's latest message, receipt time, age, generation, and freshness. |
| Send API | `publish(name, object)` accepts a declared publish-capable topic and JSON object. It reports submission to rosbridge; it cannot confirm subscriber delivery. The bridge's correlated error status is surfaced as an endpoint error. |
| Request API | `callService(name, object)` accepts a declared service and returns a promise for response `values`. Requests have unique IDs and a finite timeout; rejection, malformed/mismatched response, disconnect, stop, or send failure rejects the promise. |
| Reconnect | Active subscriptions are restored. Publications and service calls are never queued or replayed. An older sample does not become fresh solely because the bridge reconnects. |
| Validation | The endpoint manifest is validated at the browser and launch boundary. Type-specific validation is optional and injected by the owning view; without it, received message objects are opaque. |
| Permissions | The launcher derives topic subscribe, topic publish, and service allowlists from the same manifest. The list bounds this managed launcher but does not disable rosapi or replace network isolation. |

The `RosbridgeClient` browser surface is `new RosbridgeClient({ url, topics, services, validators })` followed by `start`, `stop`, `subscribe`, `unsubscribe`, `onChange`, `snapshot`, `publish`, or `callService`. Methods that send data resolve endpoint names and direction against the declarations; callers do not send raw rosbridge envelopes. Invalid JSON objects, undeclared operations, and disconnected sends fail at the client boundary. Keep sensor-specific parsing, configuration semantics, and rendering in the owning ROS driver or browser view.

Current production capabilities are deliberately smaller than the generic channel:

| Endpoint | Direction | Payload / effect |
| --- | --- | --- |
| `/hmmd/rdmap` | Receive | Existing `hmmd_interfaces/msg/RangeDopplerMap`; the view validates its 20 by 16 uint32 matrix and renders the heatmap. |
| `/hmmd/status` | Receive | Existing `diagnostic_msgs/msg/DiagnosticArray`; the view shows HMMD serial and frame status. |
| `/hmmd_sensor/get_parameters` | Request | Read-only `rcl_interfaces/srv/GetParameters` for the listed ROS node parameters. |
| HMMD setting writes | Not available | The current sensor node exposes no runtime hardware write operation. The browser has no production publish endpoint. |

Synthetic `/demo/*` endpoints exist only in the test fixture and are not part of production permissions. Future radar configuration work must add a real driver-owned ROS operation before declaring it in the production manifest. Its user-visible result should state whether the driver accepted the request and, where hardware supports acknowledgement, whether the setting was confirmed by the device. A generic rosbridge submission message alone is not a configuration success.

For comparable future changes, record the endpoint additions, ROS types, direction, timeout/freshness/QoS policy, response semantics, and any payload-size or compatibility changes. Demonstrate the new path in the synthetic fixture and test both successful and failed operations. Keep the original HMMD map/status path covered. Do not silently turn read-only parameters into writable configuration or add a command until the device protocol and driver behavior are verified.

To add another sensor, add its exact ROS endpoint name, type, direction, and metadata to the manifest. For example, a driver that publishes radar detections using [`radar_msgs/msg/RadarScan`](https://github.com/ros-perception/radar_msgs/blob/ros2/msg/RadarScan.msg) could declare:

```json
{
  "name": "/front_radar/scan",
  "type": "radar_msgs/msg/RadarScan",
  "label": "Front radar scan",
  "direction": "subscribe",
  "reliability": "best_effort",
  "throttleMs": 50,
  "staleAfterMs": 500,
  "pinned": false
}
```

For planar LiDAR, use the standard [`sensor_msgs/msg/LaserScan` definition](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/LaserScan.msg). For compressed camera frames, use [`sensor_msgs/msg/CompressedImage`](https://github.com/ros2/common_interfaces/blob/jazzy/sensor_msgs/msg/CompressedImage.msg); rosbridge represents its `uint8[]` data as base64 JSON. The browser transport currently sends each topic message or service call as one JSON WebSocket message; it does not chunk or fragment large payloads. Keep the encoded message below the configured rosbridge message-size limit. Array-valued scans grow with the number of returns, and base64 expands image data, so reduce rate, resolution, or payload size when needed. Large camera frames, point clouds, or raw radar matrices may need a different transport. Keep acquisition, validation, and device configuration in the sensor's ROS driver. Keep sensor-specific rendering in the browser view that owns it. Service payloads use the declared ROS service request and response fields.

`RosbridgeClient` accepts `{ url, topics, services, validators }`. It exposes `start`, `stop`, `subscribe`, `unsubscribe`, `onChange`, `snapshot`, `publish`, and `callService`. `publish` throws on an undeclared direction, invalid JSON object, disconnect, or send failure. `callService` returns a promise for response values and rejects on failure or timeout. The manifest bounds the managed launch configuration. It is not a complete authorization boundary because installed rosbridge versions can still expose `/rosapi/*` services. Bind rosbridge to loopback and use the SSH tunnel.

The browser shows raw values or `log1p` values. Its color scale can follow each frame or use a fixed maximum. These display choices do not change the ROS message. The axes show bin indices because physical range, speed, and orientation are unconfirmed.

For a Python client on the PC, install `roslibpy` in a virtual environment. This example uses the same SSH tunnel as the browser:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install roslibpy==2.1.0
```

```python
import roslibpy
from threading import Event

client = roslibpy.Ros(host="127.0.0.1", port=9090)
client.run()
topic = roslibpy.Topic(
    client,
    "/hmmd/rdmap",
    "hmmd_interfaces/msg/RangeDopplerMap",
    throttle_rate=100,
    queue_length=1,
)
topic.subscribe(lambda message: print(
    message["doppler_bins"], message["range_gates"],
    len(message["amplitude_squared"]), message["header"]["stamp"],
))
wait = Event()
try:
    while client.is_connected:
        wait.wait(1)
finally:
    topic.unsubscribe()
    client.terminate()
```

The PC client does not need ROS or generated HMMD message packages. Keep the process open while you receive messages. Call `topic.unsubscribe()` and `client.terminate()` when the client exits.

## Record and replay data

Record the three planned scenes while the sensor node is running: an empty scene, a stationary person, and a moving person. The example below records the empty scene:

```bash
ros2 bag record -o hmmd-empty /hmmd/rdmap /hmmd/status
```

Stop the recorder with Ctrl+C. Use `hmmd-stationary` and `hmmd-moving` for the other two bags. Check each bag with `ros2 bag info` and confirm that it contains map messages. Add the setup, position, duration, rate, and observations to [HMMD observations](../../../agent_notes/cm5/sensors/hmmd/observations.md).

For replay, stop the live sensor node first. In one sourced terminal on the CM5, start rosbridge with the same loopback address and topic limits as the managed launcher:

```bash
mapfile -t BRIDGE_FILTER_ARGS < <(python3 scripts/sensor_channel_config.py \
  src/cm5/ros2/src/hmmd_radar/web/endpoint-manifest.json --format shell)
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=127.0.0.1 port:=9090 \
  "${BRIDGE_FILTER_ARGS[@]}"
```

In another CM5 terminal, start the static server from the project root:

```bash
cd ~/Navbot-ES02-cm5-hmmd
python3 -m http.server 8080 --bind 0.0.0.0 --directory src/cm5/ros2/src/hmmd_radar/web
```

On the PC, forward both CM5 loopback ports and open `http://127.0.0.1:8080/`:

```bash
ssh -N -L 127.0.0.1:8080:127.0.0.1:8080 -L 127.0.0.1:9090:127.0.0.1:9090 cm5
```

The helper rejects wildcard names and malformed endpoint declarations. It generates the filters used by the managed launcher. The service filter still leaves rosapi operations available, so bind rosbridge to loopback and use the SSH tunnel. Keep rosbridge and the static web server running, then replay a bag in a sourced ROS terminal:

```bash
ros2 bag info hmmd-empty
ros2 bag play hmmd-empty
```

Open the browser through the SSH tunnel. Pause playback and let it end to check that the map becomes stale. Do not use `scripts/start_hmmd.py` for replay because it starts a live sensor node.

## Current status and remaining checks

The ROS packages, parser, browser, and unified launcher are implemented. The local checks use synthetic serial data. The existing evidence records a real HMMD stream and browser access on the CM5, but the current worktree's ROS build and the planned three-scene bag recording and replay have not been verified here. The physical carrier, wiring, supply voltage, and sensor firmware remain unconfirmed. See the [dated observations and test evidence](../../../agent_notes/cm5/sensors/hmmd/observations.md).

Before treating the roadmap milestone as complete, verify the current CM5 checkout and build, confirm the physical UART and voltage, record and compare all three scenes, and replay a bag without the live sensor. Keep the heatmap axes in bin indices until physical scaling and orientation have evidence.
