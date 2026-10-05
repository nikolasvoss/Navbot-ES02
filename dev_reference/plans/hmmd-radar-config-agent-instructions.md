# Agent instructions: runtime HMMD radar configuration

## Objective

Add a safe, browser-based way to inspect and change HMMD radar settings while the sensor node is running. Extend the committed sensor channel (`4a8e854`) and keep its browser interface. Do not implement firmware changes, other sensor drivers, or a second transport.

Start by reading the repository `AGENTS.md`, `docs/cm5/software/hmmd-ros2.md` (especially **Sensor-channel interface baseline**), `agent_notes/cm5/sensors/hmmd/README.md`, `agent_notes/cm5/sensors/hmmd/protocol.md`, and `agent_notes/cm5/sensors/hmmd/observations.md`. Before changing serial behavior, also read `agent_notes/robot/usb-serial.md` and `docs/robot/software/development/usb-serial-flashing.md`. Follow the development workflow and create a fresh Scope Contract for this configuration task before implementation. Preserve unrelated worktree files.

## Evidence that defines the starting point

- The HMMD node currently declares `port`, `baud_rate`, `poll_period_sec`, `stale_timeout_sec`, and `diagnostics_period_sec` as read-only ROS parameters. See `src/cm5/ros2/src/hmmd_radar/hmmd_radar/sensor_node.py`.
- `SerialSession` owns the UART and sends a debug-mode initialization byte sequence when opening the port. It currently parses map frames only; it has no request/response command path. See `serial_session.py` and `protocol.py`.
- The production browser manifest exposes map/status receive topics and read-only `/hmmd_sensor/get_parameters`. The generic channel now supports declared ROS services, so hardware configuration should be owned by the HMMD driver and exposed as an explicit service. See `src/cm5/ros2/src/hmmd_radar/web/endpoint-manifest.json` and the interface baseline in the guide.
- The locally saved Waveshare guide says **Maximum Distance Gate** is 0–15, with one gate described as 70 cm, and **Target Disappearance Delay Time** is 0–65535 seconds. These are manufacturer statements, not confirmed measurements on the installed module. The same guide documents a read-configuration command `0x0008`; the local protocol note explicitly says the complete initialization sequence and ACK behavior are unconfirmed. The available guide text does not establish a write command, parameter IDs for these settings, persistence behavior, or read-back semantics.
- In the published map, `range_gates` is a dimension of 16 and a `range_gate` index is 0–15. Do not assume that the UI's array index, the manufacturer's “Maximum Distance Gate” value, and a physical distance are interchangeable. Their mapping, inclusive/exclusive behavior, and effect on the matrix need evidence.

Sources: `agent_notes/cm5/sensors/hmmd/vendor/HMMD-waveshare-wiki.txt`, `agent_notes/cm5/sensors/hmmd/protocol.md`, and the current driver files. Keep manufacturer claims, code behavior, user reports, and physical measurements distinct.

## Evidence gate: establish writable device behavior

Before adding a serial write path, determine from authoritative protocol material and, if available, a controlled device observation:

1. The exact module and firmware revision.
2. The command frame for each supported read and write, including command ID, parameter ID, byte order, value width, units, bounds, and checksums or framing rules.
3. Positive and negative acknowledgement formats, request correlation, timeout behavior, and whether the device echoes or supports reading back the applied setting.
4. Whether a setting applies immediately, needs a mode transition or restart, and persists across power cycles.
5. How command replies interleave with the continuous debug data stream and how to recover from a partial reply or timeout.

Do not infer a write opcode from `0x0008` (documented as read), reverse engineer a command by sending guesses, or report a setting as applied based only on rosbridge submission. If the write protocol cannot be verified, stop short of an enabled hardware write UI: finish the read-only discovery if useful, record the exact missing evidence, and return a small next-step request. A manufacturer GUI screenshot alone does not prove command bytes or ACK semantics.

For any direct serial observation, check `list_ports` through the serial MCP first, establish which process owns the UART, and do not start a second `hmmd_sensor` on the same device. Keep such observations read-only unless the user has approved the specific hardware write and its test value. Record new findings in `agent_notes/cm5/sensors/hmmd/observations.md` with date, evidence, source type, and practical consequence; update `protocol.md` only with supported protocol facts. Do not flash firmware.

## Implementation boundaries

- Keep one serial owner. `SerialSession` should serialize requests with frame polling; no browser, ROS callback, or second node may write directly to the UART.
- Keep transport mechanics generic in the sensor channel. Put HMMD command encoding, decoding, valid-setting metadata, and hardware acknowledgement handling in the HMMD package/driver.
- Prefer a small typed ROS service contract for get/set operations. Use existing standard ROS parameter services only if the value is truly a runtime ROS parameter and the driver can reject a change without claiming it succeeded. Do not tunnel JSON command strings through `std_msgs/String` or mark startup-only parameters writable to bypass the hardware protocol.
- Register only the exact configuration services in `endpoint-manifest.json`. Keep the generated bridge allowlist, loopback bind, and SSH tunnel. Do not add a generic command topic or automatic endpoint discovery.
- A set response must distinguish at least: request validation failure, device rejection, timeout/no acknowledgement, acknowledged application, and (if available) read-back confirmation. State clearly when the device acknowledges a command but cannot confirm persistence.
- The browser should show the current known value, allowed range/unit, request-in-progress state, and final response/error. Do not optimistically replace the displayed current value before the driver response. Preserve the HMMD map/status and generic topic inspector.
- Keep physical-distance conversion out of the UI until gate-to-distance semantics are confirmed for the installed sensor. Label the setting “maximum distance gate” and the map index as an index if that is all evidence supports.

## Suggested first controls

Treat these as candidates, not confirmed writable controls:

| Candidate | Manufacturer statement in local guide | Required confirmation |
| --- | --- | --- |
| Maximum Distance Gate | Integer 0–15; one gate is described as 70 cm | Exact parameter ID and write command; whether the value denotes an index or count; effect on range-map output; device read-back and persistence. |
| Target Disappearance Delay Time | Integer 0–65535 seconds | Exact parameter ID and write command; whether zero is valid; ACK/read-back and persistence. |

Do not expose any other setting unless its name, units, bounds, command encoding, and effect have equivalent evidence.

## Required tests and documentation

- Unit-test byte encoding/decoding with manufacturer-backed vectors. Cover bounds, endianness, positive/negative ACK, malformed reply, timeout, and unrelated map frames arriving during a transaction.
- Test the serial session with a fake port: exactly one owner, serialized requests, no partial-write success, timeout recovery, reconnect behavior, and continued map parsing.
- Test ROS service outcomes with mocked device responses. An error or missing device ACK must not update the reported applied value or parameter state.
- Extend the synthetic rosbridge fixture and browser check to cover loading current settings, submitting a valid setting, pending state, confirmed result, and every failure state. Fixture commands must remain synthetic and absent from production permissions until implemented.
- Preserve tests for the committed generic channel, HMMD map/status validation, startup allowlists, and no replay of writes after reconnect.
- Update `docs/cm5/software/hmmd-ros2.md` with the actual ROS service contract, fields, ranges, units, ACK/read-back meaning, and limitations. Record hardware evidence separately in `agent_notes/`; do not present vendor values as observed device behavior.

## Completion criteria

The task is complete only when a browser user can read and set each enabled setting through the existing page, the driver reports outcomes based on actual protocol responses, invalid or unsupported values are rejected, failures leave the last confirmed value intact, and tests cover the command and request lifecycles. If the evidence gate fails, deliver the verified investigation and identify the missing protocol evidence instead of shipping an unverified write path.

Before completion, run the repository's applicable tests, inspect the full diff, and obtain a fresh independent scope review. Do not deploy to the CM5 or issue a physical configuration write unless the user has authorized that concrete action.
