# Reusable sensor channel handoff

Implementation is complete and committed in the HMMD radar worktree as `4a8e854` (`feat: generalize HMMD browser sensor channel`). Local checks and the synthetic browser interaction passed. No push, PR, deployment, or hardware operation was performed.

## User requirements

Original request, verbatim:

> i want this to be reused in the future. i imagine a general use send and receive channel, where different kinds of sensors like radar, lidar, camera can send and receive data. take the current implementation as a starting point and enable it to be more general use without rewriting everything.

Explicit amendments, verbatim:

> i do like the browser interaction, so keep that

> a lightly standardized and documented interface should be used if possible

The current pause request is to document findings and plans now so a smaller model can continue. It is not cancellation of the development task.

## Durable planning artifacts

The Scope Contract and frozen plan are in `dev_notes/scope-sensor-channel.md`. Architecture grounding, rubric and synthesis are in `dev_notes/design-sensor-channel.md`. Progress checklist is in `dev_notes/todo-sensor-channel.md`. All are in the existing HMMD worktree, not the isolated implementation checkout. These artifacts preserve the scope wrapper and should be reused.

## Repository and checkout state

- Existing radar checkout: `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/.worktrees/hmmd-ros2-rdmap`, branch `codex/hmmd-ros2-rdmap`.
- Task baseline: `0479bef25fca10c36ed98864d08302b777a85702`. The implementation is now present as working-tree changes in the existing radar checkout.
- Isolated implementation checkout: `/tmp/navbot-sensor-channel`, branch `codex/general-sensor-channel`.
- WIP commit: `44c032c`, message `wip(sensor-channel): preserve interrupted manifest refactor`. The isolated checkout's later completed files remain in its working tree. The final implementation was copied into the radar worktree; do not treat the WIP commit alone as the finished change.
- Pre-existing untracked files in the radar checkout were developer notes under `dev_notes/`, `dev_reference/plans/`, and `work/`. Preserve them. Do not include unrelated notes in task commits.
- The original implementation worker stopped at its usage limit; the owner independently reviewed and completed the change. A fresh Luna scope review returned `STATUS: OK` after a radar example and payload limits were added to the guide.
- No remote deployment, firmware flash, push, or PR occurred. The local synthetic web/rosbridge fixture was used for browser verification and then stopped.

Although the implementation checkout is under `/tmp`, its WIP commit lives in this repository’s common Git storage. Do not delete the checkout during a model switch. If a later reboot removes it, recover the branch with normal worktree tooling after checking `git worktree list`.

## Findings from the existing code

1. `src/cm5/ros2/src/hmmd_radar/hmmd_radar/serial_session.py` owns the HMMD UART and bounded frame parser/reconnect behavior. `sensor_node.py` publishes `/hmmd/rdmap` as `hmmd_interfaces/msg/RangeDopplerMap` and `/hmmd/status` as `diagnostic_msgs/msg/DiagnosticArray`. Keep this device ownership. No sensor-driver or firmware changes are needed for this task.
2. Existing browser `web/rosbridge_client.mjs` owns the WebSocket, retries, subscriptions, and latest-message snapshots. It unnecessarily imports HMMD topic names, its matrix validator, fixed active topics, and map-versus-status freshness logic. This is the main reuse seam. The baseline source is available through read-only `git show` if the file is absent in the WIP checkout.
3. `web/app.mjs` owns the HMMD heatmap, raw/log color scaling, status badge interpretation, and a generic JSON topic inspector. Preserve these browser interactions. `web/model.mjs` has the HMMD matrix validator and reshaping. Inject that validator into the generic client from the view, or validate at the view boundary while preserving the last valid sample.
4. Existing `web/topic_registry.mjs` contains only the two HMMD receive endpoints. `scripts/start_hmmd.py` separately hardcodes those endpoint names and rosbridge permissions twice, for launch and reuse validation. A shared ordinary JSON manifest removes this duplication without replacing ROS or the existing static HTTP server.
5. The existing client has no publish/service path. The current bridge startup denies publish topics and non-rosapi services. The node’s normal `rcl_interfaces/srv/GetParameters` service can demonstrate an actual read-only request/response; it reads ROS node parameters, not HMMD hardware configuration. Source parameters are `port`, `baud_rate`, `poll_period_sec`, `stale_timeout_sec`, and `diagnostics_period_sec`, all read-only at startup.
6. ROS data is already appropriate for future sensors. Prefer standard `sensor_msgs/msg/LaserScan` for LiDAR and `sensor_msgs/msg/CompressedImage` for camera examples. Keep the HMMD’s existing custom matrix type. No proprietary sensor wire envelope or schema language is needed.
7. Physical HMMD range/Doppler scaling and hardware config command/ACK behavior are still unverified. This task adds the reusable browser channel, not those device commands.

## Agreed architecture

Keep the existing `RosbridgeClient` and extend it in place. Use documented ROS 2 topics for streams, ROS 2 services for request/response, and rosbridge JSON operations over the existing WebSocket. The public API should be small:

```js
new RosbridgeClient({ url, topics, services, validators });
client.start();
client.stop();
client.subscribe(topicName);
client.unsubscribe(topicName);
client.onChange(listener);
client.snapshot();
client.publish(topicName, jsonObject);
await client.callService(serviceName, jsonObject);
```

The constructor receives declarations explicitly and has no HMMD-specific imports, defaults, selected-topic policy, or sensor branches. The page owns its selected topic and subscribes to pinned endpoints plus that selection. Incoming samples carry receipt time, connection generation, and freshness derived from endpoint metadata. Keep only the latest sample per declared receive topic.

One `web/endpoint-manifest.json` is read by the browser and startup. Flat topic records have `name`, `type`, `label`, `direction` (`subscribe`, `publish`, or `both`), `reliability`, `throttleMs`, `staleAfterMs`, `pinned`, and optional `defaultPayload`. Service records have `name`, `type`, `label`, `timeoutMs`, and optional `defaultPayload`. Do not introduce duplicate endpoint IDs, sensor grouping, an adapter framework, or a JSON-schema engine.

The production manifest declares the current two HMMD subscriptions with their current timing/QoS plus `/hmmd_sensor/get_parameters` of type `rcl_interfaces/srv/GetParameters`. There is no production publish endpoint yet. A test fixture declares a second synthetic sensor’s receive/publish/service capabilities to prove the reusable outgoing path. Its endpoints must remain absent from the production manifest.

Client invariants:

- Resolve names against declared capabilities before sending. Reject wrong direction, disconnected state, invalid/nonserializable JSON, and socket send failure.
- Advertise known topic type/QoS once per connection generation before publication. Report publication only as submitted to the bridge; there is no positive delivery acknowledgement. Route correlated rosbridge status errors to endpoint errors shown in the UI.
- Service calls use unique correlation IDs and a pending-request map with a finite timeout. Match both request ID and expected service, and resolve only successful well-formed `service_response` values.
- Reject pending requests on server refusal, malformed matched response, timeout, disconnect, stop, or send failure; clear their timers/state once. Ignore late replies.
- Reconnect resubscribes active receive topics. Never queue or replay publications/service calls.
- Keep HMMD payload validation in its model/view or an injected type-validator map. Other sensor payloads remain opaque ROS JSON.

Startup adds `scripts/sensor_channel_config.py` to validate the manifest and derive deterministic `topics_sub_glob`, `topics_pub_glob`, and `services_glob` values. Reject wildcard/glob ROS names and malformed types. Use the same resulting argument array for launch and checking reused bridge processes. Keep bridge loopback binding and existing SSH tunnel behavior. No shell `eval`. Fail an invalid manifest before starting services.

Browser changes retain the heatmap, status, topic inspector, style and interaction. Populate a generic outgoing endpoint selector and JSON payload editor from the manifest, with send/request button and clear pending/submitted/response/error feedback. Show manifest-load errors. Use `textContent` for displayed payloads. Keep the existing page title unless required changes are reflected in startup identity checks.

## Design review completed

The how exploration traced source ownership and extension seams. Three independent Luna architect candidates were compared, and a separate Luna cross-judge selected Candidate 1 as the smallest design. Graft Candidate 2’s lifecycle/second-sensor testing and documentation checklist. Reject its extra adapter/schema engine and Candidate 3’s sensor grouping. Model the Domain led to endpoint records and maps for latest samples/pending requests. Laziness Protocol led to extending the one existing client instead of adding transport layers.

Candidate artifacts remain at `/tmp/sensor-channel-design-1.md`, `/tmp/sensor-channel-design-2.md`, and `/tmp/sensor-channel-design-3.md`. Their essential decisions are recorded above and in the design note, so continuation does not depend on those temporary files.

Required Gemini CLI review was attempted once using the configured adapter. It returned `ok:false`, `deadline_exceeded`, after 90 seconds. Gemini was unavailable and did not participate or agree. Result is `/tmp/sensor-channel-gemini-result.json`. Do not rerun solely because the model switched.

## Completed implementation

The existing browser client is now generic over manifest-declared ROS topics and services. The HMMD heatmap, status view, and topic inspector remain. The same endpoint manifest drives the browser controls and the managed launcher's bounded rosbridge filters. Production exposes HMMD map/status receive topics and a read-only GetParameters service; synthetic-only range/publish/echo endpoints are confined to the test fixture. The guide documents a future `radar_msgs/msg/RadarScan` example, LiDAR/camera message options, and the current JSON payload-size limitation.

The completed changes are in `src/cm5/ros2/src/hmmd_radar/web/`, `scripts/start_hmmd.py`, `scripts/sensor_channel_config.py`, the corresponding client/launcher tests and synthetic fixture, and `docs/cm5/software/hmmd-ros2.md`. The HMMD-specific validator is injected by the view. Outgoing values are strict JSON objects; services have bounded timeout/correlation handling; outgoing calls are never replayed after reconnect.

The obsolete `web/topic_registry.mjs` was removed as part of the caller migration. The client uses rosbridge's current `call_service` fields and endpoint QoS for topic advertisements.

## Verification and limits

Commands run from the radar worktree:

- `node --test src/cm5/ros2/src/hmmd_radar/test/web/client.test.mjs` — passed.
- `python3 scripts/test_start_hmmd.py -v` — 14 tests passed.
- `PYTHONPATH=src/cm5/ros2/src/hmmd_radar python3 -m unittest discover -s src/cm5/ros2/src/hmmd_radar/test -v` — 11 tests passed; the ROS graph test skipped because ROS 2 is unavailable.
- JavaScript/Python compilation, startup bash syntax, and `git diff --check` — passed.
- Synthetic browser check — HMMD map/status remained visible, a declared topic publish was submitted, and the synthetic service returned a response.

The browser check used synthetic ROS messages, not the physical radar. Full ROS graph integration, CM5 deployment, and hardware behavior remain unverified and outside the frozen task.

`ss` in the sandbox reported netlink access denied. Local simulator listening sockets might require automatic approval escalation; do not interpret sandbox device/network restrictions as hardware absence.

The first attempt to stage from `/tmp/navbot-sensor-channel` could not write the shared Git index in that checkout. The requested commit succeeded from the existing radar worktree. Unrelated pre-existing untracked notes remain untouched.

## Next planned task

The user next requested agent instructions for runtime radar configuration. See `dev_reference/plans/hmmd-radar-config-agent-instructions.md`. It separates the documented sensor-channel baseline from the still-unverified HMMD serial write protocol and requires evidence before exposing a configuration write.

Existing docs record rosbridge 2.7.1’s extra `/rosapi/*` service allowance and action-filter launch limitation. Do not claim the manifest makes rosbridge a complete authorization boundary. Retain loopback plus SSH. Do not broaden this task into unrelated security refactoring.

## Official protocol evidence

Primary protocol source consulted: `https://raw.githubusercontent.com/RobotWebTools/rosbridge_suite/ros2/ROSBRIDGE_PROTOCOL.md`.

- `subscribe` carries topic/type/throttle/queue/QoS.
- `advertise` plus `publish` handles topic sending.
- `call_service` carries `id`, `service`, `args`, and optional timeout in seconds.
- `service_response` carries matching `id`, `service`, boolean `result`, and object `values` for successful calls.
- `uint8[]` arrays such as compressed images appear as base64 strings in JSON.

Use a lightweight documented ROS interface. This task excludes automatic discovery, new device drivers, HMMD hardware config writes, binary/fragmented/video optimization, compatibility aliases, remote deployment, and firmware/motor changes.
