# HMMD radar configuration design

## Caller usage

The radar page shows a small settings area next to or directly below the heatmap. On first connection it reads the confirmed values. Each row has one integer draft and an Apply button:

- **Maximum distance gate**, 0–15 gates. Help: “Limits the farthest distance gate used for detections.”
- **Target disappearance delay**, 0–65535 seconds. Help: “How long a target remains reported after it disappears.”

The page keeps draft values separate from device-confirmed values. A displayed value changes only after the corresponding setting is read back. Save acknowledgment is shown as a separate fact and does not imply persistence across power loss. Bridge timeout/disconnect or device timeout leaves the last confirmed value visible but stale until a fresh read.

```js
const settings = new RadarSettings(client);
await settings.read(); // both confirmed device values
await settings.set("maximumDistanceGate", draft);
await settings.set("targetDisappearanceDelaySeconds", draft);
```

The UI and ROS boundary reject invalid integers and ranges. Only one setting is written per operation, so a stale draft cannot rewrite the other setting.

## Ownership and module map

- `hmmd_interfaces` defines two typed ROS services: one coherent read of both settings and one write of exactly one selected setting.
- `hmmd_radar.protocol` privately encodes verified firmware v1.6.1 commands and decodes command replies; the existing map parser remains specialized for map frames.
- `SerialSession` remains the only UART reader and writer. It exclusively coordinates command replies and map frames during a bounded transaction, retaining complete interleaved maps in a bounded buffer for publication when the service returns.
- `HmmdSensorNode` validates service inputs, maps session outcomes into typed responses, and publishes any map frames collected by the session. Keep the current single-threaded executor; do not add a lock or second reader.
- The production endpoint manifest declares only the two new services. `RosbridgeClient.callService` remains the browser transport.
- `web/radar_settings.mjs` hides ROS selector/outcome details behind `read()` and `set(setting, value)`. `app.mjs`, `index.html`, and `style.css` render two controls near the radar image with draft, confirmed, pending, stale, and error states.
- Extend the existing protocol/session/ROS and browser fixture tests for observable contracts. Update `docs/cm5/software/hmmd-ros2.md` and `agent_notes/cm5/sensors/hmmd/protocol.md` with the service behavior and actual hardware evidence.

## Service contract

`GetRadarConfig.srv` returns both settings only when both device reads are valid, plus explicit outcome/stage and concise detail. `SetRadarSetting.srv` accepts a closed setting selector and one integer value. The driver performs write → positive ACK → same-setting readback → exact match → save → positive save ACK. It returns separate write/save ACK flags, observed value when available, and named outcome/stage. A response is successful only when the readback matches and save is acknowledged. It must not claim power-cycle persistence.

Reject unknown selectors, booleans, non-integers, and out-of-range requests before serial I/O. A write timeout is uncertain; never retry it automatically, and require a fresh read before another write. Keep per-stage and total deadlines finite, with the service/manifest timeout longer than the session transaction. Drop late command replies rather than associating them with a subsequent operation.

## UART/map transaction

The current single-threaded `rclpy.spin()` serializes polling and service callbacks. During the synchronous service callback, `SerialSession` itself drains bounded chunks and classifies both map and command frames; it routes only the matching reply to the active step and retains complete map frames in a bounded buffer for the next publish. This briefly pauses ROS map publication but does not silently discard interleaved map frames. Keep one outstanding device command. Do not retry after I/O loss because the device may have applied the write before disconnection.

The hardware evidence covers firmware `v1.6.1`, maximum gate `12`, and disappearance delay `30 s`: each unchanged value was written, positively acknowledged, read back, and followed by a save acknowledgment. This does not establish power-cycle persistence or authorize changing either value in this task.

## Synthesis decision

Use candidate 1's coherent snapshot read and per-setting write services. Graft candidate 3's domain-oriented browser wrapper so page code does not manipulate ROS selectors or numeric result codes. The read/set transaction stays in one `SerialSession`; candidate 2's paired-value write is rejected because a one-row edit could rewrite a stale value. Candidate 3's generic operation/setting service is rejected because it permits meaningless field combinations and makes stage reporting less precise. The cross-judge found no red-flag blocker and favored this shape for correctness and scope.

The Gemini CLI review did not run successfully: its adapter rejected the supplied task name before reviewing any design. No Gemini findings are attributed to this decision.

## Tradeoffs and open risks

- Keep synchronous bounded transactions and accept a short map publication pause; revisit the ownership model only if measured device latency exceeds the budget.
- Keep explicit browser and ROS validation at their external boundaries and validate all protocol replies before reporting them.
- A timeout after a write is unknown outcome, not proof the setting stayed unchanged. Refresh before another write.
- A save ACK confirms command acceptance only; power-cycle persistence remains untested.
- Preserve maps received during an exchange in a bounded queue; no unbounded buffering.
- Verify the service timeout hierarchy and late-reply drain behavior against the actual serial-session implementation and synthetic streams before enabling the browser control.

## Verification contract

Exercise only no-change hardware writes at gate `12` and delay `30 s`; no deployment or value change. Software checks cover exact boundaries and invalid values, command/reply correlation, ACK/readback/save outcomes and timeouts, interleaved map retention, bounded buffering, no automatic retry, ROS response mapping, manifest-derived endpoints, and browser confirmed-only/reconnect behavior. Existing map/status operation must remain available.

## Implementation reconciliation

The v1.6.1 read replies contain the command ID and uint32 value but do not echo the parameter ID. The serialized session therefore associates a reply only with its one outstanding read step; it drains late command replies between steps and quarantines a timed-out command ID before reusing it. It never retries a setting write. The measured read sequence enters config mode, reads parameter IDs 1 and 4, and sends the acknowledged `0x00FE` exit command. A snapshot succeeds only when both reads and that exit ACK succeed. A setting write stays blocked after an uncertain write or readback mismatch until a successful fresh snapshot.

The mixed stream parser retains the existing map decoder for complete map frames and adds command-frame classification in the same bounded UART stream. Complete maps seen during a service transaction wait in a bounded 64-frame queue for the next poll; queue overflow is counted in HMMD diagnostics. Command stages have finite 350 ms response deadlines and a short bounded late-reply drain, below the 5 s browser service timeout. No second UART reader or executor was introduced.

If later measurements show the finite synchronous transaction exceeds the service budget, stop and return to design/scope review before changing ownership or concurrency.
