Original Outcome (verbatim): Das soll aufgeräumt werden. Ideal wäre nur ein einziges Skript, was man auf dem CM5 oder Remote aufrufen kann. Falls das zu aufwendig ist, kann Remote auch ein Startskript auf dem CM5 aufrufen, was den Sensor dann startet. Falls es eine einzige Datei sein kann, stelle ich mir vor, dass das entweder automatisch gehandelt wird, ob der Server Remote läuft oder nicht, oder dass man einstellt, zum Beispiel den SSH-Zugang mitgibt oder so ähnlich, um dem Programm mitzuteilen, dass es sich erst auf einen anderen PC verbinden muss, um den Server zu starten. Suche dafür die beste Lösung, damit es ein sauberes Ding ist und auf dem CM5 lokal funktioniert, aber auch Remote über ein lokales Netzwerk.
Source: user message in current chat, 2026-10-05.
Acceptance Criteria: One canonical startup implementation works locally on CM5 and from PC via explicit SSH target across LAN. Starts/reuses sensor, rosbridge and static web server with existing identity/UART conflict checks. Remote browser connection works via loopback SSH tunnel. CLI makes target/workspace/lifecycle clear. Retire superseded startup scripts/launch entry without aliases. Relevant docs and behavioral checks match new interface.
Approved Approach: Compare three architecture sketches, select the smallest single-file design where viable, consolidate startup without changing sensor/browser protocols, verify local and SSH dispatch plus conflict/reuse/error behavior using local test tools. Use optional host read-only inspection/live checks only if a configured reachable CM5 path is available; avoid unrequested deployment changes.
Explicit Non-Goals: Firmware/hardware changes, sensor or browser feature changes, systemd or boot services, new dependencies, password storage, compatibility aliases, unrelated cleanup, modifying earlier desktop-removal diff, commits or PRs.
Compatibility / Migration Requirements: NONE
Baseline: HEAD 06a5c7e plus pre-existing desktop removal tracked diff saved at /tmp/hmmd-unified-baseline.patch.
Pre-existing status:
 M docs/cm5/software/how-to/hmmd-ros2.md
 M docs/cm5/software/reference/hmmd-ros2.md
 D src/cm5/ros2/src/hmmd_radar/hmmd_radar/heatmap.py
 D src/cm5/ros2/src/hmmd_radar/hmmd_radar/view_model.py
 D src/cm5/ros2/src/hmmd_radar/hmmd_radar/viewer_node.py
 M src/cm5/ros2/src/hmmd_radar/package.xml
 M src/cm5/ros2/src/hmmd_radar/setup.py
 D src/cm5/ros2/src/hmmd_radar/test/test_heatmap.py
 M src/cm5/ros2/src/hmmd_radar/test/test_ros_integration.py
 D src/cm5/ros2/src/hmmd_radar/test/test_view_model.py
?? dev_notes/decision-trail-hmmd-pc-data-agent.md
?? dev_notes/scope-hmmd-desktop-removal.md
?? dev_notes/scope-hmmd-pc-data-agent.md
?? dev_notes/scope-hmmd-remote-autostart.md
?? dev_notes/todo-hmmd-desktop-removal.md
?? dev_notes/todo-hmmd-remote-autostart.md
?? dev_reference/plans/hmmd-j8-next-step.scope.md
?? dev_reference/plans/hmmd-j8-next-step.todo.md
?? dev_reference/plans/hmmd-main-merge-scope.md
?? dev_reference/plans/hmmd-ros2-design.md
?? dev_reference/plans/hmmd-ros2-scope.md
?? dev_reference/plans/hmmd-ros2-todo.md

Grounding: Current PC launcher scripts/start_hmmd_web.py carries one embedded REMOTE_STARTUP Bash transaction. It sources Jazzy/workspace, flock serializes starts, validates proc/UART ownership and ROS status, starts/reuses expected sensor/rosbridge/static HTTP. Python handles SSH and local loopback9090 tunnel/browser. Shell sensor launcher separately sources ROS and starts sensor foreground; ROS sensor launch offers third configuration. The previous remote-autostart note says deployed CM5 checkout lacked shell launcher, explaining duplicated starts. Existing rosbridge loopback restriction and topic filters must remain. Services currently persist after PC tunnel exits. Single-file target can reuse identical startup body locally and over SSH; remote cannot require unpublished files implicitly.
Frozen implementation plan:
1. Replace scripts/start_hmmd_web.py with one scripts/start_hmmd.py. No --ssh means local CM5; --ssh user@host or SSH alias means remote. Same STARTUP_BASH is executed locally or streamed to remote bash -s. Local workspace defaults to launcher checkout root; remote empty workspace resolves in Bash to $HOME/Navbot-ES02-cm5-hmmd; explicit --workspace overrides. Preserve existing Bash identity/UART/readiness/lock logic, loopback rosbridge and HTTP root/filter policy. Preserve numeric device/baud/timeout options, optional --no-browser for headless use, validate SSH destination at boundary.
2. Remote mode preflights and forwards both loopback8080 and9090 so both modes open http://127.0.0.1:8080/. Local mode requires only bash, runs startup and exits after printing/opening browser. Remote mode requires ssh, waits for both forwards, keeps tunnel foreground and cleans it up on interruption/failure. Services persist in both modes.
3. Delete scripts/start_hmmd_sensor.sh and hmmd_radar/launch/sensor.launch.py; remove launch-file data entry and unused launch/launch_ros package dependencies. No compatibility aliases. Raw ros2 run remains available for diagnostics, not a managed startup alternative.
4. Update docs/cm5/software/how-to/hmmd-ros2.md and reference/hmmd-ros2.md with one CLI, local/SSH defaults, both-forward lifecycle, standalone PC file use, correct replay manual path, prerequisites, and startup test command. Preserve previous desktop removal.
5. Add scripts/test_start_hmmd.py with focused behavior tests for target validation, safe argument transport, shared local/SSH payload, ports, startup failures, timeout and tunnel cleanup, matching reuse/conflict checks where practical. Verify Python+browser tests and shell syntax, then exercise startup/reuse locally on real CM5 and from PC via SSH without changing remote checkout. Inspect endpoints/status/UART ownership and refusal of mismatched config.
6. Review complete task diff, fix in-scope defects, final independent scope check.
Expected area: two retired scripts and replacement/new test, sensor launch deletion/setup/package edits, two canonical docs, local task notes. Expected extent: one existing ~300-line launcher generalized, ~70 obsolete startup lines removed, focused tests and short docs edits. No service-manager rewrite.
Rubric: single startup owner/no deployed-script dependency; explicit usable local/remote CLI and workspace resolution; preserved UART/process identity/reuse safety; small maintainable implementation; demonstrable failure/lifecycle behavior.
Checkpoint: Three architecture candidates compared and owner implementation completed.

Synthesis decision: A scored5/5/5/4/5, B5/4/3/2/4, C5/4/5/4/5 in independent Luna cross-judge. Select A shared startup body and optional --ssh; graft C target validation and preflight for both forwarded ports. Reject full Python service rewrite and remote-helper version dependency. All three use Luna per project policy; no model diversity claimed. Gemini cross-judge pending.

Implementation amendment: In live verification, listener inspection after interrupt showed both CM5 services persisted while immediate local port binding transiently rejected a port with no listening socket. The owner reproduced the state: no TCP listener remained, and the same port became bindable after connection cleanup. An ephemeral local TCP test reproduced `TIME_WAIT=true`: plain bind returned `EADDRINUSE`, while `SO_REUSEADDR` preflight accepted the port. The actual PC-to-CM5 launcher succeeded, received Ctrl+C, and immediately succeeded again with Sensor, rosbridge, and Web reported as reused. Accepted fix: set `SO_REUSEADDR` on local preflight sockets and assert each port receives it.

Final verification: Local CM5 mode ran the shared payload without copying files into the remote checkout and started Sensor, rosbridge, and Web. The PC SSH mode then reused all services, forwarded both ports, and the production `RosbridgeClient` connected to the browser endpoints, fetched the page/assets, and received a fresh 20×16 map with 320 values plus `connected=true`, `stale=false`. A 9600 baud mismatch was refused before another UART reader started; PID 2911 remained the sole `/dev/ttyAMA0` owner. Port-occupied preflight returned 1. After Ctrl+C, services remained on CM5 and PC tunnel ended. A second immediate remote start after Ctrl+C succeeded and reused all three services. Latest Python runner tests: 12 passed. Package suite: 10 passed, 1 skipped for absent local ROS; browser Node suite passed; Bash syntax, CLI help, metadata, docs searches and whitespace check passed. Read-only review: no concrete defects/no comment flags. Cross-model Gemini timed out; no Gemini judgement accepted.

Final independent scope check: STATUS OK. Required implementation and files follow frozen plan; no questionable or out-of-scope changes, no unapproved compatibility measures, no parked issues. Read-only review and comment check found no defects or MUST KILL flags. Gemini timed out and supplied no verdict.
