# Localization on the saved map

Status: Proposed, engineering review done, not started. Branch `feat/localization` (from `main` at `a14fdc6`).

## Problem

Today the robot only knows where it is on the saved map if it starts at the spot where the
map began: the generated `slam_params.yaml` loads the posegraph with `map_start_at_dock: true`
and keeps mapping. If the robot is switched on anywhere else, the map and the robot's pose
are wrong, and continued mapping then damages the saved map.

## Outcome

With a saved map, the robot works out its own position and heading on that map, with no
start hint, and then tracks it while it drives. The web hub shows the pose and whether the
localization is confident. The saved map does not change during localization.

## Decisions

| Question | Decision | Source |
| --- | --- | --- |
| How the first pose is found | Fully automatic, global search over the whole map | User, 2026-10-28 |
| Behavior after boot | Saved map present → localization. No map → mapping, as today. A web-hub button switches to mapping to extend or remake the map | User, 2026-10-28 |
| Engine | Nav2 AMCL only. slam_toolbox stays the mapping engine | User, 2026-10-28 |

### Engine: why AMCL and not slam_toolbox localization

slam_toolbox localization mode (2.8.5, installed) tracks well from a known pose, but it has
no global search: it matches scans only near the current estimate
(`loop_search_maximum_distance`). Fully automatic start would need a custom global matcher
in front of it, and we would rather not write our own.

Nav2 AMCL (`nav2_amcl` 1.3.13, installed with `nav2_map_server`, `nav2_lifecycle_manager` and
`nav2_bringup`) is the standard ROS 2 localizer and already has what we need:

- `/reinitialize_global_localization` (std_srvs/Empty) spreads particles over all free
  space in the map: this is the global search.
- `/request_nomotion_update` runs a filter update without motion, so a standing, balancing
  robot can still converge on scans alone.
- `/initialpose` accepts a manual hint, if one is ever wanted.
- It publishes `map → odom` TF and `/amcl_pose` with covariance, which gives a confidence value.

The cost is a second map format: AMCL needs the occupancy image (`map.pgm` + `map.yaml`)
next to the serialized posegraph. slam_toolbox writes both formats itself through its
`/slam_toolbox/save_map` service, so this is one extra service call on save, not new code.

slam_toolbox stays the mapping engine. Only one of the two runs at a time, because both
publish `map → odom`.

Planned autonomous driving with Nav2 also favors this choice. The `map.pgm`/`map.yaml` pair
that map_server serves is the static map that the Nav2 global costmap plans on, and Nav2
keepout and speed filter masks are drawn on top of it. The AMCL confidence is the stop
condition for a robot that no longer knows its pose. If AMCL turns out too imprecise in
narrow passages, AMCL can keep doing the global search and hand the pose to slam_toolbox
localization for tracking, without discarding this work.

## Scope

1. **Map export on save.** Saving a map also writes the occupancy image `map.pgm` +
   `map.yaml` next to `map.posegraph` + `map.data`, committed atomically as one set of
   four. Delete removes all four. A saved map without `map.yaml` (saved before this
   change) boots into mapping, and the web hub asks you to save it once.
2. **Localization launch.** The stock `nav2_bringup/launch/localization_launch.py`
   (map_server + amcl + lifecycle manager) with our own `localization.yaml`.
3. **Mode supervision.** One supervised launch at a time: slam_toolbox in mapping,
   map_server + AMCL in localization. The boot rule is in the decision table.
4. **Global localization.** At localization start, and on "Locate again": global
   reinitialization, then forced no-motion updates until the pose converges or a time limit
   passes. After the time limit, the web hub says that the robot should drive or turn a
   little; AMCL keeps updating from motion.
5. **Web hub and health.** Mode, AMCL pose, state `searching` / `localized` / `lost`, and
   standard deviations on `/slam` and `/map.json`. Buttons "Locate again" and "Switch to
   mapping" / "Switch to localization". The health row reflects the active mode.
6. **Tests and lab check.** See the verification plan.

## Engineering review (2026-10-28)

Status: `issues_open` until the user signs off the work-package split below. No open design
question blocks implementation.

### Verified facts the design relies on

Checked against the Jazzy sources and with probes on the ros2_pc:

- `nav2_amcl` 1.3.13, `globalLocalizationCallback`: spreads particles uniformly and sets
  `initial_pose_is_known_ = true`. Without that call or an `/initialpose`, AMCL publishes no
  `/amcl_pose` and no `map → odom` TF. So the service must call
  `/reinitialize_global_localization` after every AMCL start, or the robot never localizes.
- `/request_nomotion_update` sets `force_update_`. The next scan then runs a filter update
  even when the robot has not moved past `update_min_d` / `update_min_a`. It has to be
  called before every scan you want used, about 5 Hz.
- AMCL passes `+inf` ranges through, and the likelihood field model skips every reading
  `>= range_max`. Our `LaserScan` (360 bins, `+inf` for no return) works without changes.
- AMCL sends `map → odom` with expiry `scan stamp + transform_tolerance` (default 1.0 s).
  The existing `update_pose` TF lookup in `ros_node.py` therefore works in both modes.
- `nav2_map_server` writes the `image:` field of the YAML as the **bare file name** given to
  the saver (probe: `-f …/map.saving.123` writes `image: map.saving.123.pgm`). The current
  `MapStore.commit` pattern (save under a temporary base name, then rename) would leave a
  YAML that points to a file that no longer exists. Export into a temporary **directory**
  with the final base name `map` instead.
- `ros2 run nav2_map_server map_saver_cli -f <base> --ros-args -p
  map_subscribe_transient_local:=true` saves the live `/map` (probe passed). slam_toolbox's
  own save_map path was not verified, so the design calls `map_saver_cli` directly.
- map_server publishes `/map` once, transient-local. `ros_node.py` already subscribes with
  transient-local QoS and receives it. But `SlamState.snapshot` reports `stale` after 10 s
  without a new map, and `health.py` turns that into a red "Stale" row. Both must ignore map
  age in localization mode.
- Wheel odometry (`/odom` and the `odom → base_link` TF from `navbot_link_bridge`) works on
  the moving robot (user report, 2026-10-28). AMCL uses it unchanged as its motion model input.
- Test baseline on `feat/localization` (= `main`): 69 passed, 1 failed
  (`tests/scripts/test_start_lidar.py::test_demo_browser_hides_stale_plot_and_reports_server_loss`,
  Playwright, fails before any change).

### Design and flow

```
boot ── MapStore.has_occupancy()? ──yes──▶ localization ──▶ SlamProcess runs localization_command
          │                                   │
          no                                  └─▶ GlobalLocalizer: wait for AMCL services,
          ▼                                         reinitialize_global_localization,
      mapping ──▶ SlamProcess runs slam_command     nomotion updates until converged / timeout
                                                    /amcl_pose ─▶ LocalizationTracker ─▶ /map.json, health
web: POST /slam/mode {"mode"} ─▶ SlamControl.set_mode: stop launch, clear SlamState, start other launch
web: POST /slam/locate ─▶ SlamControl.locate ─▶ GlobalLocalizer.start()
web: POST /slam/save (mapping only) ─▶ serialize posegraph + map_saver_cli into tmp dir ─▶ MapStore.commit
```

Failure paths:

- AMCL launch exits: `SlamProcess` restarts it after 5 s, as today. Every new start must
  trigger global localization again: `GlobalLocalizer` waits for the service with a timeout
  and retries until the mode changes.
- AMCL services do not appear within 60 s: the tracker state is `lost` with an error text,
  and the health row turns `error`.
- Save fails at either step: the temporary directory is removed, the old four files stay.
- Save or locate in the wrong mode: HTTP 409 with a text that says which mode is needed.
- Two concurrent mode switches or a switch during a save: the `SlamControl` lock already
  serializes save, delete, and reload; `set_mode` and `locate` take the same lock.

### Interface contract (all work packages code against this)

New module `lds08rr_lidar/localization.py`:

```python
LOCALIZATION_PARAMS = Path(__file__).with_name("localization.yaml")
XY_STD_MAX_M = 0.15
YAW_STD_MAX_RAD = math.radians(10.0)
STABLE_UPDATES = 5            # consecutive /amcl_pose messages under both limits
SEARCH_TIMEOUT_SEC = 30.0     # then state "lost" with hint "drive or turn a little"
LOST_FACTOR = 2.0             # localized -> lost when a std exceeds LOST_FACTOR x its limit for STABLE_UPDATES

def localization_command(map_yaml: Path) -> list[str]:
    # ["ros2", "launch", "nav2_bringup", "localization_launch.py", f"map:={map_yaml}",
    #  f"params_file:={LOCALIZATION_PARAMS}", "autostart:=true", "use_sim_time:=false",
    #  "use_composition:=False"]

class LocalizationTracker:     # thread-safe, clock injectable like SlamState
    def __init__(self, clock=time.monotonic) -> None: ...
    def start_search(self) -> None: ...                 # state -> "searching", resets counters
    def add_pose(self, x, y, yaw, var_x, var_y, var_yaw) -> None: ...   # from /amcl_pose covariance [0], [7], [35]
    def fail(self, error: str) -> None: ...             # state -> "lost" with error text
    def reset(self) -> None: ...                        # state -> "idle" (mapping mode)
    def searching(self) -> bool: ...                    # True while "searching" and before timeout
    def snapshot(self) -> dict: ...
    # {"state": "idle"|"searching"|"localized"|"lost", "pose": {x,y,yaw}|None,
    #  "std": {"x","y","yaw"}|None, "search_age_sec": float|None, "error": str|None, "hint": str|None}

class GlobalLocalizer:
    def __init__(self, call_service: Callable[[str, float], str | None], tracker: LocalizationTracker,
                 service_timeout: float = 60.0, nomotion_period: float = 0.2) -> None: ...
    def start(self) -> None: ...   # background thread: wait for /reinitialize_global_localization,
                                   # call it, tracker.start_search(), call /request_nomotion_update
                                   # every nomotion_period while tracker.searching(); a new start()
                                   # cancels the running one
    def stop(self) -> None: ...
```

`call_service(name, timeout)` calls a `std_srvs/srv/Empty` service and returns `None` on
success or an error text.

Changes to `lds08rr_lidar/slam.py`:

```python
class MapStore:
    def has_occupancy(self) -> bool: ...     # map.posegraph and map.yaml both exist
    def occupancy_yaml(self) -> Path: ...    # directory / "map.yaml"
    def commit(self, tmp_dir: Path) -> None: ...  # tmp_dir holds map.{data,posegraph,pgm,yaml}; all four
                                                  # required; same backup-and-restore rule as today
    def delete(self) -> None: ...            # removes all four

def export_occupancy(base: Path, timeout: float = 20.0) -> str | None: ...
    # runs map_saver_cli -f base with map_subscribe_transient_local:=true; None or error text

class SlamControl:
    def __init__(self, store, process, serialize, export=export_occupancy, localizer=None, tracker=None): ...
    mode: str                                # "mapping" | "localization"
    def initial_mode(self) -> str: ...       # "localization" if store.has_occupancy() else "mapping"
    def command(self) -> list[str]: ...      # slam_command(...) or localization_command(store.occupancy_yaml())
    def set_mode(self, mode: str) -> str | None: ...
    def locate(self) -> str | None: ...      # 409 text unless mode == "localization"
    def save(self) -> str | None: ...        # 409 text unless mode == "mapping"
    def status(self) -> dict: ...            # adds "mode" and "localization" (tracker.snapshot())

class SlamState:
    def set_map_latched(self, latched: bool) -> None: ...   # True in localization: map age never makes it "stale"
    # snapshot()["state"] in localization: "waiting" (no map) | "no_pose" | "localization"
```

`SlamProcess` keeps its signature. `service.py` passes `command=control.command`, so the
supervisor builds the right command on every start and restart.

### Verification plan

| Behavior | Expected result | Catches | Check |
| --- | --- | --- | --- |
| Boot mode | map.yaml present → localization, else mapping | Booting old maps into a broken AMCL launch | Unit, `SlamControl.initial_mode` |
| Four-file commit | All four replaced together; on failure the old four stay; YAML `image:` is `map.pgm` | The renamed-image bug above | Unit with `tmp_path`, fake export writing real files |
| Export | `map_saver_cli` writes `map.pgm` + `map.yaml` from a published `/map` | Wrong CLI args, QoS mismatch | ROS probe with `ROS_DOMAIN_ID=83`, as done in this review |
| Save refused in localization | 409, files unchanged | Saving the map_server map over the SLAM map | Unit + web test |
| Launch command | Exact argument list | Typos in launch args | Unit |
| Params file | AMCL and map_server come up active with `localization.yaml` and a real saved map | Bad YAML keys, lifecycle not activating | ROS probe on the ros2_pc, `ros2 lifecycle get /amcl` → `active` |
| Global localization calls | Reinitialize once per start, then nomotion updates while searching, stop on convergence or timeout | Never localizing after an AMCL restart | Unit with fake `call_service` and injectable clock |
| Convergence | `localized` after 5 poses under the limits; `lost` after timeout or sustained high std | Thresholds wired backwards | Unit |
| Map not stale in localization | No `stale` state and no red health row with a single latched map | Today's 10 s stale rule | Unit `SlamState`, `health.py` |
| Web endpoints | `/slam/mode`, `/slam/locate` drive the control; cross-site POST gets 403 | Missing guard on new POSTs | Web tests like `test_web_refuses_cross_site_posts` |
| Live accuracy | Found pose within ±0.2 m and ±10° of the true pose at 3+ starts, converged within 30 s or after a short turn | Real-world failure | Lab note on the robot (needs the user) |

Run the suite with
`source /opt/ros/jazzy/setup.bash && source ~/.venvs/navbot-lidar/bin/activate && ROS_DOMAIN_ID=83 python3 -m pytest -q src/ros2_pc/ros2/src/lds08rr_lidar/test tests/scripts/test_start_lidar.py`.
The one Playwright failure listed above is pre-existing and must not get worse.

### Risk register

| Risk | Impact | Mitigation | Tier |
| --- | --- | --- | --- |
| Converges confidently to the wrong place in similar rooms | Wrong pose looks `localized` | Lab check at several starts; click fallback stays an option | Medium |
| No `/reinitialize_global_localization` after an AMCL restart | Robot never localizes after a crash | `GlobalLocalizer` runs on every process start; unit test | Medium, covered |
| YAML `image:` points to a renamed file | Localization launch fails after the first save | Export into a temp directory with base name `map`; unit test checks the YAML | Medium, covered |
| Sparse 5 Hz scans, 8 m provisional range | Slow convergence while standing | Timeout hint to drive or turn; nomotion updates | Low |
| CPU during global search | Scan drops, slow web hub | Measure CPU with `top` during search on the ros2_pc; lower `max_particles` if needed | Low |
| Tilt gate not configured | Tilted scans corrupt localization | Same dependency as SLAM; AMCL reads `/scan_slam` | Low |

Security: the new POST endpoints only change the localization mode on the robot's own
service and use the existing cross-site guard. Risk is medium, so `/security-review` is not
required.

QA route: **UI** (web hub `/slam`) plus internal service. UI checks: mode badge, state and
std update every poll; buttons disabled in the wrong mode; health dot colors in both modes.
Internal checks: lifecycle states, TF `map → odom` present after convergence, restart
recovery (kill the AMCL process, expect relocalization).

## Work packages for parallel agents

Each package runs in its own git worktree on a branch from `feat/localization`, named
`feat/localization-wpN`. An agent edits **only** the files its package owns, codes against
the interface contract above, and adds tests for its own behavior. If the contract does not
fit, it stops and reports instead of editing another package's files. Every agent runs the
full suite before reporting and states the result exactly. No agent commits to
`feat/localization` itself, pushes, or touches systemd units; the orchestrator merges.

Order: **Wave 1** = WP1, WP2 in parallel. **Wave 2** (after both are merged) = WP3, WP4 in
parallel. **Wave 3** = WP5.

### WP1: Localization module (wave 1)

Owns: `lds08rr_lidar/localization.py` (new), `lds08rr_lidar/localization.yaml` (new),
`test/test_localization.py` (new).

1. Write `localization.yaml` with `map_server` and `amcl` sections under `ros__parameters`.
   `amcl`: `base_frame_id: base_link`, `odom_frame_id: odom`, `global_frame_id: map`,
   `scan_topic: /scan_slam`, `robot_model_type: nav2_amcl::DifferentialMotionModel`,
   `laser_model_type: likelihood_field`, `laser_max_range: 8.0`, `max_beams: 60`,
   `min_particles: 500`, `max_particles: 8000`, `set_initial_pose: false`,
   `tf_broadcast: true`, `transform_tolerance: 1.0`, `update_min_d: 0.1`,
   `update_min_a: 0.1`. Comment each non-default value in one line, like
   `slam_toolbox.yaml`. `map_server`: `yaml_filename: ""` (the launch fills it in).
2. Implement `localization_command`, `LocalizationTracker`, `GlobalLocalizer` exactly per
   the contract. Variances come from `PoseWithCovarianceStamped.pose.covariance` indices 0,
   7, 35; std = sqrt(variance).
3. Unit tests: command list; tracker transitions (searching → localized after 5 good poses,
   one bad pose resets the count, timeout → lost with hint, localized → lost); localizer
   with a fake `call_service` (retries while the service is missing, one reinitialize per
   start, nomotion calls stop when localized, a second `start()` cancels the first,
   service timeout → `tracker.fail`).
4. ROS probe on the ros2_pc with `ROS_DOMAIN_ID=83`: run `localization_command` with a
   small test map (written by `map_saver_cli` from a published grid, as in the review) and
   confirm `ros2 lifecycle get /amcl` and `/map_server` report `active`. Do not stop the
   live `navbot-lidar` service. Report the probe output.

### WP2: Map store and export (wave 1)

Owns: `MapStore`, `export_occupancy`, and `SlamControl.save` in `lds08rr_lidar/slam.py`,
plus the `MapStore` / save tests in `test/test_slam.py`. Does not touch `SlamState`,
`SlamProcess`, or other `SlamControl` methods.

1. `MapStore`: four-file `commit(tmp_dir)`, `has_occupancy`, `occupancy_yaml`, four-file
   `delete`. Keep the backup-and-restore behavior and its existing tests, adapted to four
   files.
2. `export_occupancy(base)`: subprocess `ros2 run nav2_map_server map_saver_cli -f <base>
   --ros-args -p map_subscribe_transient_local:=true`, timeout, check that both files exist.
3. `SlamControl.save`: create `directory/map.saving.<ns>/`, serialize to `<tmp>/map`, export
   to `<tmp>/map`, commit, and always remove the temporary directory. `SlamControl` takes
   `export` as a constructor argument (default `export_occupancy`) so tests inject a fake.
4. Tests: commit needs all four files; failure keeps the old four; after commit `map.yaml`
   says `image: map.pgm`; save removes the temp dir on export failure. A ROS probe for
   `export_occupancy` with `ROS_DOMAIN_ID=83` and a published test grid.

### WP3: Mode control and ROS wiring (wave 2)

Owns: the rest of `SlamControl` in `slam.py`, `lds08rr_lidar/service.py`,
`lds08rr_lidar/ros_node.py`, and the mode tests in `test/test_slam.py`.

1. `SlamControl`: `mode`, `initial_mode`, `command`, `set_mode`, `locate`, the mode check
   in `save`, `delete` switches to mapping, `status` adds `mode` and `localization`. On
   every switch: stop the process, `slam_state.clear()`, `slam_state.set_map_latched(...)`,
   `tracker.reset()`, start the process, and in localization start the localizer.
   `set_mode` and `locate` hold the same lock as `save`.
2. `ros_node.py`: a `call_service(name, timeout)` for `std_srvs/srv/Empty`, built like
   `serialize_map` (wait for service, `call_async`, `Event` wait). Subscribe
   `/amcl_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`, reliable, depth 1) and
   feed `tracker.add_pose`.
3. `service.py`: build tracker, localizer, and control; `SlamProcess(command=control.command, …)`;
   set the initial mode before `start()`; print the mode in the startup line.
4. Tests: boot mode with and without `map.yaml`; switch stops and starts the process with the
   right command; save and locate refused in the wrong mode; delete in localization ends in
   mapping.

### WP4: Web hub, state and health (wave 2)

Owns: `SlamState` in `slam.py`, `lds08rr_lidar/web.py`, `lds08rr_lidar/health.py`,
`lds08rr_lidar/templates/slam.html`, `test/test_health.py`, and the web tests in
`test/test_slam.py`. Uses `control.status()["mode"]` and `["localization"]` from the
contract; tests use a fake control.

1. `SlamState.set_map_latched` and the localization states per the contract.
2. `web.py`: `POST /slam/mode` (JSON `{"mode": "mapping"|"localization"}`) and
   `POST /slam/locate`, both behind the existing cross-site guard, 409 when there is no
   control or the control refuses. `/map.json` passes `mode` and `localization` through.
3. `health.py`: the row is named "SLAM / localization". Mapping: today's rules. Localization:
   `localized` → ok, `searching` → warn "Searching", `lost` → warn "Lost" (error if the
   tracker has an error text), process down → today's rules.
4. `slam.html`: mode badge, localization state with std (m, °) and the hint text, "Locate
   again" and "Switch to mapping/localization" buttons; disable Save in localization.
   Keep the page's existing state words and severity colors.
5. Tests: stale rule off when latched; health rows for each localization state; new
   endpoints including the 403 cross-site case.

### WP5: Live verification and docs (wave 3, needs the user at the robot)

Owns: a new `lab/ros2_pc/YYYY-MM-DD-amcl-localization.md`,
`docs/ros2_pc/software/lds08rr-lidar.md`, `docs/ros2_pc/plans/roadmap.md`, and the removal
of this plan file.

1. Save a map in mapping mode, restart the service, confirm it boots into localization.
2. At 3+ measured start poses: record found pose, error, time to `localized`, CPU load.
3. Kill the AMCL process once; confirm restart and relocalization.
4. Move the implemented behavior into `software/lds08rr-lidar.md`, link the lab note,
   update the roadmap, delete this plan, and run `python3 tests/layout/test_repo_layout.py`.

## Non-goals

- Path planning, goals, or autonomous driving. That is the next project, with Nav2 on top of
  this localization and the exported map files.
- Changing the map while localizing, or merging new areas into the saved map.
- More than one saved map.
- Tuning the LDS08RR range or calibration (roadmap item 2).

## Optional, not in scope until chosen

- Click-and-drag start pose on the web-hub map, sent as `/initialpose`. AMCL supports it, so
  it is a small fallback for rooms where global search picks the wrong one of two similar
  places.
- Repeat the global search automatically when the state stays `lost` while driving.

## Done when

- With a saved map, the robot boots into localization, finds its pose without a hint, and
  the web hub shows `localized` at three or more test starts, within ±0.2 m and ±10° of the
  measured pose.
- The saved map files are unchanged after a localization session.
- Switching to mapping and back works from the web hub without a service restart.
- Tests pass and the lab note records the measured accuracy, convergence time, and CPU load.
