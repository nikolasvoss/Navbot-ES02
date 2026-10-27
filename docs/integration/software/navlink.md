# navlink: signals, parameters and actions over the link

Status: **PC side only.** The `navbot_link` library and the `navlink` tool speak the catalog and subscription messages of the [protocol](ros2-uart-protocol.md#catalog-subscriptions-parameters-and-actions). **The robot firmware does not answer these messages yet**, so against the real robot `navlink` stops with `no reply to get_catalog`. The tool and the library were tested only against the fake firmware in `tests/navbot_link/`. The firmware side is migration step 3 of the [logging and control plan](../plans/logging-and-control-link.md#migration).

## Run

`navlink` is [scripts/navlink.py](../../../scripts/navlink.py). It needs `python3-serial`, `python3-protobuf` 3.21 and `python3-yaml`.

```bash
python3 scripts/navlink.py shell --port /dev/ttyAMA3
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset speed_loop --seconds 30 -o speed-01.csv
```

- `/dev/ttyAMA3` is the ros2_pc UART. Stop the bridge service first; only one program can read the port.
- `/dev/ttyUSB0` is the robot's USB port. A port name with `USB` or `ACM` defaults to 576000 baud, any other to 2000000. `--baud` overrides it. Opening the CH340 can reset the board ([USB/serial facts](../../robot/software/usb-serial-flashing.md)); `navlink` keeps DTR and RTS off, which does not prevent it on every adapter.
- `--presets` selects the preset file; the default is [scripts/navlink_presets.yaml](../../../scripts/navlink_presets.yaml).

On connect, `navlink` reads the catalog entry by entry and resolves every name against it. A name the firmware does not have fails with `unknown signal <name>`.

## Shell

| Command | Effect |
| --- | --- |
| `list [pattern]` | Entries whose name matches the glob pattern, with kind, type and unit; for an action, its arguments and bounds. |
| `get <name>...` | Current values. |
| `set <param> <value>` | Writes a parameter and prints `name: old -> new`. A bool takes `0`, `1`, `true`, `false`, `on` or `off`. |
| `call <action> [arg=value]...` | Checks the arguments against the catalog bounds, shows them and asks `y/N`. Only `y` sends. |
| `sub <names\|@preset>... [@<Hz>] [-o file.csv]` | Starts a subscription. `@<Hz>` overrides the preset rate; without a rate it streams every control tick. Without `-o` the rows are printed. |
| `unsub <slot>` | Stops a subscription and closes its file. |
| `log on\|off` | Shows or hides log events and raw text. |
| `quit` | Closes all files and exits. |

```text
navlink> sub @speed_loop -o speed-01.csv
slot 1: 6 entries @ 100 Hz, 2.8 kB/s -> speed-01.csv
navlink> set speed_kp 0.12
speed_kp: 0.08 -> 0.12
navlink> unsub 1
slot 1 closed
```

The load is the row size, `4 + 4 × entries` bytes, times the rate, without frame overhead.

## Capture

`capture` streams one subscription into a CSV file for `--seconds` of robot time: it stops at the first row whose `t_us` is that far from the first row. Pass `--preset <name>` or `--signals a,b,c`, and optionally `--rate <Hz>` (0 = every control tick). It fails when no event arrives for 2 s or three periods, whichever is longer.

```text
# build: fake-build-1
# rate: 100 Hz
# param speed_kp=0.08
# param speed_ki=0.5
# param speed_i_on=1
t_us,speed_target,speed_measured,speed_p,speed_i,tilt_ref,tilt_clipped
1010000,0.5,0.125,-0.03125,0.001,0,1
# log t_ms=1020 INFO [param] speed_kp: 0.08 -> 0.12
1020000,0.5,0.25,-0.03125,0.002,0,0
1030000,0.5,0.375,-0.03125,0.003,0,1
```

- The header comment holds the firmware build, the rate and the value of every parameter at the start.
- The column header is `t_us,<names>`. `t_us` is the robot's `micros()` at the start of the sampled tick; it wraps at 2^32.
- `f32` values have 7 significant digits; `bool` values are `0` or `1`.
- Each `Log` event becomes a comment line at the position it arrived, so a parameter change made over the other port shows in the file. A gap in the port's `seq` becomes `# lost <n> frames before seq <s>`.

## Presets

A preset is a named signal list with a rate in the preset file:

```yaml
speed_loop:
  rate_hz: 100
  signals: [speed_target, speed_measured, speed_p, speed_i, tilt_ref, tilt_clipped]
```

`speed_loop` is the only preset. Presets that replace the `K55` to `K58` profiles follow once the firmware table carries their signals.

## Library

`navbot_link.link.Link(port, baud)` opens the port; it also takes any object with `read()`, `write(data)` and `close()`. It runs one I/O thread.

- `catalog()` reads the table once per connection and again after a robot restart. A `GetCatalog` without a reply is retried 3 times with a 0.5 s timeout.
- `subscribe(names, rate_hz)` returns a `Subscription` with the slot the robot assigned (1 to 4, shared by all ports). The library renews the 5 s lease every 2 s.
- `get(name)`, `set(name, value)` and `call(name, **args)` return when the robot acknowledges. A refused request raises `LinkError` with the robot's reason. `call` is not retried, so a lost `Ack` never starts an action twice.
- `events` is a queue in stream order: `Row`, `LogEvent`, `Text` (raw text from the USB port), `FramesLost`, `Restarted`, `SubscriptionEnded` and `Disconnected`.
- A port error or an unexpected error on the I/O thread ends the link with `Disconnected(reason)`; pending and later requests raise `LinkError(reason)`. A `Log` level the schema does not name shows as its number. A reply value that is not 4 bytes raises `LinkError`.
- A restart of the robot (its `seq` starts over) drops the catalog and ends every subscription, because table indices can change between builds. A request still waiting for its reply raises `LinkError("robot restarted")`, even when the reply that reveals the restart accepts it.

## Tests

```bash
python3 -m pytest -q tests/navbot_link tests/scripts/test_navlink.py
```

`tests/navbot_link/fake_firmware.py` implements the robot side of the catalog, subscription, parameter and action messages, with leases, seq gaps, restarts and USB text. Its clock advances one control tick per read while a subscription is active, so rows and `t_us` are reproducible. The tests run the library and `navlink` against it in memory and, for one capture, as a separate process on a pseudo-terminal. They check request bytes, CSV files and shell output against literal values. The example file above is the start of the preset capture test output.
