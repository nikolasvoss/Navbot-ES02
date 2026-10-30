# navlink quick start

Talk to the robot from the ros2_pc: read values, watch streams, change settings, record CSV files. Full reference: [navlink](navlink.md).

## 1. Start

Only one program can use the serial port, so stop the ROS bridge first.

```bash
sudo systemctl stop navbot-link-bridge
cd ~/Navbot-wt/fw
python3 scripts/navlink.py shell --port /dev/ttyAMA3
```

When you are done, type `quit` and run `sudo systemctl start navbot-link-bridge`.

Over USB, use `--port /dev/ttyUSB0`. Opening it resets the robot, so wait about 20 seconds before the first command.

## 2. The three kinds of things

| Kind | What it is | Example |
| --- | --- | --- |
| Signal | A value you can read or stream. Read-only. | `roll_deg`, `voltage_filtered_v` |
| Parameter | A setting you can change. | `speed_kp`, `servo1_trim` |
| Action | Something the robot does. Asks `y/N` first. | `motor_stop`, `calibrate` |

## 3. Commands

| Type | Effect |
| --- | --- |
| `list` | Show everything. `list motor*` shows only matching names. |
| `get roll_deg` | Read the current value. |
| `set speed_kp 0.12` | Change a parameter. It prints `old -> new`. |
| `sub @bal` | Stream a preset to the screen. |
| `sub roll_deg gyro_z_rad_s @50 -o a.csv` | Stream two signals at 50 Hz into `a.csv`. |
| `unsub 1` | Stop stream number 1. |
| `call motor_stop` | Run an action after you confirm with `y`. |
| `log on` / `log off` | Show or hide robot log messages. |
| `quit` | Stop all streams and exit. |

Presets (`@trace`, `@ctrl`, `@bal`, `@drive`) are named signal lists at 100 Hz, defined in `scripts/navlink_presets.yaml`.

## 4. Record a file without the shell

```bash
python3 scripts/navlink.py capture --port /dev/ttyAMA3 --preset drive --seconds 30 -o drive-01.csv
```

The file has a `#` header with the firmware build and all parameters, then `t_us,<names>` rows. Robot log messages appear as `#` lines.

## 5. Rules and safety

- The robot refuses every `set` while balance is on (CH5 on). Switch balance off first.
- The robot refuses a value that breaks a rule, such as `trim_envelope` above `tilt_ref_limit`. The old value stays, and the message says why.
- The motor actions and `calibrate` move the motors. Lift the wheels first and keep a hand on the power. `motor_stop` stops them.
- Streams stop 5 seconds after the shell dies, so a crashed shell never leaves the robot streaming.

## 6. If something is wrong

| Message | Meaning |
| --- | --- |
| `no reply to get_catalog` | The robot is still booting, or another program holds the port. Wait 20 seconds, check that the bridge is stopped, and try again. |
| `unknown signal <name>` | The name is not in the robot's list. Use `list`. |
| `refused while balancing` | Switch balance off. |
| `slot in use` | Another port owns the stream. Stop it there first. |
