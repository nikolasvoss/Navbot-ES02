# RC channel map

The remote control is connected to the NavBot through the receiver over **SBUS**. The firmware reads the channels as follows (SBUS channels are counted from 1 here):

| Channel | Function |
| --- | --- |
| CH1 | Unused; body roll was removed |
| CH2 | Unused; leg length and pitch were removed |
| CH3 | Drive speed forward/backward |
| CH4 | Rotation / steering |
| CH5 | PID control: off, on (3-position switch; the lower position is off, the two upper positions are on) |
| CH6 | Unused; posture and mark modes were removed |
| CH7 | Unused; the roll control that used it was removed |
| CH8 | Unused; the attitude modes were removed |
| CH9 | Unused; ball balance was removed (the trim knob comes in package 4) |
| CH10 | Unused; ball balance was removed |

**Note on the HT-10A:** The table describes the SBUS channels the robot expects, not the fixed wiring of particular HT-10A switches. Which sticks, switches, or dials send CH1–CH10 depends on the transmitter configuration. After any change to the transmitter, check the channel assignment in the receiver/servo monitor.

## From transmitter to firmware value

Each proportional channel passes through three conversions before a controller uses it.

1. **Transmitter to SBUS.** The transmitter shows 1000-2000 µs, with 1500 µs as default. The receiver sends these as raw SBUS values `raw = (µs - 880) * 1.6`.
2. **SBUS range in the firmware.** `SBUS_CHANNEL_MIN = 192` and `SBUS_CHANNEL_MAX = 1792` bound the mapping; centre is 992. The firmware does not clip raw values outside this range.
3. **Scaling to controller units in `applyRcChannels()`.** Each channel has its own `mapf()` range and sign. `readSbus()` then changes some values again.

| Transmitter | Raw SBUS | Measured |
| --- | --- | --- |
| 1000 µs | 192 | yes (CH9) |
| 1500 µs | 992 | yes (CH1-CH4, CH9, CH10) |
| 2000 µs | 1792 | yes (CH9) |

Direction on the HT-10A as configured on 2026-10-22: CH3 stick forward gives 1792, CH4 stick left gives 192. CH5 is a 3-position switch (positions 0/1/2, raw about 192/992/1792). CH6 and CH7 are latching 2-position switches (0/1, raw about 192/1792); CH6 does not spring back. CH8 was only seen at 192. See the [lab note](../../../lab/robot/2026-10-22-rc-transmitter-raw-values.md).

Current conversion of the drive channels:

| Channel | `applyRcChannels()` | Later change | Controller use |
| --- | --- | --- | --- |
| CH3 speed | `movementSpeed = mapf(raw, 192, 1792, -15, 15)`; forward gives +15 | none | None since Package 3 row 2 removed the speed loop; the value is parsed only |
| CH4 turn | `bodyTurn = -mapf(raw, 192, 1792, -11, 11)`; left gives +11 | none | Yaw-rate setpoint compared with gyro Z in rad/s |

On RC loss (no frame for 100 ms or receiver failsafe), `movementSpeed` and `bodyTurn` become 0.

This description matches the current SBUS handling in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`.
