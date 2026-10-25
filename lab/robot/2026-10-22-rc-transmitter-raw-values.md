# RC transmitter raw values at the firmware

Status: concluded

Date: 2026-10-22

## Observation

With the normal firmware running, `scripts/remote_status.py` read the raw SBUS values the firmware receives (`K8` output). The transmitter displays its proportional channels as 1000-2000 µs with a 1500 µs default.

| Transmitter setting | Raw value at the firmware |
| --- | --- |
| Sticks and CH9 knob centred (1500 µs) | 992 on CH1-CH4, CH9, CH10 |
| CH9 knob at 1000 µs | 192 |
| CH9 knob at 2000 µs | 1792 |
| CH3 stick forward | 1792 |
| CH4 stick left | 192 |
| CH5-CH8 switches in the tested position | 192 |

The values match the linear SBUS scaling `raw = (µs - 880) * 1.6` and the firmware constants `SBUS_CHANNEL_MIN = 192`, `SBUS_CHANNEL_MAX = 1792`, and `kSbusBodyHeightCenter = 992`. The reported SBUS frame interval was 7-11 ms.

## Evidence

Four `remote_status.py` JSON results from `/dev/ttyUSB0`, captured between 11:48:50 and 11:51:09 UTC, were pasted into the task chat. The CH3-forward and CH4-left values come from one snapshot in which both sticks were held at the same time. The other switch positions of CH5-CH8 and the extremes of CH1, CH2, and CH10 were not recorded.

## Practical consequence and limits

The full transmitter range reaches the firmware's full mapping range without clipping, and centre maps exactly to 992. Forward on CH3 and left on CH4 are now known as raw values. Whether a positive firmware speed moves the robot forward depends on the motor and encoder signs, which this check did not test. The [RC channel map](../../docs/robot/software/rc-channel-map.md) describes the resulting conversion chain.

## 2026-10-22 addendum: switch positions

The user reported the switch positions from further `remote_status.py` runs: CH5 has three positions (0/1/2), CH6 and CH7 have two (0/1), and CH6 is latching, not momentary. The script reports the nearest of 192, 992, and 1792, so these are estimated positions; the raw values were not pasted. CH8 positions other than 192 remain unrecorded.
