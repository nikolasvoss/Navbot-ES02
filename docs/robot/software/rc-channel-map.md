# RC channel map

The remote control is connected to the NavBot through the receiver over **SBUS**. The firmware reads the channels as follows (SBUS channels are counted from 1 here):

| Channel | Function |
| --- | --- |
| CH1 | Unused; body roll was removed |
| CH2 | Unused; leg length and pitch were removed |
| CH3 | Speed request: forward stick is forward, scaled to plus and minus 0.3 m/s (`speedCapMps`) |
| CH4 | Yaw-rate request: left stick is a left turn, scaled to plus and minus 1.0 rad/s |
| CH5 | Enable: 3-position switch; raw below 592 is off, the two upper positions are on |
| CH6 | Save request for the trim: 2-position latching switch, on at raw 992 or above; works only in Commissioning |
| CH7 | Commissioning: 2-position latching switch, on at raw 992 or above |
| CH8 | Unused; the attitude modes were removed |
| CH9 | Trim knob: raw 992 is trim 0, the end stops are plus and minus 3 degrees; works only in Commissioning |
| CH10 | Unused; ball balance was removed |

The states, thresholds, and pickup rules behind CH5 to CH9 are in [Balance controller](balance-controller.md#remote-switches).

**Note on the HT-10A:** The table describes the SBUS channels the robot expects, not the fixed wiring of particular HT-10A switches. Which sticks, switches, or dials send CH1–CH10 depends on the transmitter configuration. After any change to the transmitter, check the channel assignment in the receiver/servo monitor.

## From transmitter to firmware value

Each proportional channel passes through three conversions before a controller uses it.

1. **Transmitter to SBUS.** The transmitter shows 1000-2000 µs, with 1500 µs as default. The receiver sends these as raw SBUS values `raw = (µs - 880) * 1.6`.
2. **SBUS range in the firmware.** `SBUS_CHANNEL_MIN = 192` and `SBUS_CHANNEL_MAX = 1792` bound the mapping; centre is 992.
3. **Scaling to controller units in `balanceRequest()`.** `rcStickFraction()` turns a raw value into -1 to 1 around 992 (divided by 800, clamped to plus and minus 1), so raw values outside the range are clipped there. The request multiplies the fraction by the speed cap, the yaw-rate cap, or the trim envelope.

| Transmitter | Raw SBUS | Measured |
| --- | --- | --- |
| 1000 µs | 192 | yes (CH9) |
| 1500 µs | 992 | yes (CH1-CH4, CH9, CH10) |
| 2000 µs | 1792 | yes (CH9) |

Direction on the HT-10A as configured on 2026-10-22: CH3 stick forward gives 1792, CH4 stick left gives 192. CH5 is a 3-position switch (positions 0/1/2, raw about 192/992/1792). CH6 and CH7 are latching 2-position switches (0/1, raw about 192/1792); CH6 does not spring back. CH8 was only seen at 192. See the [lab note](../../../lab/robot/2026-10-22-rc-transmitter-raw-values.md).

Current conversion of the proportional channels in `balanceRequest()`:

| Channel | Conversion | Controller use |
| --- | --- | --- |
| CH3 speed | `speedCapMps * rcStickFraction(raw)`; forward gives a positive request | Speed request; ignored in Commissioning |
| CH4 turn | `-yawRateCapRadPerSec * rcStickFraction(raw)`; left gives a positive yaw rate | Yaw-rate request |
| CH9 trim | `rcStickFraction(raw) * trimEnvelopeRad`, valid only for raw 172 to 1811 | Trim, in Commissioning with pickup |

On RC loss (no frame for 100 ms or receiver failsafe), the request counts as zero and the trim is held; see [Losing the remote](balance-controller.md#losing-the-remote).

This description matches the current SBUS handling in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`.
