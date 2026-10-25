# RC channel map

The remote control is connected to the NavBot through the receiver over **SBUS**. The firmware reads the channels as follows (SBUS channels are counted from 1 here):

| Channel | Function |
| --- | --- |
| CH1 | Body roll / lateral tilt |
| CH2 | Leg length; in "Pitching Adjust" mode, pitch angle instead |
| CH3 | Drive speed forward/backward |
| CH4 | Rotation / steering |
| CH5 | PID control: off, set 1, set 2 (3 levels) |
| CH6 | Posture mode or mark mode (2 levels) |
| CH7 | Roll control manual or automatic (2 levels) |
| CH8 | Drive/posture mode: standard, Pitching Adjust, or Ball Balance (3 levels) |
| CH9 | X target position of the ball in Ball Balance mode |
| CH10 | Y target position of the ball in Ball Balance mode |

**Note on the HT-10A:** The table describes the SBUS channels the robot expects, not the fixed wiring of particular HT-10A switches. Which sticks, switches, or dials send CH1–CH10 depends on the transmitter configuration. After any change to the transmitter, check the channel assignment in the receiver/servo monitor.

This description matches the current SBUS handling in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino` and the mode constants in `OllieFOCdrive.h`.
