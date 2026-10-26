# UART1 repurposed for the ros2_pc link, 2026-10-25

Status: open

## Observation

The touchscreen is removed. The firmware no longer contains the touch driver, the touch PID loops or the touch log selectors (K26, K27, K29, K30, K34, K35, K36, K39, K40).

## Evidence

- `link_uart.h` opens `Serial1` at 500000 baud, RX = GPIO20, TX = GPIO19 (`board::kLinkUartRxPin`, `board::kLinkUartTxPin`). The firmware builds with `scripts/build_firmware.py`, and the host tests pass.
- No board test, flash or wiring check was done.

## Practical consequence

With no touch data, the old control path produced zero touch terms, so the removal keeps the control behavior. Only the RC ball-pose offsets (CH9/CH10) remain. The CH5 gain sets are now called set 1 and set 2, with unchanged values. The DRIVE trace row lost its touch column. Verify 3.3 V levels and the pin direction before connecting the ros2_pc. Protocol: [NavBot link](../../docs/integration/software/ros2-uart-protocol.md).

## 2026-10-25 update

The link baud rate was lowered from 500000 to 115200 for the first hardware test, in `link_uart.h` and in the bridge default. The `usbPad=0` boot line is visible on the robot (user report). The pad release was added because GPIO19/20 power up as USB pads; whether the pins now carry UART1 is not yet confirmed.

Flashed and wired to the Pi. The link failed until the STC8H1K08 on the line was disconnected; see [the STC note](2026-10-25-stc8h1k08-backpowers-uart1.md). Frames decode at 115200 (diagnostic build); the 500000 baud rate is still to be checked on the robot.

## 2026-10-26 update

The two branches above were merged. The link runs at 500000 baud with the USB pad release, the boot pin check and the periodic link counters from the first branch, and the 20 ms send schedule and `Imu.t_us` from the second. Before the merge, a build without the pad release carried clean 500000-baud frames once the STC8H1K08 was disconnected, so the STC was the cause. The merged build has not been flashed.

## 2026-10-26 merged build on the robot

Built `feature/esp32-ros2-uart-interface` (commit `0c5e5f9`) with `scripts/build_remote.sh` and flashed it with `scripts/upload_firmware.py` (esptool; hash verified). Boot log: `Link: GPIO19 drive low reads 0, high reads 1` and `Link: UART1 500000 baud RX=20 TX=19 usbPad=0`. A 10 s capture on `/dev/ttyAMA3` at 500000 baud gave 1002 CRC-valid frames, none bad, no `seq` gaps, 4.9 KB/s. `WheelOdometry` and `Imu` each 501 frames at 20.000 ms mean interval (50.0 Hz; min 18.64, max 20.43 ms). This confirms the USB pad release, the pin check and the 20 ms schedule together on 500000 baud. Status stays open for the loop-timing item in the [roadmap](../../docs/robot/plans/roadmap.md).

## 2026-10-26 link under balance

With the robot balancing (supported, pitch within 0.3 degrees), a 120 s capture on `/dev/ttyAMA3` at 500000 baud in four 30 s windows gave 12002 CRC-valid frames, none bad, no `seq` gaps. `WheelOdometry` and `Imu` each ran at 50.00 Hz; the longest interval was 21.8 ms. Net wheel travel per window was about 1 to 1.6 rad in either direction. The motor PWM and balance control did not disturb the link in this test. Only the Pi's UART was read: opening the CH340 console resets the board, so it was not opened while the robot balanced.
