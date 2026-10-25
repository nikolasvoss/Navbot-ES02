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

Flashed and wired to the Pi. The link failed until the STC8H1K08 on the line was disconnected; see [the STC note](2026-10-25-stc8h1k08-backpowers-uart1.md). Frames decode at 115200 (diagnostic build); the 500000 baud rate is still to be checked on the robot.
