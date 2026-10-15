# UART1 repurposed for the ros2_pc link

- **Observation:** The touchscreen is removed. The firmware no longer contains the touch driver, touch PID loops or touch log selectors (K26, K27, K29, K30, K34, K35, K36, K39, K40).
- **Evidence:** `link_uart.h` opens `Serial1` at 500000 baud, RX=GPIO20, TX=GPIO19. Firmware builds with `scripts/build_firmware.py`. No board test, flash or wiring check was done.
- **Consequence:** With no touch data, the old control path produced zero touch terms, so the removal keeps control behaviour; only the RC ball-pose offsets (CH9/CH10) remain. CH5 gain sets are now called set 1 and set 2 with unchanged values. The DRIVE row lost its `touch_x` column. Verify 3.3 V levels and the pin direction before connecting the ros2_pc.
