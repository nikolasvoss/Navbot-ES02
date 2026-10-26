# STC8H1K08 on the UART1 line back-powered and corrupted the ros2_pc link, 2026-10-25

Status: concluded

## Observation

With the Pi wired to `RX1`/`TX1`, the link never produced valid frames. The Pi's RX pin (GPIO9 on `/dev/ttyAMA3`) saw a low pulse of about 0.4 ms at about 123 Hz and no CRC-valid frame at any baud rate from 9600 to 1000000. With the Pi unplugged, both robot UART1 pins showed UART-like activity, and the high level was often about 2.8 V instead of 3.3 V.

An STC8H1K08 microcontroller sits on the UART1 line. Its supply is not connected. Its VCC pin measured 2.3 V with the robot powered. The part is not in the netlist or the component tables.

## Evidence

- Pi captures before the fix (`/dev/ttyAMA3`): 500000 baud gave mostly `00` and `f8`; 115200 gave the repeating bytes `80 00 00 00 00` at about 616 B/s. No baud rate gave valid frames.
- ESP-ROM boot at 115200 on the CH340 console was clean (`rst:0x1 (POWERON)`, `entry 0x403c88b8`), and the application reached `system run.` at 576000 baud, so the board was not in a boot loop.
- With `LINK_UART_BAUD` temporarily set to 115200 (the link is 500000 baud in the normal firmware) and the STC still fitted, 33 of 1702 segments in a 10 s capture passed the CRC. After the STC's UART connection was removed, 711 of 712 frames passed, `seq` was gapless (155322 to 156032), and the rate was 3.5 KB/s. Capture: [2026-10-25-uart1-link-115200.bin](captures/2026-10-25-uart1-link-115200.bin), decoded as COBS with a `0x00` delimiter and CRC-16/CCITT-FALSE (little-endian) as in `link_frame.cpp`.

## Practical consequence

An unpowered microcontroller on a signal line is back-powered through its input protection diodes and can load the line and drive its own pins. Do not leave an unpowered chip connected to `RX1`/`TX1`. Keep it removed or isolated.

The 500000 baud rate was restored in the firmware after the 115200 test. The link has been decoded only at 115200 so far; check it at 500000 on the robot before relying on it. The odometry signs have not been checked on hardware.
