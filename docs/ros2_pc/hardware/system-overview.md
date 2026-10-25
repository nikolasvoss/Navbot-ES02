# ros2_pc-Hardware

This directory documents the ros2_pc, its carrier board, power supply, connectors, and attached hardware.

The measured current of CM5 together with a running LiDAR is recorded below. The LDS08RR reference below includes module label and PCB photo evidence; its electrical behavior and connection remain unverified. Do not infer pin assignments, power limits, or peripheral connections from the roadmap. Add each fact with its board revision and evidence when the hardware is selected or inspected.

## Peripheral status

| Peripheral | State | Reference |
| --- | --- | --- |
| ros2_pc and carrier board | **Confirmed.** The ros2_pc sits on the carrier board described in the [carrier pinout reference](ros2-pc-carrier-board.md). The board revision is not recorded. | User statement, 2026-10-20 |
| LDS08RR LiDAR | **Connected and receiving.** UART0, `/dev/ttyAMA0`, live `/scan` verified. Supply wiring, voltage margins and motor control are not verified. | [LDS08RR page](lds08rr-lidar.md), 2026-10-15 |
| HMMD mmWave radar | **Planned on UART2.** An earlier setup read it on UART0 (`/dev/ttyAMA0`, 2026-10-04/05); UART0 now belongs to the LiDAR. Current wiring, module variant, supply and signal levels are not verified. | [HMMD radar](hmmd-radar.md), [HMMD guide](../software/hmmd-ros2.md) |
| ESP32 robot link | **Not recorded as wired.** The ros2_pc UART for the link is not recorded here. The robot side is UART1, 500000 baud, 3.3 V logic. | [Link protocol](../../integration/software/ros2-uart-protocol.md), [robot serial interfaces](../../robot/hardware/electronics-reference.md#serial-interfaces) |

## J8 pin allocation

J8 is the carrier's 40-pin GPIO header. Physical pin numbers follow the [carrier pinout](ros2-pc-carrier-board.md). Only pins with a project use are listed.

| J8 pin | Signal | Use | State |
| ---: | --- | --- | --- |
| 1 or 17 | 3V3 | HMMD supply | Planned |
| 4 | 5V | LiDAR `VIN` | Connected |
| 6 | GND | LiDAR `GND`; the HMMD guide also names pin 6 | Connected (LiDAR) |
| 7 | GPIO4 / UART2 TX | HMMD RX | Planned |
| 8 | GPIO14 / UART0 TX | Reserved; LiDAR `RX` stays disconnected | Reserved |
| 10 | GPIO15 / UART0 RX | LiDAR `TX` | Connected |
| 11 | GPIO17 | Reserved for an external sink on LiDAR `M−`; never wired directly | Reserved, inactive |
| 29 | GPIO5 / UART2 RX | HMMD TX | Planned |
| — | Not recorded | ESP32 robot link | Open |

Notes:

- The LiDAR and the HMMD guide both name pin 6 for ground. J8 has further GND pins (9, 14, 20, 25, 30, 34, 39); use a separate one when wiring both.
- UART2 is not enabled yet. It needs a boot overlay and a reboot; the [HMMD guide](../software/hmmd-ros2.md) has the steps. Confirm the resulting `/dev/ttyAMA*` path on the running image before use.
- J8 logic is 3.3 V. The ros2_pc does not tolerate 5 V on a GPIO. The LiDAR's `M−` idles at 5 V.
- The total GPIO load is limited to 50 mA across all pins. See the [electrical limits](ros2-pc-carrier-board.md#electrical-limits).

## Measured current with LiDAR

User-reported measurement, 2026-10-15:

| Configuration | Average current | Observed peaks |
| --- | --- | --- |
| CM5 together with LiDAR, with LiDAR running continuously | Approximately 800 mA | Up to 1.2 A |

These values describe the combined load in the measured operating condition. Supply voltage, measurement point, instrument, measurement duration, and CM5 workload were not reported. Startup current was not specified. The observed peaks do not establish a maximum current requirement or verify the Navbot supply's available capacity.

Source: [user measurement record](../../../lab/ros2_pc/2026-10-15-power-observations.md).

The [ros2_pc carrier-board pinout](ros2-pc-carrier-board.md) is a reference. It does not confirm that the project has selected or verified that carrier board.

The [LDS08RR LiDAR hardware reference](lds08rr-lidar.md) records the photographed module rating, connector markings, upstream protocol variants, and pending electrical and mechanical checks.

The planned HMMD radar and Roborock LiDAR work is listed in the [ros2_pc software roadmap](../plans/roadmap.md). Record their electrical and mechanical details here only after the exact modules are identified.
