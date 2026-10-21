# ros2_pc hardware overview

Entry page for the ros2_pc: the onboard computer that runs ROS 2, its carrier board, and the sensors wired to it. The robot's ESP32 board has its own tree: [robot hardware](../../robot/hardware/system-overview.md). Software for the ros2_pc is in [ros2_pc software](../software/README.md).

## Status

| Item | Status | Evidence |
| --- | --- | --- |
| ros2_pc and carrier board | **Confirmed.** The ros2_pc sits on the carrier board described in the [carrier pinout reference](ros2-pc-carrier-board.md). The board revision is not recorded. | User statement, 2026-10-20 |
| LDS08RR LiDAR | **Connected and receiving.** UART0, `/dev/ttyAMA0`, live `/scan` verified. Supply wiring, voltage margins and motor control are not verified. | [LDS08RR page](lds08rr-lidar.md), 2026-10-15 |
| HMMD mmWave radar | **Planned on UART2.** An earlier setup read it on UART0 (`/dev/ttyAMA0`, 2026-10-04/05); UART0 now belongs to the LiDAR. Current wiring, module variant, supply and signal levels are not verified. | [HMMD guide](../software/hmmd-ros2.md), [HMMD agent notes](../../../agent_notes/ros2_pc/sensors/hmmd/README.md) |
| ESP32 robot link | **Not wired.** The ros2_pc UART for the link is not chosen. The robot side is UART1, 500000 baud, 3.3 V logic. | [Link protocol](../../robot/software/interfaces/ros2-uart-protocol.md), [robot serial interfaces](../../robot/hardware/electronics/reference.md#serial-interfaces) |

## J8 pin allocation

J8 is the carrier's 40-pin GPIO header. Physical pin numbers follow the [carrier pinout](ros2-pc-carrier-board.md#gpio-header-j8). Only pins with a project use are listed.

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
| — | Not chosen | ESP32 robot link | Open |

Notes:

- The LiDAR and the HMMD guide both name pin 6 for ground. J8 has further GND pins (9, 14, 20, 25, 30, 34, 39); use a separate one when wiring both.
- UART2 is not enabled yet. It needs a boot overlay and a reboot; the [HMMD guide](../software/hmmd-ros2.md) has the steps. Confirm the resulting `/dev/ttyAMA*` path on the running image before use.
- J8 logic is 3.3 V. The ros2_pc does not tolerate 5 V on a GPIO. The LiDAR's `M−` idles at 5 V.
- The total GPIO load is limited to 50 mA across all pins. See [electrical limits](ros2-pc-carrier-board.md#electrical-limits).

## Pages in this tree

| Page | Contents |
| --- | --- |
| [Carrier pinout reference](ros2-pc-carrier-board.md) | Manufacturer pinout of J8 with all alternate functions, other connectors, electrical limits. |
| [LDS08RR LiDAR](lds08rr-lidar.md) | Wiring, label, connector, data format, mechanics, open checks. |
| [`mechanics/`](mechanics/) | LDS08RR dimensions, hole drawing, simplified model; carrier board model. |

## Mechanics

| File | Contents | Source |
| --- | --- | --- |
| [`mechanics/lds08rr-dimensions.json`](mechanics/lds08rr-dimensions.json), [`lds08rr-lochbild.svg`](mechanics/lds08rr-lochbild.svg) | LDS08RR dimensions and hole pattern | User sketches, not measured |
| [`mechanics/lds08rr-simple.step`](mechanics/lds08rr-simple.step), [`lds08rr_simple.py`](mechanics/lds08rr_simple.py) | Simplified LDS08RR envelope and its generator | Derived from the dimension record |
| [`mechanics/ROS2_PC_IO_Board.step`](mechanics/ROS2_PC_IO_Board.step) | 3D model of the carrier board | Added with the stand models |
| [`hardware/3d print/lidar_ros2_pc_stand/`](../../../hardware/3d%20print/lidar_ros2_pc_stand/) | Stand that carries the carrier board and the LiDAR on the robot (rev A, first print). STEP/STL parts and the generator `lidar_ros2_pc_stand_box.py`. | Generator script |

The stand mounts on the robot's [96.4 × 72.9 mm hole pattern](../../robot/hardware/mechanics/touchscreenstand.md). Its generator takes the carrier's four corner holes (121.5 × 69 mm on a 160 × 90 mm board) from a Waveshare drawing of the board, not from the manufacturer documents. Check the holes on the actual board before drilling.

## Rules for adding hardware facts

- Record each fact with its board revision, date and evidence. Keep dated observations in [`agent_notes/ros2_pc/`](../../../agent_notes/ros2_pc/), and update this tree when a fact changes the overview.
- Do not infer pin assignments, power limits or peripheral connections from the [roadmap](../ROS_ROADMAP.md).
- Refer to the computer as "ros2_pc" and to its board as "ros2_pc carrier board". Do not name the module vendor or model in the docs.
