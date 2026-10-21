# Robot hardware overview

Entry page for the robot's hardware (the ESP32-S3 controller board and everything attached to it). Read it in about a minute, then open only the section you need in the [electronics reference](electronics/reference.md). The ros2_pc and its sensors have their own tree: [ros2_pc hardware](../../ros2_pc/hardware/README.md).

## System at a glance

```text
2 × 18650 in series ── protection / switch ── VIN (2S)
                                             ├─ 2 × MP6536 → 2 × three-phase 2208 wheel motor
                                             ├─ RT8289 → +5 V → XC6210 → +3.3 V
                                             └─ servo rail, jumper-selected: +5 V or VIN

ESP32-S3-WROOM-1 on MAIN (no PSRAM)
  ├─ 2 separate I²C buses → 1 AS5600 board (CODER) each, wheel angle for SimpleFOC
  ├─ SPI → ICM-42688P IMU, attitude and balance control
  ├─ 6 PWM + 2 enable → 2 motor drivers; 4 PWM → leg servos
  ├─ UART0 → CH340X → USB-C, flashing and serial (576000 baud)
  ├─ UART1 → ros2_pc link (500000 baud)
  ├─ UART2 ← inverted SBUS from the RC receiver
  ├─ GPIO17 ADC ← 2S pack voltage via 10 kΩ / 1 kΩ divider
  └─ BLE, built into the ESP32
```

The mechanics have two brushless wheel motors and four leg servos. A [photo of the robot](../media/photo1.JPG) shows the assembly.

## Pages in this tree

| Page | Contents |
| --- | --- |
| [Electronics reference](electronics/reference.md) | Power, ESP32 module, full GPIO map, motors, sensors, all connectors, serial interfaces, source discrepancies. |
| [Component netlist](electronics/netlist.md) | Every pad of every component on MAIN and CODER. Search it; do not read it whole. |
| [Touchscreen stand](mechanics/touchscreenstand.md) | Mounting-hole pattern on the robot top, now used by the LiDAR/ros2_pc stand. |
| [SBUS connection sheet](connectors/ES02-SBUS-connect.pdf), [wiring photo](connectors/ES02-SBUS-wire.png), [RadioLink connector](connectors/RadioLink-connector.png) | RC receiver wiring. |
| [`BOM.xlsx`](BOM.xlsx) | Intended purchased parts, sheet `Sheet1`. |

## Find by task

| Task | Relevant hardware | Read |
| --- | --- | --- |
| Wheel motor does not turn / FOC | MP6536 `U6`/`U7`, three phases per wheel, AS5600 on `H5`/`H6` | [Motors and encoders](electronics/reference.md#motors-and-encoders) |
| Attitude, balance or driving behavior | ICM-42688P `U5` on SPI, battery ADC GPIO17 | [Sensors](electronics/reference.md#sensors-and-signals), [control flow](../software/control/control-flow.md) |
| Servo problem or supply dip | `H9`/`H10` servo outputs, supply jumper `H11`, `VIN` from the 2S pack | [Power](electronics/reference.md#power), [connectors](electronics/reference.md#connectors) |
| Which GPIO does what | ESP32-S3 `U4` | [GPIO map](electronics/reference.md#esp32-s3-gpio-map) |
| RC receiver | `H4`: GND, +5 V, SBUS; inverter `Q1`; GPIO1 | [Serial interfaces](electronics/reference.md#serial-interfaces), [RC channel map](../software/control/rc-channel-map.md) |
| Wiring the ros2_pc link | `H2`/`CN4`, UART1 | [Serial interfaces](electronics/reference.md#serial-interfaces), [ros2_pc J8 allocation](../../ros2_pc/hardware/README.md#j8-pin-allocation) |
| Flashing / USB diagnosis | USB-C → CH340X → UART0; no native USB CDC | [USB/serial guide](../software/development/usb-serial-flashing.md) |
| Memory budget (logging, buffers) | ESP32-S3 without PSRAM | [Module and memory](electronics/reference.md#esp32-s3-module-and-memory) |
| One resistor, IC pin or unnamed net | All pads of both boards | [Netlist](electronics/netlist.md) |
| Robot mounting holes on top | 96.4 × 72.9 mm pattern | [Touchscreen stand](mechanics/touchscreenstand.md) |

## Pitfalls

Each item is explained in the linked section.

- **No native USB CDC.** USB-C goes to the CH340X, and the native USB pins carry UART1. A firmware option cannot change that. [Details](electronics/reference.md#serial-interfaces).
- **UART1 net names are reversed.** Net `RX1` (GPIO19) is the firmware's TX. Wire by GPIO number. [Details](electronics/reference.md#source-discrepancies).
- **Servos 3 and 4 are swapped** between net names and firmware. [Details](electronics/reference.md#source-discrepancies).
- **No PSRAM.** Plan with internal SRAM only, despite the N8R8 label in the netlist. [Details](electronics/reference.md#esp32-s3-module-and-memory).
- **Servo rail can carry 2S VIN.** Check the `H11` jumper and the servo voltage rating first. The servos are not on the 3.3 V regulator. [Details](electronics/reference.md#power).
- **Battery reading is a pack estimate.** GPIO17 reads the 2S pack through a divider, not the 3.3 V rail. Readings drop sharply under balance load; battery, contacts, regulator and ADC effects are not yet separated ([sensor diagnostic guide](../software/diagnostics/sensor-diagnostic-mode.md)). [Details](electronics/reference.md#power).
- **No IMU interrupt line.** `INT1`/`INT2` do not reach the ESP32. [Details](electronics/reference.md#imu-interrupts).
- **Identical encoder addresses are fine.** Each AS5600 has its own I²C controller. [Details](electronics/reference.md#motors-and-encoders).
- **SBUS is inverted on the board.** The receiver gets 5 V on `H4`; `Q1` inverts the signal to GPIO1. Check pin order before plugging in. [Details](electronics/reference.md#serial-interfaces).
- **Encoder/UART footprints have extra GND pads** (5/6). They are not cable wires. [Details](electronics/reference.md#connectors).

## Sources of truth

| Source | Describes | Location |
| --- | --- | --- |
| PCB netlist | Intended electrical connections | [`hardware/pcb/ProPrj_NavBot-ES02.epro`](../../../hardware/pcb/ProPrj_NavBot-ES02.epro): `project.json`, MAIN/CODER `.esch` and `.epcb`. Connector pins and nets come from `PAD_NET` in the `.epcb` files. |
| Firmware | Configured pins and behavior | [`OllieFOCdrive.ino`](../../../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino), [`ICM42688.h`](../../../src/ES-02/OllieFOCdrive/ICM42688.h), [`link_uart.h`](../../../src/ES-02/OllieFOCdrive/link_uart.h), [`FUTABA_SBUS.h`](../../../src/ES-02/OllieFOCdrive/FUTABA_SBUS.h), [`ServoControl.h`](../../../src/ES-02/OllieFOCdrive/ServoControl.h) |
| BOM | Intended purchased parts | [`BOM.xlsx`](BOM.xlsx) |
| User statements and captures | The assembled robot | [Hardware findings](../../../agent_notes/robot/hardware-findings/), [USB/serial guide](../software/development/usb-serial-flashing.md), [sensor diagnostic guide](../software/diagnostics/sensor-diagnostic-mode.md) |

When sources disagree, name the discrepancy instead of choosing silently. Check the physical board when the answer depends on layout, connector orientation, fitted parts or actual wiring. Verify pinout and voltage before applying power or changing wiring.

## Open hardware questions

- `U1` RT8289 `EN` pin appears unconnected in the export. [Check](electronics/reference.md#5-v-regulator-check).
- Which `H11` servo supply jumper is fitted, and whether the servos tolerate 2S VIN.
- Pole-pair check warnings (8.46 and 7.80 against the configured 7). [Details](electronics/reference.md#pole-pairs).
- Battery protection cut-off threshold, charging behavior, operation while charging.
- Free internal SRAM under firmware and Wi-Fi load.
- Physical UART rate at 576000 baud is not measured. [USB/serial guide](../software/development/usb-serial-flashing.md).
