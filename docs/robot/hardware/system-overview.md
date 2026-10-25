# Robot with ESP32: Electronics Hardware Overview

**Purpose:** Read this page in about one minute. For pin assignments, power paths, and sources, open only the required section of the [hardware reference](electronics-reference.md). For individual components and unnamed nets, there is also the searchable [component netlist](netlist.md). The information refers to the checked-in EasyEDA project and the current firmware, not to a measurement of every trace on the assembled board.

## System at a glance

```text
2 × 18650 in series ── protection / switch ── VIN (2S)
                                            ├─ 2 × MP6536 → 2 × three-phase 2208 wheel motor
                                            ├─ RT8289 → +5 V → XC6210 → +3.3 V
                                            └─ selectable servo supply (+5 V or VIN)

ESP32-S3-WROOM-1 (MAIN)
  ├─ 2 separate I²C buses → 1 AS5600 board each (CODER), wheel angle for SimpleFOC
  ├─ SPI → ICM-42688P, acceleration and angular rate for attitude/balance control
  ├─ 6 PWM + 2 enable → 2 motor power stages; 4 PWM → leg servos
  ├─ UART2 → inverted SBUS receiver; UART1 → ros2_pc link (500000 baud)
  ├─ UART0 → CH340X → USB-C for flash and serial
  ├─ GPIO17/ADC → 2S battery voltage via 10-kΩ/1-kΩ divider
  └─ BLE → control without an additional radio chip
```

The EasyEDA project contains **MAIN** (control, power, motor drivers, connectors) and **CODER** (AS5600 encoder board; two units according to the BOM). The mechanics have two brushless wheel motors and four leg servos. The touchscreen was removed; UART1 is now the link to the ros2_pc. RC control uses an external receiver and its SBUS output.

## Quick orientation

| Task | Relevant hardware | Read next |
| --- | --- | --- |
| Wheel motor does not turn / FOC | MP6536 `U6/U7`, three motor phases per wheel, AS5600 `H5/H6` | [Motors and encoders](electronics-reference.md#motors-and-encoders) |
| Check attitude, balance, or driving behavior | ICM-42688P `U5` on SPI, battery ADC `GPIO17` | [Control flow](../software/control-flow.md#not-yet-resolved), [Sensors](electronics-reference.md#sensors-and-signals) |
| Servo, supply dip | Four servo outputs `H9/H10`, supply selection `H11`, `VIN` from 2S battery | [Power supply](electronics-reference.md#power-supply) and [Connectors](electronics-reference.md#connectors) |
| Remote control | `H4`: GND, +5 V, SBUS; inverter `Q1`; ESP32 `GPIO1` | [Connectors](electronics-reference.md#connectors) |
| ros2_pc link or UART | `H2`/`CN4`: UART1 (ros2_pc link); `H3`: UART2; `H1`: UART0 | [Serial interfaces](electronics-reference.md#serial-interfaces) |
| Single resistor, IC pin, or unnamed net | All electrically connected pads on both PCBs | [Component netlist](netlist.md) |
| Touchscreen stand: mounting holes | `touchscreenstand.stl`: rectangle 96.4 × 72.9 mm, Ø 2.2 mm with countersink; dimensions according to the user | [Touchscreen stand](touchscreenstand.md) |
| Flash / USB diagnostics | USB-C → CH340X → UART0; do not assume native USB CDC | [USB/serial guide](../software/usb-serial-flashing.md) |

## Key findings and consequences

- **Native USB CDC requires a hardware change:** In the PCB netlist, USB-C leads to CH340X, not to the native USB pins of the ESP32-S3. GPIO19/20 carry the UART1 link to the ros2_pc. Native USB CDC is therefore not possible through the existing USB-C connector without a hardware change. A firmware option alone is not sufficient. See [serial interfaces](electronics-reference.md#serial-interfaces).

- **No external RAM:** According to user confirmation of 2026-09-28, the ESP32-S3-WROOM-1 is populated without external RAM; the pin assignment is correct. The N8R8 designation in the netlist differs from this hardware statement. Plan only for internal SRAM and do not enable PSRAM.

- **Servo power rail:** `H11` connects the common supply pin of the servo connectors to either `+5V` or `VIN`. Before connecting and loading, check which jumper is actually fitted and the voltage tolerance of the servos. The servos are not connected to the 3.3 V regulator. Source: MAIN PCB netlist (`H9`, `H10`, `H11`).
- **Battery measurement:** The firmware calculates a battery value from the GPIO17 ADC. It is **not a measurement of the 3.3 V rail**. The documented readings drop strongly under balance load; the battery, contact, regulator, and ADC behavior are therefore not yet assessed separately. Source: MAIN netlist `R1/R2`, `readBatteryVoltage()`, and [sensor diagnostic guide](../software/sensor-diagnostic-mode.md).
- **Encoders:** The same AS5600 I²C addresses are possible because the two wheel encoders connect to separate ESP32 I²C controllers: `GPIO4/5` and `GPIO41/42`. Source: MAIN/CODER netlist and firmware initialization.
- **SBUS:** The receiver is supplied with 5 V via `H4`; the signal passes through transistor `Q1`, inverted, to `GPIO1`/UART2. Check the receiver output and pin order before plugging in. Source: MAIN netlist and [SBUS connection sheet](../assets/es02-sbus-connect.pdf).
- **UART1 mapping:** The PCB nets `RX1`/`TX1` are connected to GPIO19/20 according to the ESP32 symbol, but the firmware uses RX=GPIO20 and TX=GPIO19 for UART1. This is a real discrepancy between net naming and the software assignment; do not derive the cable wiring from the names alone. Source: `U4` in the [component netlist](netlist.md) and `link_uart.h`.
- **5 V regulator:** `U1` pin 5 (`EN`) appears as unconnected in the PCB export. This alone does not establish its operating behavior; for 5 V problems, check this pin and the datasheet. Source: [component netlist](netlist.md).
- **Data status:** README, BOM, and schematic name different models for some purchased parts: the BOM lists servo `GX-S7445`, the README `Pro-Tronik PTK 7452 MG-D`. In one place the README also mentions `MPU6050`, while the schematic, BOM, and firmware establish `ICM-42688P`. For pin and circuit questions, use the PCB netlist; for the function currently executed, use the firmware; check the specific purchased parts fitted to the robot.

## Sources and scope

- **Schematic/netlist:** [`cad/robot/ProPrj_NavBot-ES02.epro`](../../../cad/robot/ProPrj_NavBot-ES02.epro), containing `project.json`, MAIN/CODER `.esch` and `.epcb`. The connector pins and net names cited here were read from `PAD_NET` in the respective `.epcb`.
- **Purchased parts:** [`BOM.xlsx`](../assets/bom.xlsx), sheet `Sheet1`, as of the repository state.
- **Firmware mapping:** [`src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`](../../../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino), [`ICM42688.h`](../../../src/ES-02/OllieFOCdrive/ICM42688.h), [`link_uart.h`](../../../src/ES-02/OllieFOCdrive/link_uart.h), [`FUTABA_SBUS.h`](../../../src/ES-02/OllieFOCdrive/FUTABA_SBUS.h).
- **Observations on the existing board:** [USB/serial guide](../software/usb-serial-flashing.md) and [sensor diagnostic guide](../software/sensor-diagnostic-mode.md). These describe measurements and local tests; the netlist describes the design.
