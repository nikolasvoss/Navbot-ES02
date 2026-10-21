# Robot electronics reference

Detailed reference for the robot's electronics. Start with the [hardware overview](../system-overview.md) and open only the section you need here. For a single component pad or an unnamed net, search the [component netlist](netlist.md).

## Conventions

- **Boards.** The [EasyEDA project](../../../../hardware/pcb/ProPrj_NavBot-ES02.epro) contains two boards: **MAIN** (controller, power, motor drivers, connectors) and **CODER** (AS5600 encoder board, two pieces per robot).
- **Pin numbers** are physical connector contacts from the PCB netlist (`PAD_NET` in the `.epcb` files). They do not define the contact view or cable direction. Check the board silkscreen before plugging in.
- **GPIO numbers** are ESP32-S3 GPIOs from the schematic symbol of `U4` and from the firmware.
- **Sources.** The netlist describes the intended design. The firmware describes configured behavior. The BOM lists the intended purchased parts. User statements and captures describe the assembled robot. Each section names its source. Where sources disagree, the [discrepancy list](#source-discrepancies) says so.

## Power

| Stage | Components / nets | Function and practical consequence |
| --- | --- | --- |
| Battery | External 2S 18650 holder → `CN2`: 1 `B2`, 2 `B1`, 3 `B-` | `B1` is the cell midpoint tap. Do not confuse the three-pin plug with a plain two-wire battery lead. BOM: one 2S holder, two cells. |
| Protection and on/off | `U8` HY2120-LB, `Q2` WSD3056DN, `SW4`/`SW1`; `SW1` switches `B2` to `VIN` | Cell protection and a switched main supply exist in the design. `VIN` feeds the motor drivers directly. The actual cut-off threshold has not been measured. |
| Charging / USB | Charger module `M1` (`2S_RECHARGE`, pins `VBUS`, `B2`, `B1`, `GND`, `VSTA`); USB-C `USB1`; `D5`/`J1` in the path to `+5V` | Charging and USB supply are part of the design. Charging behavior and simultaneous operation under load have not been verified. `VSTA` goes to ESP32 GPIO18. |
| 5 V | `U1` RT8289GSP buck, `L1` FXL0630-100-M, input `VIN`, output `+5V` | Supplies the 5 V loads and connectors. See the [5 V regulator check](#5-v-regulator-check). |
| 3.3 V | `U2` XC6210B332MR, `+5V` → `+3.3V` | Supplies the ESP32, IMU, encoders and logic. Not the servo load. |
| Servo supply | `H11`: 1 `+5V`, 2 servo rail, 3 `VIN`; the servo rail is on `H9`/`H10` pins 3/4 | Jumper 1–2 selects 5 V, jumper 2–3 selects VIN (2S). The fitted jumper cannot be read from the PCB project. Check it, and check that the servos tolerate 2S VIN, before powering them. |
| Battery voltage | `R1` 10 kΩ from `VIN` to `ADC`, `R2` 1 kΩ to GND, `C1` 100 nF; `ADC` → GPIO17 | Divider of about 1:11. `ReadVoltage()` uses a local calibration `7.77 / 813.43 × ADC`. The result estimates the pack voltage. It is not a 3.3 V rail reading and not a per-cell value. |

### 5 V regulator check

The PCB netlist export marks pin 5 (`EN`) of the RT8289 `U1` as unconnected. The netlist alone does not tell whether this relies on an internal default, differs from the assembled board, or is a design error. If 5 V is missing, check this pin on the real board against the datasheet first. No measurement of pin 5 exists.

## ESP32-S3 module and memory

**Fact for the assembled robot (user, 2026-09-28).** The board carries an ESP32-S3-WROOM-1 **without external RAM (PSRAM)**, and its pin assignment is correct.

The netlist labels `U4` as ESP32-S3-WROOM-1-**N8R8** and assigns GPIO35–37 to LED2, LED3 and `EN_B`. According to the [Espressif datasheet, section 3, footnote b](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf), GPIO35–37 are not available on octal-PSRAM modules. The user's statement is authoritative for the built robot. Treat the N8R8 label as a design-file discrepancy, not as a real PSRAM pin conflict.

Consequences:

- Plan with internal SRAM only. Keep PSRAM disabled. Do not change pins.
- The free internal buffer capacity under firmware and Wi-Fi load has not been measured.
- The flash size is not determined by the base module name.

## ESP32-S3 GPIO map

Source: MAIN netlist `U4` and the current firmware. "Not used" means the firmware does not reference the pin.

| GPIO | MAIN net | Function | Firmware |
| ---: | --- | --- | --- |
| 0 | `BOOT` | Boot strap, driven by CH340X `U3` (DTR/CTS) | Not used |
| 1 | `RX2` | SBUS input, inverted by `Q1` | UART2 RX, 100000 baud (`FUTABA_SBUS.h`) |
| 2 | `TX2` | `H3` pin 4 | UART2 TX (`FUTABA_SBUS.h`) |
| 3 | `SCLK` | IMU SPI clock, `H12` pin 3 | `SCK_PIN` (`ICM42688.h`) |
| 4 | `SDA2_A` | Encoder A SDA, `H5` pin 1 | `I2Cone.begin(4, 5, 400000)` |
| 5 | `SCL2_A` | Encoder A SCL, `H5` pin 2 | `I2Cone` |
| 6 | `PWM3_A` | Motor driver A, phase 3 | `BLDCDriver3PWM(15, 7, 6, 16)` |
| 7 | `PWM2_A` | Motor driver A, phase 2 | `driver` |
| 8 | `VERSION_ADC` | Board version divider 180 kΩ/100 kΩ | Not used |
| 9 | `MOSI` | IMU SPI MOSI, `H12` pin 4 | `MOSI_PIN` |
| 10 | `MISO` | IMU SPI MISO, `H12` pin 5 | `MISO_PIN` |
| 11 | `SERVO1` | Servo 1, `H9` pin 1, `CN3` pin 1 | Servo 1 |
| 12 | `SERVO2` | Servo 2, `H9` pin 2, `CN3` pin 2 | Servo 2 |
| 13 | `CS1` | IMU chip select, `H12` pin 6 | `CS_PIN` |
| 14 | `SERVO3` | `H10` pin 1, `CN3` pin 3 | **Servo 4** (`LEG_SERVO_4_SIGNAL_PIN`) |
| 15 | `PWM1_A` | Motor driver A, phase 1 | `driver` |
| 16 | `EN_A` | Motor driver A enable, active high | `driver` |
| 17 | `ADC` | Battery divider | `BOARD_PIN_BATTERY_VOLTAGE_ADC` |
| 18 | `VSTA` | Charger status from `M1` | Not used |
| 19 | `RX1` | ros2_pc link, `H2` pin 3, `CN4` pin 4 | **UART1 TX** (`LINK_UART_TX_PIN`) |
| 20 | `TX1` | ros2_pc link, `H2` pin 4, `CN4` pin 5 | **UART1 RX** (`LINK_UART_RX_PIN`) |
| 21 | `SERVO4` | `H10` pin 2, `CN3` pin 4 | **Servo 3** (`LEG_SERVO_3_SIGNAL_PIN`) |
| 35 | `LED2` | Status LED, anode at 3.3 V | `BOARD_PIN_LED`; LOW switches it on |
| 36 | `LED3` | Status LED, anode at 3.3 V | Not used |
| 37 | `EN_B` | Motor driver B enable, active high | `driver2` |
| 38 | `PWM3_B` | Motor driver B, phase 3 | `BLDCDriver3PWM(40, 39, 38, 37)` |
| 39 | `PWM2_B` | Motor driver B, phase 2 | `driver2` |
| 40 | `PWM1_B` | Motor driver B, phase 1 | `driver2` |
| 41 | `SDA2_B` | Encoder B SDA, `H6` pin 1 | `I2Ctwo.begin(41, 42, 400000)` |
| 42 | `SCL2_B` | Encoder B SCL, `H6` pin 2 | `I2Ctwo` |
| `TXD0` | `TX0` | Module pin 37 → CH340X | `Serial` (USB) |
| `RXD0` | `RX0` | Module pin 36 ← CH340X | `Serial` (USB) |

Module pads 16, 24, 25 and 26 are unconnected. For UART0 the netlist names only the module pins `TXD0`/`RXD0`, not GPIO numbers.

Bold firmware entries differ from the net names. See [source discrepancies](#source-discrepancies).

## Motors and encoders

| Channel | MAIN nets / GPIO | Peripheral |
| --- | --- | --- |
| A / `motor1` | `U6` MP6536; `PWM1_A/2_A/3_A` = GPIO15/7/6, `EN_A` = GPIO16; `H7` pins 1/2/3 = `U_A/V_A/W_A` | Three-phase wheel motor; firmware `BLDCDriver3PWM(15, 7, 6, 16)`. |
| B / `motor2` | `U7` MP6536; `PWM1_B/2_B/3_B` = GPIO40/39/38, `EN_B` = GPIO37; `H8` pins 1/2/3 = `U_B/V_B/W_B` | Second three-phase wheel motor; firmware `BLDCDriver3PWM(40, 39, 38, 37)`. |
| Encoder A | `H5` pins 1 SDA = GPIO4, 2 SCL = GPIO5, 3 GND, 4 +3.3 V | `I2Cone.begin(4, 5, 400000)`, `sensor1` AS5600. |
| Encoder B | `H6` pins 1 SDA = GPIO41, 2 SCL = GPIO42, 3 GND, 4 +3.3 V | `I2Ctwo.begin(41, 42, 400000)`, `sensor2` AS5600. |

Both AS5600 encoders have the same fixed I²C address. This works because each sits on its own ESP32 I²C controller (GPIO4/5 and GPIO41/42).

The **CODER** board carries `U1` AS5600-ASOM, local 3.3 V decoupling and 10 kΩ pull-ups on SDA/SCL. Its `H1` has the same order as `H5`/`H6`: 1 SDA, 2 SCL, 3 GND, 4 3.3 V. MAIN adds a 3.3 kΩ pull-up per I²C line. The BOM lists two AS5600 modules, two 2208 90 kV motors and two radial 6 × 1.5 mm magnets. Magnet position and the electrical zero angle must be calibrated mechanically and with SimpleFOC; the netlist does not define them.

### Pole pairs

The [SimpleFOC motor documentation](https://docs.simplefoc.com/bldc_motors) lists the 2208 90 kV gimbal motor as `12N14P`: 12 stator slots and 14 rotor poles, so **7 pole pairs** (research of 2026-10-01). This matches the model `2208-90kv` in `BOM.xlsx`, `Sheet1!D26`. Both firmware motors use `BLDCMotor(7)`. Nobody has counted the magnets or measured the installed motors.

A startup capture after the SimpleFOC library update (local file `build/serial-startup-after-foc-update.txt`, not in git) showed two successful MCPWM initializations and `Motor ready.`, but pole-pair check estimates of 8.46 and 7.80. In SimpleFOC 2.3.5, `BLDCMotor::alignSensor()` compares the angle travelled with the configured value and prints an estimate when they differ. It does not overwrite `pole_pairs`.

**Consequence.** Keep 7. Treat the warnings as an open calibration deviation. Do not derive 8 pole pairs from this short alignment, and do not suppress the warning by skipping calibration. Load, incomplete rotor motion or an encoder fault are possible causes; none is proven.

## Sensors and signals

| Function | Chip / signal | ESP32-S3 and firmware |
| --- | --- | --- |
| IMU | `U5` ICM-42688P-HXY on SPI `SCLK/MOSI/MISO/CS1` | GPIO3/9/10/13. `ICM42688.cpp` uses SPI mode 3, accelerometer ±8 g, gyro ±2000 °/s at 1 kHz. |
| IMU interrupts | `U5` `INT1`/`INT2` | Not connected to the ESP32 in the netlist; `R28` pulls `INT2` to GND with 10 kΩ. See [IMU interrupts](#imu-interrupts). |
| Four leg servos | `SERVO1`–`SERVO4` | GPIO11/12/14/21 by net name, 50 Hz PWM in `ServoControl`. Firmware servos 2 and 3 are inverted in the default drive. The GPIO order says nothing about left/right on the mechanics. |
| Status LEDs | `LED2`/`LED3` | GPIO35/36. `BOARD_PIN_LED=35`; LOW switches the LED on. |
| Charger status | `VSTA` | GPIO18, from `M1`. Not read by the firmware. |
| Board version | `VERSION_ADC` | GPIO8, divider 180 kΩ/100 kΩ. Not read by the firmware. |

### IMU interrupts

The exported `U5` symbol has `INT1`/`INT2`, but `U4` has no matching nets. The IMU driver reads data registers over SPI and uses no FIFO or interrupt path.

**Consequence.** Do not assume an interrupt line to the ESP32. For FIFO capture, either use bounded polling, or plan a connection after checking the layout and the real board.

## Connectors

All MAIN connectors. Pin numbers are from the PCB netlist.

| Connector | Type (netlist) | Pins | Purpose / note |
| --- | --- | --- | --- |
| `CN2` | HX25003-3WAP | 1 `B2`, 2 `B1`, 3 `B-` | 2S battery with midpoint tap. |
| `USB1` | USB-C 16-pin | D+/D− to CH340X `U3`, `VBUS` | Flash, serial and charging. See [serial interfaces](#serial-interfaces). |
| `H1` | SH1.0 4-pin | 1 +5 V, 2 GND, 3 `RX0`, 4 `TX0` | UART0 (same signals as the USB bridge). |
| `H2` | SH1.0 4-pin | 1 +5 V, 2 GND, 3 `RX1`, 4 `TX1` | UART1, ros2_pc link. |
| `CN4` | SH1.0 5-pin | 1 GND, 2 +5 V, 3 GND, 4 `RX1`, 5 `TX1` | UART1, ros2_pc link (second footprint). |
| `H3` | SH1.0 4-pin | 1 +5 V, 2 GND, 3 `RX2`, 4 `TX2` | UART2. |
| `H4` | 2.54 mm 3-pin | 1 GND, 2 +5 V, 3 SBUS in via `R12`/`Q1` | External RC receiver; see the [SBUS connection sheet](../connectors/ES02-SBUS-connect.pdf) and [wiring photo](../connectors/ES02-SBUS-wire.png). |
| `H5`, `H6` | SH1.0 4-pin | 1 SDA, 2 SCL, 3 GND, 4 +3.3 V | AS5600 encoders A/B. |
| `H7`, `H8` | 2.54 mm 3-pin | 1 U, 2 V, 3 W | Motor phases A/B. |
| `H9` | PH2.54 | 1 servo 1, 2 servo 2, 3/4 servo rail, 5/6 GND | Leg servos 1/2. |
| `H10` | PH2.54 | 1 `SERVO3`, 2 `SERVO4`, 3/4 servo rail, 5/6 GND | Leg servos 3/4. Mind the [servo 3/4 swap](#source-discrepancies). |
| `H11` | 2.54 mm 3-pin | 1 +5 V, 2 servo rail, 3 VIN | Servo supply jumper. Check the fitted jumper before use. |
| `CN3` | SH1.0 5-pin | 1–4 `SERVO1`–`SERVO4`, 5–7 GND | Servo signals and GND only, no servo supply. |
| `H12` | 2.54 mm 1×8 female | 1 +5 V, 2 GND, 3 SCLK, 4 MOSI, 5 MISO, 6 CS1, 7 unconnected, 8 GND | Extra SPI header. Its +5 V pin does not mean 5 V logic; SPI signals are 3.3 V. |

Footprint pads beyond the cable contacts are GND: pads 5/6 on `H1`, `H2`, `H3`, `H5`, `H6`, the CODER `H1`, and pads 6/7 on `CN4`. They are not extra cable wires. Use contacts 1–4 (or 1–5) and the silkscreen.

The BOM lists three SH1.0 cables of 200 mm (two encoders, one link between two units), one of 100 mm (originally for the touchscreen, now removed) and a three-pin cable for the SBUS receiver.

## Serial interfaces

**Native USB CDC is not possible over the existing USB-C connector without a hardware change.** `USB1` D+/D− connect to the CH340X `U3`, not to the ESP32. The ESP32-S3 native USB pins, GPIO20 (D+) and GPIO19 (D−), carry `TX1`/`RX1` for the ros2_pc link. Native USB would need a separate connection to these pins and a new home for UART1. The firmware option "USB CDC On Boot" does not change this wiring. Sources: [netlist](netlist.md), [Espressif USB pin requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html#hardware-requirements), [observations on the board](../../software/development/usb-serial-flashing.md), [finding](../../../../agent_notes/robot/hardware-findings/native-usb-cdc.md).

| Path | Connector / nets | Firmware and limits |
| --- | --- | --- |
| USB / debug | `USB1` → CH340X `U3` → `RX0`/`TX0`; also `H1` | `Serial` at 576000 baud in all modes (`SERIAL_BAUD_RATE`, `DIAGNOSTIC_SERIAL_BAUD_RATE`). Seen on the PC as CH340 `1a86:7523`, `/dev/ttyUSB*`. Read the [USB/serial guide](../../software/development/usb-serial-flashing.md) before changing anything. |
| UART1, ros2_pc link | `H2` or `CN4`, nets `RX1`/`TX1` | **Firmware RX = GPIO20 (net `TX1`), TX = GPIO19 (net `RX1`).** 500000 baud, 8N1, [NavBot link protocol](../../software/interfaces/ros2-uart-protocol.md). 3.3 V logic. The touchscreen that used this port is removed. Check the physical signal direction before wiring. |
| UART2, SBUS | `H4` (receiver) and `H3`; nets `RX2`/`TX2` = GPIO1/2 | `FUTABA_SBUS` opens UART2 at 100000 baud. `Q1` inverts the SBUS signal into `RX2`. Check the receiver output and pin order before plugging in. |

## Source discrepancies

| Topic | What differs | Use |
| --- | --- | --- |
| UART1 direction | Net `RX1` is GPIO19, but the firmware uses GPIO19 as **TX**; net `TX1` is GPIO20, used as **RX**. | Derive cable wiring from GPIO numbers and the firmware, not from net names. |
| Servos 3/4 | Net `SERVO3` is GPIO14 and `SERVO4` is GPIO21. The firmware drives GPIO21 as servo 3 and GPIO14 as servo 4. | Firmware servo 3 is on `H10` pin 2 / `CN3` pin 4; firmware servo 4 is on `H10` pin 1 / `CN3` pin 3. |
| ESP32 module | Netlist says N8R8 (octal PSRAM); the user confirmed a module without PSRAM. | Follow the user statement; see [module and memory](#esp32-s3-module-and-memory). |
| Servo model | BOM: `GX-S7445`. Root README: `Pro-Tronik PTK 7452 MG-D`. | Check the fitted servos. |
| IMU | Root README mentions `MPU6050` once. Schematic, BOM and firmware use `ICM-42688P`. | `ICM-42688P`. |

For pin and circuit questions use the netlist. For configured behavior use the firmware. For purchased parts check the real robot. Fitted parts, cable orientation, the servo supply jumper and voltages under load must be checked on the real board.

## Related software pages

- RC channel assignment (CH5 gain set, CH6 posture, CH7 roll, CH8 mode, CH9/CH10 ball pose) is firmware and transmitter configuration, not a connector property. See the [RC channel map](../../software/control/rc-channel-map.md).
- `SENSOR_DIAGNOSTIC_MODE=1` drives GPIO16/37 LOW, skips motor and servo setup, and streams IMU, battery ADC and RC state as CSV. Read the [sensor diagnostic guide](../../software/diagnostics/sensor-diagnostic-mode.md) before collecting data.
