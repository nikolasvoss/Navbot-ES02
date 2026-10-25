# Electronics hardware: technical reference

This page supplements the [system overview](system-overview.md). All component pads and nets are listed in the [component netlist](netlist.md). **Pins are physical connector contacts according to the EasyEDA PCB netlist.** GPIOs are ESP32-S3 numbers according to the schematic symbol and firmware. The contact view and cable direction are therefore not yet fixed. Check the board labeling before plugging anything in. `MAIN` and `CODER` are the two boards in `project.json` of the [EasyEDA project](../../../cad/robot/ProPrj_NavBot-ES02.epro).

## Power supply

**Assembly per user, 2026-09-28:** ESP32-S3-WROOM-1 without external RAM. The existing pin assignment is correct. The N8R8 entry in the netlist does not describe the confirmed RAM equipment. Plan for internal SRAM and do not enable PSRAM.

| Stage | Components / nets | Function and practical consequence |
| --- | --- | --- |
| Battery | External 2S 18650 holder → `CN2`: 1 `B2`, 2 `B1`, 3 `B-` | Center tap `B1`. Do not confuse it with a simple two-pole battery lead. BOM: one 2S holder and two cells. |
| Protection and on/off | `U8` HY2120-LB, `Q2` WSD3056DN, `SW4`/`SW1`; `SW1` routes `B2` to `VIN` | Cell protection and a switched main supply are present in the design. `VIN` supplies the motor drivers directly. The actual cutoff threshold was not measured here. |
| Charging / USB | `M1` "2S充电模块" (2S charging module, IP2326 boost charger; see [charger module](charger-module.md)) with `VBUS`, `B2`, `B1`, `GND`, `VSTA`; USB-C `USB1`; `D5`/`J1` in the path to `+5V` | Charging and USB supply are intended in the design. The module project sets about 1.3 A charge current (`ISET` 68 kΩ) and 8.4 V; not yet verified on the built module. No charging behavior or simultaneous operation under load has been verified. `VSTA` goes to the ESP32 (`GPIO18`). |
| 5 V | `U1` RT8289GSP, `L1` FXL0630-100-M, input net `VIN`, output `+5V` | Switching regulator for 5 V loads and connectors. |
| 3.3 V | `U2` XC6210B332MR from `+5V` to `+3.3V` | Supplies the ESP32, IMU, encoders, and logic. Does not supply the servo load. |
| Servo supply | `H11`: 1 `+5V`, 2 servo supply net, 3 `VIN`; same net on `H9/H10` pins 3/4 | Three-pin select: jumper 1–2 = 5 V, 2–3 = VIN. The PCB project does not show which jumper position is fitted. Before connecting 2S VIN to the servos, check that they are rated for it. |
| Battery voltage | `R1` 10 kΩ from `VIN` to `ADC`, `R2` 1 kΩ to GND, `C1` 100 nF; `ADC` → ESP32 `GPIO17` | Voltage divider of about 1:11. `readBatteryVoltage()` uses a local calibration `7.77 / 813.43 × ADC`. This estimate is not a direct 3.3 V or single-cell value. |

**Checkpoint for the 5 V regulator:** In the PCB netlist export, pin 5 (`EN`) of the RT8289 `U1` is marked as unconnected. The netlist alone does not establish whether this is an internal default, a different real assembly, or a design error. If 5 V is missing, first check this point on the real board and against the component datasheet. The existing board observation does not include a pin 5 measurement.

## Motors and encoders

| Channel | MAIN nets / GPIO | Peripherals |
| --- | --- | --- |
| A / `motor1` | `U6` MP6536; `PWM1_A/2_A/3_A` = GPIO15/7/6, `EN_A` = GPIO16; `H7` pins 1/2/3 = `U_A/V_A/W_A` | Three-phase wheel motor; firmware `BLDCDriver3PWM(15, 7, 6, 16)`. |
| B / `motor2` | `U7` MP6536; `PWM1_B/2_B/3_B` = GPIO40/39/38, `EN_B` = GPIO37; `H8` pins 1/2/3 = `U_B/V_B/W_B` | Second three-phase wheel motor; firmware `BLDCDriver3PWM(40, 39, 38, 37)`. |
| Encoder A | `H5` pins 1 SDA = GPIO4, 2 SCL = GPIO5, 3 GND, 4 +3.3 V; 5/6 GND per PCB pads | `i2cOne.begin(4, 5, 400000)` and `sensor1` AS5600. Pins 5/6 are not part of the four-wire cable. |
| Encoder B | `H6` pins 1 SDA = GPIO41, 2 SCL = GPIO42, 3 GND, 4 +3.3 V; 5/6 GND per PCB pads | `i2cTwo.begin(41, 42, 400000)` and `sensor2` AS5600. Pins 5/6 are not part of the four-wire cable. |

The small **CODER** board contains `U1` AS5600-ASOM, local 3.3 V decoupling, and 10 kΩ pull-ups for SDA/SCL. Its `H1` has the same logical order: 1 SDA, 2 SCL, 3 GND, 4 3.3 V. Additional footprint pads 5/6 are connected to GND and are not extra cable wires. The MAIN pull-up is 3.3 kΩ per I²C line. The BOM lists two AS5600 modules, two 2208-90-kV motors, and two radial 6 × 1.5 mm magnets. The magnet position and electrical zero angle must be calibrated mechanically or with SimpleFOC. They do not follow from the netlist.

**Pole pairs, research of 2026-10-01:** The [SimpleFOC motor documentation](https://docs.simplefoc.com/bldc_motors) lists `12N14P` for the 2208-90KV gimbal motor: 12 stator slots and 14 rotor poles, so **7 pole pairs**. This matches the model designation `2208-90kv` in `BOM.xlsx`, `Sheet1!D26`. Both firmware instances already use `BLDCMotor(7)` as a fixed value. The installed individual motors were not confirmed by magnet counting or an independent measurement. The BOM describes the intended purchased parts.

The [startup capture after the library change](../../../lab/robot/captures/serial-startup-after-foc-update.txt) shows two successful MCPWM initializations and `Motor ready.`, but pole-pair check estimates of 8.46 and 7.80. In the installed SimpleFOC version 2.3.5, `BLDCMotor::alignSensor()` checks the angle travel against the configured value and prints an estimate on deviation. It does not overwrite `pole_pairs`. **Consequence:** keep 7 and treat the warnings as an open deviation to resolve during calibration. This short check alone does not justify deriving 8 pole pairs, and it does not justify suppressing the warning by skipping calibration. A load, incomplete rotor movement, or an encoder fault are possible causes, but none is established here.

## Sensors and signals

| Function | Chip / signal | ESP32-S3 and firmware |
| --- | --- | --- |
| IMU | `U5` ICM-42688P-HXY, SPI `SCLK/MOSI/MISO/CS1` | GPIO3/9/10/13; `ICM42688.cpp` uses SPI mode 3, accelerometer ±8 g and gyro ±2000 °/s at 1 kHz. `INT1/INT2` are present in the design but, according to the netlist, not connected to U4 (R28 pulls INT2 to GND). The firmware shown reads over SPI without FIFO or interrupt. Check the real board before interrupt or FIFO capture. |
| Four leg servos | `SERVO1/2/3/4` | GPIO11/12/21/14, 50 Hz PWM in `ServoControl`. Servos 2/3 are inverted in the standard drive. The GPIO order does not yet say anything about left/right on the mechanics. **Servos 3 and 4 are swapped between the PCB nets and the firmware:** net `SERVO3` is GPIO14 and `SERVO4` is GPIO21, but the firmware drives GPIO21 as servo 3 and GPIO14 as servo 4. Firmware servo 3 is on `H10` pin 2 / `CN3` pin 4, and firmware servo 4 is on `H10` pin 1 / `CN3` pin 3. Do not change either side without checking which leg each servo drives ([lab note](../../../lab/robot/2026-10-20-servo-3-4-gpio-swap.md)). |
| Operating LEDs | `LED2/LED3` | GPIO35/36. `board::kStatusLedPin=35`; LOW switches the firmware LED on. |
| Charging signal | `VSTA` | GPIO18. Not the IP2326 status pin (unconnected). Per the module project, a divider from `BAT+` through `Q2` when USB is present, about 2 V at 8.4 V. See [charger module](charger-module.md). No software evaluation confirmed here. |
| Version identifier | `VERSION_ADC` | GPIO8, divider 180 kΩ/100 kΩ. Present in the design; use by the current firmware not established. |

## Connectors

| Connector | Physical pins per PCB | Purpose / note |
| --- | --- | --- |
| `CN2` | 1 `B2`, 2 `B1`, 3 `B-` | 2S battery with center tap. |
| `H4` | 1 GND, 2 +5 V, 3 SBUS input via `R12/Q1` | External RC receiver; see [SBUS connection sheet](../assets/es02-sbus-connect.pdf). |
| `H5`, `H6` | 1 SDA, 2 SCL, 3 GND, 4 +3.3 V | AS5600 A/B. Additional GND pads 5/6 appear in the footprint netlist. |
| `H7`, `H8` | 1 U, 2 V, 3 W | Three-phase motor connections A/B. |
| `H9` | 1 servo 1, 2 servo 2, 3/4 shared servo supply, 5/6 GND | Leg servos 1/2. |
| `H10` | 1 servo 3, 2 servo 4, 3/4 shared servo supply, 5/6 GND | Leg servos 3/4. Net names: mind the servo 3/4 swap in the [GPIO table](#sensors-and-signals). |
| `H11` | 1 +5 V, 2 servo supply, 3 VIN | Servo supply select. Check the actual jumper before operation. |
| `CN3` | 1–4 servo 1–4, 5–7 GND | Signals/GND. No servo supply on these pins. |
| `H12` | 1 +5 V, 2 GND, 3 SCLK, 4 MOSI, 5 MISO, 6 CS1, 7 unused, 8 GND | Additional SPI header. Do not equate its +5 V with 3.3 V logic levels. |

## Serial interfaces

**Native USB CDC is not possible through the existing USB-C connector without a hardware change.** According to the PCB netlist, `D+` and `D−` connect the `USB1` connector to CH340X `U3`. The native USB pins of the ESP32-S3, GPIO20 for D+ and GPIO19 for D−, are instead on `TX1` and `RX1` for the ros2_pc link. Native USB would require a separate connection to these pins and a new home for UART1. The firmware option "USB CDC On Boot" does not change this wiring. Sources: [PCB netlist](netlist.md), [Espressif USB hardware requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html#hardware-requirements), and [observations on the existing board](../software/usb-serial-flashing.md).

| Path | Connector / net side | Firmware and limitation |
| --- | --- | --- |
| USB/Debug | USB-C `USB1` → CH340X `U3` → `RX0/TX0`; `H1`: 1 +5 V, 2 GND, 3 RX0, 4 TX0 | Flash / `Serial`. Observed on the existing board as CH340 `/dev/ttyUSB*`. Normal mode 2 Mbaud, sensor diagnostics 115200 baud. Read the [USB/serial guide](../software/usb-serial-flashing.md) before making changes. |
| UART1, ros2_pc link | `H2`: 1 +5 V, 2 GND, 3 RX1, 4 TX1; `CN4`: 2 +5 V, 3 GND, 4 RX1, 5 TX1 (1/7 GND) | **PCB net names:** RX1 on GPIO19, TX1 on GPIO20. **Firmware:** `RXD1=20`, `TXD1=19`, which is reversed relative to the net names. The firmware opens UART1 at 500000 baud, 8N1, 3.3 V logic, for the [NavBot link protocol](../../integration/software/ros2-uart-protocol.md). The touchscreen that used this port is removed ([lab note](../../../lab/robot/2026-10-25-uart1-ros2-link.md)). Check the physical line direction before wiring. |
| SBUS / UART2 | `H3`: 1 +5 V, 2 GND, 3 RX2, 4 TX2; `H4`: 1 GND, 2 +5 V, 3 SBUS via `Q1` | `RX2/TX2` = GPIO1/2. `FUTABA_SBUS` starts UART2 at 100000 baud. The transistor inverts the SBUS input for `RX2`. |

`H1/H2/H3` have additional GND pads 5/6 per the PCB footprint, but only four cable wires. For external cables, use contacts 1–4 and the board labeling. The BOM lists three SH1.0 cables of 200 mm (two encoders, connection of two units) and one of 100 mm (formerly the touch board; unused). For the SBUS receiver, a three-pole cable.

## External devices, modes and data quality

- The RC operation is described in the [README](../../../README.md): CH5 stop/start (two gain sets), CH6 posture/"Mark", CH7 roll manual/auto, CH8 standard/pitch/ball balance, CH9/10 ball coordinates. This mapping is firmware and remote-control configuration, not a property of the connector.
- `SENSOR_DIAGNOSTIC_MODE=1` first sets GPIO16/37 LOW, skips motor and servo initialization, and outputs IMU, battery ADC, and RC state as CSV. For the safe diagnostic procedure and the observed voltage dips, read the [sensor diagnostic guide](../software/sensor-diagnostic-mode.md).
- The README contains older or differing hardware information (`MPU6050`, a different servo model). The **PCB netlist** establishes the electrical connections. The **BOM** lists purchased parts. The **firmware** establishes the currently configured GPIOs and operating modes. Assembly, cable orientation, servo voltage jumper, and voltages under load must be checked on the real board.
