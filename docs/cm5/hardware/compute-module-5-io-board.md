# Raspberry Pi Compute Module 5 IO Board pinout

## Scope and status

This page is a manufacturer reference for the Raspberry Pi Compute Module 5 IO Board (CM5IO), the standard carrier shown in the cited Raspberry Pi documents. It does not confirm that the project has selected, bought, inspected, or installed a CM5IO. The project’s CM5 hardware remains pending.

The current Raspberry Pi IO Board datasheet is document RP-008182, revision 2. Its history records the Rev 2 hardware changes dated 1 August 2025. Raspberry Pi’s design-files page lists the Rev 2 KiCad files, updated 6 October 2025. The product portal identifies the IO Board as SC1751 and SC1967. Check the revision printed on the physical board before using this reference for wiring. See the [CM5IO datasheet](https://pip-assets.raspberrypi.com/categories/1097-raspberry-pi-compute-module-5-io-board/documents/RP-008182-DS-2-cm5io-datasheet.pdf), [CM5IO Rev 2 design files](https://pip.raspberrypi.com/categories/1098-design-files), and [CM5IO product page](https://www.raspberrypi.com/products/compute-module-5-io-board/).

The GPIO alternate functions below come from the [Compute Module 5 datasheet](https://pip-assets.raspberrypi.com/categories/944-raspberry-pi-compute-module-5/documents/RP-008180-DS-6-cm5-datasheet.pdf?disposition=inline), Table 1. They describe the CM5/RP1 mux, not a promise that every function is enabled in the project’s operating system. The board-specific connector and configuration details come from the CM5IO datasheet. Raspberry Pi’s [camera connector pinout](https://www.raspberrypi.com/documentation/accessories/camera.html#pinout-information) and [40-pin HAT+ specification](https://datasheets.raspberrypi.com/hat/hat-plus-specification.pdf) provide connector orientation and standard-header context.

## GPIO header J8

J8 is the standard 40-pin, 2 × 20, 2.54 mm-pitch Raspberry Pi header. Physical pin numbers below follow the standard HAT+ numbering. Use the pin 1 marker on the board and the board drawing when locating pins; do not infer pin 1 from a cable view.

| Physical pins, odd column | Signal | Physical pins, even column | Signal |
| ---: | --- | ---: | --- |
| 1 | 3V3 | 2 | 5V |
| 3 | GPIO2 / I2C1 SDA | 4 | 5V |
| 5 | GPIO3 / I2C1 SCL | 6 | GND |
| 7 | GPIO4 | 8 | GPIO14 / UART0 TX |
| 9 | GND | 10 | GPIO15 / UART0 RX |
| 11 | GPIO17 | 12 | GPIO18 |
| 13 | GPIO27 | 14 | GND |
| 15 | GPIO22 | 16 | GPIO23 |
| 17 | 3V3 | 18 | GPIO24 |
| 19 | GPIO10 / SPI0 SIO0 (MOSI) | 20 | GND |
| 21 | GPIO9 / SPI0 SIO1 (MISO) | 22 | GPIO25 |
| 23 | GPIO11 / SPI0 SCLK | 24 | GPIO8 / SPI0 CE0 |
| 25 | GND | 26 | GPIO7 / SPI0 CE1 |
| 27 | GPIO0 / ID_SD | 28 | GPIO1 / ID_SC |
| 29 | GPIO5 | 30 | GND |
| 31 | GPIO6 | 32 | GPIO12 |
| 33 | GPIO13 | 34 | GND |
| 35 | GPIO19 | 36 | GPIO16 |
| 37 | GPIO26 | 38 | GPIO20 |
| 39 | GND | 40 | GPIO21 |

The bus names in the table are common mappings, not fixed assignments. The complete documented mux options follow. `a0` through `a8` are the alternate-function selector values from Table 1 of the CM5 datasheet. A selector omitted from a row has no named function in that table and is reserved.

| GPIO | J8 pin | Alternate functions |
| ---: | ---: | --- |
| 0 | 27 | a0 SPI0_SIO[3]; a1 DPI_PCLK; a2 UART1_TX; a3 I2C0_SDA; a5 SYS_RIO[0]; a6 PROC_RIO[0]; a7 PIO[0]; a8 SPI2_CSn[0] |
| 1 | 28 | a0 SPI0_SIO[2]; a1 DPI_DE; a2 UART1_RX; a3 I2C0_SCL; a5 SYS_RIO[1]; a6 PROC_RIO[1]; a7 PIO[1]; a8 SPI2_SIO[1] |
| 2 | 3 | a0 SPI0_CSn[3]; a1 DPI_VSYNC; a2 UART1_CTS; a3 I2C1_SDA; a4 UART0_IR_RX; a5 SYS_RIO[2]; a6 PROC_RIO[2]; a7 PIO[2]; a8 SPI2_SIO[0] |
| 3 | 5 | a0 SPI0_CSn[2]; a1 DPI_HSYNC; a2 UART1_RTS; a3 I2C1_SCL; a4 UART0_IR_TX; a5 SYS_RIO[3]; a6 PROC_RIO[3]; a7 PIO[3]; a8 SPI2_SCLK |
| 4 | 7 | a0 GPCLK[0]; a1 DPI_D[0]; a2 UART2_TX; a3 I2C2_SDA; a4 UART0_RI; a5 SYS_RIO[4]; a6 PROC_RIO[4]; a7 PIO[4]; a8 SPI3_CSn[0] |
| 5 | 29 | a0 GPCLK[1]; a1 DPI_D[1]; a2 UART2_RX; a3 I2C2_SCL; a4 UART0_DTR; a5 SYS_RIO[5]; a6 PROC_RIO[5]; a7 PIO[5]; a8 SPI3_SIO[1] |
| 6 | 31 | a0 GPCLK[2]; a1 DPI_D[2]; a2 UART2_CTS; a3 I2C3_SDA; a4 UART0_DCD; a5 SYS_RIO[6]; a6 PROC_RIO[6]; a7 PIO[6]; a8 SPI3_SIO[0] |
| 7 | 26 | a0 SPI0_CSn[1]; a1 DPI_D[3]; a2 UART2_RTS; a3 I2C3_SCL; a4 UART0_DSR; a5 SYS_RIO[7]; a6 PROC_RIO[7]; a7 PIO[7]; a8 SPI3_SCLK |
| 8 | 24 | a0 SPI0_CSn[0]; a1 DPI_D[4]; a2 UART3_TX; a3 I2C0_SDA; a5 SYS_RIO[8]; a6 PROC_RIO[8]; a7 PIO[8]; a8 SPI4_CSn[0] |
| 9 | 21 | a0 SPI0_SIO[1]; a1 DPI_D[5]; a2 UART3_RX; a3 I2C0_SCL; a5 SYS_RIO[9]; a6 PROC_RIO[9]; a7 PIO[9]; a8 SPI4_MISO |
| 10 | 19 | a0 SPI0_SIO[0]; a1 DPI_D[6]; a2 UART3_CTS; a3 I2C1_SDA; a5 SYS_RIO[10]; a6 PROC_RIO[10]; a7 PIO[10]; a8 SPI4_MOSI |
| 11 | 23 | a0 SPI0_SCLK; a1 DPI_D[7]; a2 UART3_RTS; a3 I2C1_SCL; a5 SYS_RIO[11]; a6 PROC_RIO[11]; a7 PIO[11]; a8 SPI4_SCLK |
| 12 | 32 | a0 PWM0[0]; a1 DPI_D[8]; a2 UART4_TX; a3 I2C2_SDA; a4 AUDIO_OUT_L; a5 SYS_RIO[12]; a6 PROC_RIO[12]; a7 PIO[12]; a8 SPI5_CSn[0] |
| 13 | 33 | a0 PWM0[1]; a1 DPI_D[9]; a2 UART4_RX; a3 I2C2_SCL; a4 AUDIO_OUT_R; a5 SYS_RIO[13]; a6 PROC_RIO[13]; a7 PIO[13]; a8 SPI5_SIO[1] |
| 14 | 8 | a0 PWM0[2]; a1 DPI_D[10]; a2 UART4_CTS; a3 I2C3_SDA; a4 UART0_TX; a5 SYS_RIO[14]; a6 PROC_RIO[14]; a7 PIO[14]; a8 SPI5_SIO[0] |
| 15 | 10 | a0 PWM0[3]; a1 DPI_D[11]; a2 UART4_RTS; a3 I2C3_SCL; a4 UART0_RX; a5 SYS_RIO[15]; a6 PROC_RIO[15]; a7 PIO[15]; a8 SPI5_SCLK |
| 16 | 36 | a0 SPI1_CSn[2]; a1 DPI_D[12]; a2 UART0_CTS; a5 SYS_RIO[16]; a6 PROC_RIO[16]; a7 PIO[16] |
| 17 | 11 | a0 SPI1_CSn[1]; a1 DPI_D[13]; a2 UART0_RTS; a5 SYS_RIO[17]; a6 PROC_RIO[17]; a7 PIO[17] |
| 18 | 12 | a0 SPI1_CSn[0]; a1 DPI_D[14]; a2 I2S0_SCLK; a3 PWM0[2]; a4 I2S1_SCLK; a5 SYS_RIO[18]; a6 PROC_RIO[18]; a7 PIO[18]; a8 GPCLK[1] |
| 19 | 35 | a0 SPI1_SIO[1]; a1 DPI_D[15]; a2 I2S0_WS; a3 PWM0[3]; a4 I2S1_WS; a5 SYS_RIO[19]; a6 PROC_RIO[19]; a7 PIO[19] |
| 20 | 38 | a0 SPI1_SIO[0]; a1 DPI_D[16]; a2 I2S0_SDI[0]; a3 GPCLK[0]; a4 I2S1_SDI[0]; a5 SYS_RIO[20]; a6 PROC_RIO[20]; a7 PIO[20] |
| 21 | 40 | a0 SPI1_SCLK; a1 DPI_D[17]; a2 I2S0_SDO[0]; a3 GPCLK[1]; a4 I2S1_SDO[0]; a5 SYS_RIO[21]; a6 PROC_RIO[21]; a7 PIO[21] |
| 22 | 15 | a0 SDIO0_CLK; a1 DPI_D[18]; a2 I2S0_SDI[1]; a3 I2C3_SDA; a4 I2S1_SDI[1]; a5 SYS_RIO[22]; a6 PROC_RIO[22]; a7 PIO[22] |
| 23 | 16 | a0 SDIO0_CMD; a1 DPI_D[19]; a2 I2S0_SDO[1]; a3 I2C3_SCL; a4 I2S1_SDO[1]; a5 SYS_RIO[23]; a6 PROC_RIO[23]; a7 PIO[23] |
| 24 | 18 | a0 SDIO0_DAT[0]; a1 DPI_D[20]; a2 I2S0_SDI[2]; a4 I2S1_SDI[2]; a5 SYS_RIO[24]; a6 PROC_RIO[24]; a7 PIO[24]; a8 SPI2_CSn[1] |
| 25 | 22 | a0 SDIO0_DAT[1]; a1 DPI_D[21]; a2 I2S0_SDO[2]; a3 AUDIO_IN_CLK; a4 I2S1_SDO[2]; a5 SYS_RIO[25]; a6 PROC_RIO[25]; a7 PIO[25]; a8 SPI3_CSn[1] |
| 26 | 37 | a0 SDIO0_DAT[2]; a1 DPI_D[22]; a2 I2S0_SDI[3]; a3 AUDIO_IN_DAT0; a4 I2S1_SDI[3]; a5 SYS_RIO[26]; a6 PROC_RIO[26]; a7 PIO[26]; a8 SPI5_CSn[1] |
| 27 | 13 | a0 SDIO0_DAT[3]; a1 DPI_D[23]; a2 I2S0_SDO[3]; a3 AUDIO_IN_DAT1; a4 I2S1_SDO[3]; a5 SYS_RIO[27]; a6 PROC_RIO[27]; a7 PIO[27]; a8 SPI1_CSn[1] |

### Mux rules and common buses

Each GPIO selects one function at a time. Assign a peripheral input to one GPIO only. If the same peripheral input is mapped to multiple GPIOs, the input receives their logical OR. Alternate functions also overlap across peripherals, so a GPIO assigned to one function is unavailable for its other functions.

| Peripheral | GPIO mapping options from the CM5 mux table |
| --- | --- |
| I2C0 | GPIO0/1 or GPIO8/9 |
| I2C1 | GPIO2/3 or GPIO10/11 |
| I2C2 | GPIO4/5 or GPIO12/13 |
| I2C3 | GPIO6/7, GPIO14/15, or GPIO22/23 |
| UART1 | TX/RX GPIO0/1; CTS/RTS GPIO2/3 |
| UART2 | TX/RX GPIO4/5; CTS/RTS GPIO6/7 |
| UART3 | TX/RX GPIO8/9; CTS/RTS GPIO10/11 |
| UART4 | TX/RX GPIO12/13; CTS/RTS GPIO14/15 |
| UART0 | TX/RX GPIO14/15; CTS/RTS GPIO16/17; DTR/RI GPIO5/4; DCD/DSR GPIO6/7; IrDA TX/RX GPIO3/2 |

For conventional SPI, SIO0 is MOSI and SIO1 is MISO. SPI0 uses GPIO8–11 for CE0, SIO0, SIO1, and SCLK in the common mapping; CE1 is GPIO7. The complete SPI alternate assignments, including the other controllers, are listed per GPIO above. The CM5 datasheet lists SPI0 as a four-chip-select Quad master, SPI1 as a three-chip-select Dual master, SPI2 and SPI3 as two-chip-select Dual masters, SPI4 as a single-chip-select Single slave, and SPI5 as a two-chip-select Dual master.

DPI uses GPIO0–27 for its clock, sync, and up to 24 data signals. SDIO uses GPIO22–27. I2S, PWM, audio, and clock outputs share pins with these and with the bus mappings above. GPIO0 and GPIO1 are the HAT identification signals `ID_SD` and `ID_SC`; they are also associated with MIPI1 I2C. The CM5 datasheet says they may be repurposed when that bus is unused, with `force_eeprom_read=0` in `config.txt` to stop the firmware from checking for a HAT EEPROM. The HAT+ specification reserves these pins for HAT identification.

The CM5 has additional RP1 GPIO signals that are not on J8. For example, GPIO29 is used for fan tachometer, GPIO34 and GPIO35 for camera control, GPIO38 and GPIO39 for MIPI0 I2C, and GPIO42 for USB VBUS enable. Do not treat these as J8 pins.

## Other connectors used for external devices

| Connector | Interface and pin information |
| --- | --- |
| CAM/DISP0 and CAM/DISP1 | Two 22-pin, 0.5 mm-pitch FFC connectors. Each carries four CSI-2 data lanes, a clock pair, two camera GPIOs, I2C, ground, and 3V3. Use the pin table below. CAM/DISP1 needs both J6 I2C jumpers fitted; it has no camera power-down signal. CAM/DISP0 has camera power-down support. |
| J14 fan | Four-pin, 1 mm-pitch JST-SH PWM fan connector. Pinout is listed below. It uses the 5 V rail. Some fans can run while CM5 is shut down because PWM stops. |
| USB-A | Two USB 3.0 host ports. Their VBUS current limit is approximately 1.2 A combined. |
| USB-C data | USB 2.0 data port, primarily for `rpiboot` and board updates. It is separate from the J11 power input. |
| J11 USB-C power | Main 5 V input. A compatible USB-C PD supply negotiates 5 V at 5 A. A standard Raspberry Pi 5 supply is also supported. |
| J8 5 V pins | J8 can also supply the board with external 5 V. The load depends on the CM5 and attached peripherals. Do not connect a second supply in parallel with J11 or PoE. |
| HDMI0 and HDMI1 | Two full-size HDMI 2.0 connectors. Their 5 V supply is current limited. |
| Ethernet RJ45 and J9 | Gigabit Ethernet with PoE support. The raw PoE signals route from the RJ45 magnetics to J9 for a PoE HAT. J9 is not a general-purpose power or GPIO connector. Use the revision-matched schematic for any carrier design; this page does not specify a custom PoE harness. |
| M.2 M-key | PCIe Gen 2 ×1 by default, 5 Gb/s, for standard M-key cards such as NVMe SSDs. PCIe Gen 3 ×1 is experimental and unsupported. Check card length, keying, power, and OS driver compatibility. |
| microSD | Available only with CM5 Lite modules without eMMC. |
| RTC battery | CR2032 battery socket for the real-time clock. |
| J2 | Boot, EEPROM write-protect, sync, PMIC enable, and wake/shutdown controls. See the pin table below. |
| J3 | Optional and not fitted by default. Three-pin Wi-Fi/Bluetooth disable header. See below. |

### CAM/DISP 22-pin FFC pinout

The signal names and directions below follow Raspberry Pi’s 22-pin camera connector reference. The data and clock pairs are MIPI D-PHY signals, not general-purpose logic pins. Pin 1 is marked by a small dot on CM5 IO Boards.

| Pin | Signal | Pin | Signal |
| ---: | --- | ---: | --- |
| 1 | GND | 2 | CAM_DN0 |
| 3 | CAM_DP0 | 4 | GND |
| 5 | CAM_DN1 | 6 | CAM_DP1 |
| 7 | GND | 8 | CAM_CN |
| 9 | CAM_CP | 10 | GND |
| 11 | CAM_DN2 | 12 | CAM_DP2 |
| 13 | GND | 14 | CAM_DN3 |
| 15 | CAM_DP3 | 16 | GND |
| 17 | CAM_IO0 / CAM_GPIO0 | 18 | CAM_IO1 / CAM_GPIO1 |
| 19 | GND | 20 | SCL |
| 21 | SDA | 22 | 3V3 |

The camera I2C lines are pulled up to 3.3 V on the board. The function and direction of `CAM_IO0` and `CAM_IO1` depend on the connected camera or display.

### J14 fan pinout

| Pin | Signal | Standard wire colour |
| ---: | --- | --- |
| 1 | +5V | Red |
| 2 | PWM | Blue |
| 3 | GND | Black |
| 4 | Tach | Yellow |

Raspberry Pi documents the fan header as a 1 mm-pitch JST-SH socket. CM5IO Rev 2 adds a pull-up on the fan output for broader fan support. Treat PWM and tach as logic signals, not as power outputs.

### J2 and J3 control headers

| Header pin | Signal | Function |
| ---: | --- | --- |
| J2 1–2 | nRPIBOOT | Jumper forces USB boot instead of on-board eMMC. |
| J2 3–4 | EEPROM_nWP | Jumper write-protects the CM5 EEPROM. |
| J2 6 | SYNC_OUT | IEEE 1588 timing signal; may also be configured as an external timing input. |
| J2 12 | PMIC_ENABLE | Power-management control. |
| J2 13–14 | Wake/shutdown button | Connects an external push button for wake or shutdown. |
| J3 1 | WL_nDIS | Connect to J3 pin 2 (GND) to disable Wi-Fi. |
| J3 2 | GND | Ground reference for J3. |
| J3 3 | BT_nDIS | Connect to J3 pin 2 (GND) to disable Bluetooth. |

J6 has two jumpers that route I2C to CAM/DISP1. Fit both when using CAM/DISP1. The CM5IO datasheet does not give a separate signal pinout for J6.

## Electrical limits

- The CM5IO sets GPIO Vref to 3.3 V by default through R5. Moving R5 to R4 selects 1.8 V. Fit only one of R4 or R5; the change requires soldering.
- The CM5 datasheet limits the total load across all 28 GPIO pins to 50 mA. This is an aggregate limit, not a per-pin allowance. GPIO2 and GPIO3 have 1.8 kΩ pull-ups.
- The CM5 GPIO voltage must match Vref. Do not connect a 5 V logic output directly to J8. Use a suitable level shifter for signals above the selected Vref.
- The CM5 datasheet allows an externally generated 2.5 V GPIO_VREF only under specified power-sequencing and discharge conditions. This is not the CM5IO’s R4/R5 selection and is not a stock-board wiring option.
- The two USB-A ports share an approximately 1.2 A VBUS limit. The fan also draws from the 5 V rail. USB and fan loads reduce current available elsewhere.
- J11 is the main power input. The board also permits a 5 V input through J8 and can receive 5 V through its PoE HAT path. Use one designed supply path; do not parallel independent power sources.
- J8 does not expose an ADC input. Use an external ADC for analog sensors.

The CM5 datasheet allows 600 mA from each 3.3 V and 1.8 V peripheral rail. That regulator capacity does not raise the separate 50 mA aggregate GPIO limit. The available current also depends on the chosen supply and the other board loads.

## Peripheral and controller details

The CM5 datasheet lists five UARTs, four I2C controllers, six SPI controllers, one 4-bit SDIO interface, four PWM channels, two I2S interfaces, two digital PDM audio inputs, two PWM audio outputs, two GPCLK outputs, and a 24-bit DPI interface. Many share J8 pins. The full per-pin table above is the source of truth when selecting a set of functions.

`SPI4` is device/slave mode; SPI0–SPI3 and SPI5 are master mode. Conventional SPI uses SIO0 as MOSI and SIO1 as MISO. The other SIO lanes support wider SPI modes.

The CM5IO standard 40-pin header does not include the CM4-style extra 4-pin PoE header. CM5IO routes PoE signals to J9 for the optional PoE HAT. Do not reuse a CM4 IO Board PoE header pinout.

## Sources

- [Raspberry Pi Compute Module 5 IO Board datasheet, RP-008182, revision 2](https://pip-assets.raspberrypi.com/categories/1097-raspberry-pi-compute-module-5-io-board/documents/RP-008182-DS-2-cm5io-datasheet.pdf). See sections 2–5 and schematic figures on pages 12–14. The datasheet history lists the Rev 2 hardware changes on 1 August 2025. Product portal entry last checked 4 October 2026.
- [Raspberry Pi Compute Module 5 datasheet, RP-008180, revision 6](https://pip-assets.raspberrypi.com/categories/944-raspberry-pi-compute-module-5/documents/RP-008180-DS-6-cm5-datasheet.pdf?disposition=inline). See sections 2.6 and 2.9, Tables 1 and 2, and the GPIO power details.
- [CM5 IO Board Rev 2 KiCad design files](https://pip.raspberrypi.com/categories/1098-design-files). The product information portal lists the Rev 2 files as updated 6 October 2025.
- [Raspberry Pi 22-pin camera connector pinout](https://www.raspberrypi.com/documentation/accessories/camera.html#pinout-information).
- [Raspberry Pi HAT+ specification](https://datasheets.raspberrypi.com/hat/hat-plus-specification.pdf).
- [Raspberry Pi computer hardware documentation, Raspberry Pi 5 fan connector pinout](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#raspberry-pi-5-fan-connector-pinout).
