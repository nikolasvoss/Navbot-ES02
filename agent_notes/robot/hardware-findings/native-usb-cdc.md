# Native USB CDC constraint, 2026-10-07

## Observation

Native ESP32-S3 USB CDC is unavailable through the existing USB-C connector without hardware changes. This conclusion combines PCB routing, firmware pin use, and existing device observations. No new physical continuity measurement or wiring trial was performed.

## Evidence

- [PCB netlist](../../../docs/robot/hardware/electronics/netlist.md): `USB1` D+/D− connect to CH340X `U3` UD+/UD−. ESP32 module `U4` GPIO19/20 connect to `RX1`/`TX1` instead.
- [Touch firmware](../../../src/ES-02/OllieFOCdrive/touchscreen.h) uses GPIO20 as UART1 RX and GPIO19 as UART1 TX.
- [Espressif hardware requirements](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/usb-serial-jtag-console.html#hardware-requirements) assign native USB D− to GPIO19 and D+ to GPIO20.
- [Existing board observations](../../../docs/robot/software/development/usb-serial-flashing.md): USB-C enumerated as CH340; enabling USB CDC did not expose a native CDC port and routed sketch output away from the observed UART connection.

## Practical consequence

Use the existing CH340 UART path for logging trials. Native USB requires a separate connection to GPIO19/20 and resolution of the touch-UART conflict, with layout and physical pinout verification before wiring. A firmware option cannot change the USB-C data routing. Neither transport removes the need to keep logging transmission from blocking active balance control.
