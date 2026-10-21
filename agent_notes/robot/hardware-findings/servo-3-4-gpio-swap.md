# Servo 3/4 GPIOs swapped between PCB nets and firmware, 2026-10-20

## Observation

The PCB net `SERVO3` is on GPIO14 and `SERVO4` is on GPIO21. The firmware drives GPIO21 as servo 3 and GPIO14 as servo 4. The earlier hardware reference listed "SERVO1/2/3/4 = GPIO11/12/21/14", which matched the firmware but not the net names.

## Evidence

- [Netlist](../../../docs/robot/hardware/electronics/netlist.md), `U4`: `22[IO14]=SERVO3`, `23[IO21]=SERVO4`; `H10`: `1=SERVO3, 2=SERVO4`; `CN3`: `3=SERVO3, 4=SERVO4`.
- Firmware: `LEG_SERVO_3_SIGNAL_PIN 21` and `LEG_SERVO_4_SIGNAL_PIN 14` in [`OllieFOCdrive.ino`](../../../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino); `DEFAULT_SERVO_3_PIN = 21`, `DEFAULT_SERVO_4_PIN = 14` in [`ServoControl.h`](../../../src/ES-02/OllieFOCdrive/ServoControl.h).
- Source comparison only. No servo was moved and no wiring was inspected.

## Practical consequence

Firmware servo 3 is on `H10` pin 2 / `CN3` pin 4, and firmware servo 4 is on `H10` pin 1 / `CN3` pin 3. Whether this swap is intentional (it may compensate for the mechanical servo placement) is unknown. Do not "fix" either side without checking which leg each servo drives. Canonical reference: [GPIO map](../../../docs/robot/hardware/electronics/reference.md#esp32-s3-gpio-map).
