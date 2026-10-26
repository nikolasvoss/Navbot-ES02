# Servo 3/4 GPIOs swapped between PCB nets and firmware, 2026-10-20

Status: open

## Observation

The PCB net `SERVO3` is on GPIO14 and `SERVO4` is on GPIO21. The firmware drives GPIO21 as servo 3 and GPIO14 as servo 4. The earlier hardware reference listed "SERVO1/2/3/4 = GPIO11/12/21/14", which matched the firmware but not the net names.

## Evidence

- [Netlist](../../docs/robot/hardware/netlist.md), `U4`: `22[IO14]=SERVO3`, `23[IO21]=SERVO4`; `H10`: `1=SERVO3, 2=SERVO4`; `CN3`: `3=SERVO3, 4=SERVO4`.
- Firmware: `kLegServoPin[4] = {11, 12, 21, 14}` in [`BoardConfig.h`](../../src/ES-02/OllieFOCdrive/BoardConfig.h).
- Source comparison only. No servo was moved and no wiring was inspected.

## Practical consequence

Firmware servo 3 is on `H10` pin 2 / `CN3` pin 4, and firmware servo 4 is on `H10` pin 1 / `CN3` pin 3. Whether this swap is intentional (it may compensate for the mechanical servo placement) is unknown. Do not "fix" either side without checking which leg each servo drives. Canonical reference: [GPIO table](../../docs/robot/hardware/electronics-reference.md#sensors-and-signals).
