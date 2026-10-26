# 2S charger module identified, 2026-10-24

Status: open

## Observation

The 2S charger `M1` on the robot is a separate module marked `NAVBOT-CHA_MOD 1.0`. Its chip is an IP2326 (Injoinic), a 2S/3S boost charger with built-in 2S balancing. The module's EasyEDA project in `cad/robot/` sets about 1.3 A charge current (`ISET` 68 kΩ), 8.4 V charge voltage, 4.45 V input undervoltage, 8.4 V input overvoltage, no timeout, and no NTC. The chip's status pin is not connected, and the balance reference is the module ground. The schematic and the PCB of that project agree.

## Evidence

- [Board photo](../../docs/robot/assets/charger-module-top.jpg), taken by the user on 2026-10-24. It shows the module pads, the `IP2326` marking, `R9` marked `1000` (100 Ω), two inductors marked `2R2`, and two diodes.
- [IP2326 datasheet V1.6, English](../../docs/robot/assets/ip2326-datasheet-v1.6-en.pdf), sections 4 to 9: pin functions, resistor-set limits, and the charge and balance rules.
- Module EasyEDA project `cad/robot/ProPrj_NavBot-Bat-ChargeModule-2S.epro` (not yet committed). The schematic and the PCB were compared by connectivity: all nets match, and the parts and designators match. The inductors are `FHD4012S-4R7MT` (4.7 µH) and `FHD4020S-330MT` (33 µH), and the diodes are `SS34F`. `Q1` is an N-channel `AO3400A` and `Q2` a P-channel `AO3401A`.
- [PCB netlist](../../docs/robot/hardware/netlist.md): `M1` pins 1 to 6, `U8` pin 4 (`VC`) on `B1`, `Q2` between `GND` and `B-`.

## Practical consequence

- The module is a boost charger. Its input is `VBUS` (5 V USB), so charging through `VBUS` is the intended path. A supply across the pack terminals bypasses the chip's control.
- `B1` is the middle-cell sense for balancing, and it is shared with the protection IC `U8`.
- At 1.3 A, the charge rate is high for many 18650 cells. Check the installed cells' datasheet.
- A 5 V input at full output draws about 2.4 A. A 2 A USB supply is too small.
- The module pads carry no `DP`/`DM`, so the chip stays at 5 V input.
- The balance reference `VBAT_GND` is tied to module `GND`, which is separate from `B-`. On the MAIN board the two nets connect only through its protection FET `Q2` (WSD3056DN). Whether they are at the same potential during charging is not verified.
- The module's `STA` output is not the chip's status pin. It is a divider from `BAT+` through `Q2` when USB is present: about 2 V at 8.4 V, which may read as low on `GPIO18`. This is an estimate that ignores the MOSFET resistance and needs a measurement.
- The photo's inductor markings `2R2` (2.2 µH) differ from the project's 4.7 µH and 33 µH parts, and the photo's diode markings do not match `SS34F` at this resolution. Which parts are fitted is not verified.

Details are in the [charger module page](../../docs/robot/hardware/charger-module.md). The status stays open until the inductors, diodes, and resistors are checked on the board, the balance reference is measured, and `GPIO18` is measured with USB connected and disconnected.

## Board readings, 2026-10-24 (user)

Read by the user from the board (marking or value, as reported):

- `R2` 511 (510 Ω), `R3` 1 kΩ, `R10` 1 kΩ, `R12` 10 kΩ (`01C`), `R14` 10 kΩ (`01C`): match the project.
- `R6` 100 kΩ: differs from the project's 68 kΩ. As fitted, `ISET` gives `90000 / 100000` = 0.9 A, not 1.32 A.
- `R7` 1 Ω: matches the project's LCSC part. The device name in the project (`0603WAF100KT5E`) suggests 100 kΩ, but the part is 1 Ω.

Battery: 2 × 18650, cell label 2000 mAh, 7.4 Wh, 4.2 V, designation GB 31241-2022 (user). The label is internally consistent (2000 mAh × 3.7 V = 7.4 Wh). The label gives no maximum charge current. Charge rates: 0.9 A = 0.45 C, 1.32 A = 0.66 C, 1.5 A = 0.75 C.

Planned change, not made: `R6` to 68 kΩ (about 1.32 A), the design value, to raise the charge current from 0.9 A. The board still has 100 kΩ. A 60 kΩ value (1.5 A, datasheet maximum) is a later option, only after the cells' maximum charge current is confirmed.

Status stays open until `R6` is changed and measured, the inductors are checked, and `GPIO18` is measured.
