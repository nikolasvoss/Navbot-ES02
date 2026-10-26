# 2S charger module (M1)

**Purpose:** Identify the on-board 2S lithium charger (`M1`), its IC, its pads, and the settings that the module's resistors fix. This page describes the module design and the photo of the built module. Charging behavior on the assembled robot is not measured here; see the [observation note](../../../lab/robot/2026-10-24-charger-module-identified.md).

## Identification

| Item | Value | Source |
| --- | --- | --- |
| Board marking | `NAVBOT-CHA_MOD 1.0`, a separate module PCB soldered to the robot | [photo](../assets/charger-module-top.jpg) |
| Module design | EasyEDA project `ProPrj_NavBot-Bat-ChargeModule-2S.epro`, board `RECHARGE`, project `NavBot-ChargeModule-2S`, version V1.0 | `cad/robot/` (not yet committed) |
| Netlist designator | `M1`, value `2S_RECHARGE`: 1 `VBUS`, 2 `GND`, 3 `VSTA`, 4 `GND`, 5 `B1`, 6 `B2` | [netlist](netlist.md) (MAIN) |
| Charge IC | **IP2326** (Injoinic), 2S/3S synchronous boost charger, QFN-24 4 × 4 mm, built-in 2S balancing | [datasheet V1.6, English](../assets/ip2326-datasheet-v1.6-en.pdf) |

The module is a boost charger. It takes the 5 V USB input on `VBUS` and charges the pack from `B2` to the pack negative `B-`, which is external to the module. The pack is not charged from the USB voltage directly.

## Schematic and PCB check

Checked 2026-10-24 on the module project. The schematic and the PCB agree:

- All 29 parts (`C1` to `C8`, `D1`, `D2`, `L1`, `L2`, `LED1`, `Q1`, `Q2`, `R1` to `R14`, `U1`) appear in both, with the same designators and matching pad counts.
- Every PCB net matches a schematic connection, and every schematic connection appears in the PCB. No net is split or merged.
- Five chip pins are unconnected in both: `DM`, `DP`, `VSET`, `BAT_STAT`, `CON_SEL`.

The check compares connectivity and pad counts. It does not compare footprint sizes, because the PCB file does not store them.

## Pads and the IP2326 pins

The mapping comes from the module's PCB pad-net list. The chip pin numbers are the datasheet numbers. Pads 14 to 25 agree with the datasheet's functions (`BST`, `LX`, `PGND`, `VSYS`, `VOUT`, `VBATM`, `VBAT_GND`, `GND`), which supports this numbering.

| Module pad | Net | IP2326 pin | Function and note |
| --- | --- | --- | --- |
| `VBUS` (1) | `VBUS` | `VIN` (13) | 5 V USB input. |
| `GND` (2, 4) | `GND` | `PGND` (18), `GND` (EPAD) | Module ground, which is robot system ground. |
| `STA` (3) | `VSTA` | not `BAT_STAT` (5) | Not the chip's status output. Pin 5 is not connected. `VSTA` is a divider from `BAT+` through `Q2` when USB is present, and it goes to ESP32 `GPIO18`. See consequence 6 and open point 3. |
| `B1` (5) | `BM` | `VBATM` (23) | Middle-cell sense for balancing, through `R9` (100 Ω). On the MAIN board this node also feeds the protection IC `U8` pin 4 `VC` through `R36` and `C35`. |
| `B2` (6) | `BAT+` | `VOUT` (21, 22) | Battery positive and charger output. |
| – | – | `VBAT_GND` (24) | Tied to module `GND`, not to pack `B-`. See open point 2. |
| – | – | `DM` (1), `DP` (2) | Not connected. The module cannot negotiate a higher input voltage. |
| – | – | `VSET` (3), `CON_SEL` (10) | Not connected, so the chip uses its defaults: 8.4 V and 2S. |
| – | – | `EN` (12) | Tied to `VIN`, so the chip runs whenever the input is present. |

## Resistor settings

Resistor values are decoded from the Walsin part codes in the module project. Decoding follows the code: three digits × 10^(last digit) ohms.

| Ref | Value | Pin or net | Effect (datasheet rule) |
| --- | --- | --- | --- |
| `R6` | 68 kΩ in the design; **100 kΩ read on the board** (user, 2026-10-24) | `ISET` (11) | Constant charge current `90000 / 68000` ≈ **1.32 A** in the design. With 100 kΩ as read, `90000 / 100000` = **0.9 A**. Verify the fitted value. |
| `R4` | 120 kΩ | `VIN_UVSET` (8) | Input undervoltage threshold 4.45 V (falling). |
| `R5` | 120 kΩ | `VIN_OVSET` (9) | Input overvoltage threshold 8.4 V for 2S. |
| `R3` | 1 kΩ | `TIME_SET` (7) | Charge timeout disabled. |
| `R1` | 51 kΩ | `NTC` (4) | No NTC used. The datasheet asks for 51 kΩ to GND in this case. |
| – | none | `VSET` (3) | Charge voltage 8.4 V per 2S default. |
| – | none | `CON_SEL` (10) | 2S charging. |
| `R9` | 100 Ω, 1206 | `VBATM` (23) to `BM` | Balance resistor `RCB`. The datasheet's balance current is `V_CB / RCB` and must stay below 40 mA, so 100 Ω is at its lower limit. |
| `R2` | 510 Ω | `LED` (6) | Indicator LED current limit. Board marking `511` (user, 2026-10-24) matches. |

Other resistors, which the datasheet reference does not show: `R7` 1 Ω from `VIN` to the boost node after `D1` (the device name `0603WAF100KT5E` is misleading; its LCSC part is 1 Ω), `R8` 5.1 kΩ `VIN` to GND, `R10` 1 kΩ `VSTA` to GND, `R12` 10 kΩ `VBUS` to GND, `R13` 3 kΩ `VSTA` to a MOSFET node, `R14` 10 kΩ from that node to `BAT+`. Their function is not established.

Board check (user, 2026-10-24): `R3` 1 kΩ, `R10` 1 kΩ, `R12` 10 kΩ (`01C`), and `R14` 10 kΩ (`01C`) match the design. `R7` reads 1 Ω, which matches the project's LCSC part.

## Power parts

| Ref | Part | Connection | Note |
| --- | --- | --- | --- |
| `L1` | FHD4012S-4R7MT (4.7 µH) | `LX` to the `D1` node | The datasheet application uses 2.2 µH. The photo shows `2R2` markings. See open point 4. |
| `L2` | FHD4020S-330MT (33 µH) | `VOUT` to `BAT+` | Not in the datasheet application. Its function is not established. |
| `D1` | SS34F Schottky | `VBUS` to the boost node | The datasheet calls this optional, needed for power applications. |
| `D2` | SS34F Schottky | `VOUT` to `BAT+` | Not in the datasheet application. |
| `Q1` | AO3400A (N-MOS) | gate `VBUS`, source `GND`, drain node `$1N3338` | Turns on when USB is present and pulls `$1N3338` low. |
| `Q2` | AO3401A (P-MOS) | gate node `$1N3338`, source `BAT+`, drain node `$1N3747` (via `R13` to `VSTA`) | Turns on when `Q1` is on and passes `BAT+` to `VSTA`. `R14` (10 kΩ) pulls the gate up to `BAT+`, so `Q2` is off without USB. |
| `C2`, `C5`, `C6` | 10 µF, 0805 | `VIN`, `VOUT` to GND | Per datasheet. |
| `C3`, `C4` | 22 µF, 0805 | `VSYS` to GND | Per datasheet. |
| `C1` | 100 nF | `BST` to `LX` | Bootstrap. |
| `C8` | 100 nF | `VBUS` to GND | Input decoupling. |

## Charging sequence (2S with this module's settings)

- Below 3.7 V: 50 mA trickle charge. From 3.7 V to 6 V: 100 mA. Above 6 V: constant current at about 1.32 A.
- Constant voltage at 8.4 V. End of charge when the current falls below 200 to 300 mA. The chip then waits 30 s and checks the pack voltage. Charging restarts when the pack falls below 8 V while the input is present.
- The input stays at 5 V. Fast-charge requests need `DP`/`DM`, which are not connected.
- Balancing starts when either cell reaches 4.1 V. It runs during charging only.
- Protections: input UVLO at 4.45 V with an input regulation loop that reduces the charge current near that threshold, input OVP at 8.4 V, output overcurrent and overvoltage, and thermal shutdown at 135 °C (recovers at 110 °C).
- Status LED: on while charging, off when full, 0.83 Hz blink on a fault.

## Practical consequences

1. **Charge current depends on the fitted `R6`.** The design gives about 1.3 A (68 kΩ). The board reads 100 kΩ, which gives 0.9 A. The planned value is 68 kΩ, which is the design value. The cells are 2 × 18650, labeled 2000 mAh, 7.4 Wh, 4.2 V, and designated GB 31241-2022 (user, 2026-10-24). The label is consistent: 2000 mAh at 3.7 V nominal is 7.4 Wh, and 2 cells in series give 8.4 V full, matching the 8.4 V charge voltage. The label does not state a maximum charge current, and the cell datasheet is not in the repository. GB 31241-2022 is a safety standard and does not state capacity or charge current. C-rates: 0.9 A = 0.45 C, 1.32 A = 0.66 C, 1.5 A = 0.75 C. The planned 68 kΩ value (1.32 A) is above 0.5 C, so confirm the maximum charge current before fitting it. [BOM](../assets/bom.xlsx) row 30 lists "18650 Battery", quantity 2, linked to an Amazon product from MC-ELIN, without capacity or charge rating.
2. **Input current at full output.** Estimate: 8.4 V × I ÷ 0.94 ÷ 5 V. At 1.32 A this is about 2.4 A, at 0.9 A about 1.6 A. A 2 A USB supply is too small for the design current. Use a supply rated for at least 3 A at 5 V.
3. **`B1` is part of the balance sense.** An open `B1` removes the middle-cell reading from the chip and from the protection IC `U8`.
4. **Balancing is current-limited.** At 100 Ω it is at the datasheet's limit of 40 mA.
5. **Charge through `VBUS`, not across the pack terminals.** A supply on `B2`/`B-` bypasses the chip's current and voltage control and its balancing. Use the module input (`VBUS`/`GND`) instead.
6. **`VSTA` is not a charge-status signal.** With USB and a battery present, `Q2` connects `BAT+` through `R13` (3 kΩ) and `R10` (1 kΩ) to GND, so `VSTA` sits at about one quarter of the battery voltage, roughly 2 V at 8.4 V. Without USB, `VSTA` is at 0 V. This is an estimate that ignores the MOSFET resistance. The ESP32-S3 input high level is about 2.5 V at 3.3 V (from memory of the Espressif datasheet, not checked here), so `GPIO18` may read this as low. Measure `GPIO18` before relying on it.
7. **USB host limits.** The charger draws from `VBUS` directly, and the USB-C CC pins carry 4.7 kΩ pull-downs (`R5`, `R6` in the MAIN netlist), so a host sees a sink. A host port supplies only its advertised current: about 500 mA for USB 2.0, 900 mA for USB 3.x, and up to 1.5 A or 3 A on dedicated or USB-C sources. At 1.5 A charge current the input draws about 2.7 A from 5 V. The chip's input regulation then lowers the charge current to hold `VIN` at the UVLO threshold, 4.45 V, so the port sits at its limit and the charge is slow. Overload may trip the host's overcurrent protection and disable the port. Modern hosts usually protect themselves, but this is not guaranteed and was not tested. Use a dedicated 5 V supply rated for at least 3 A.

## Open points

1. **Confirm the built module against the project.** The photo shows the resistor `1000` marking on `R9` (100 Ω), which matches. Inductors, diodes, and the other resistors are not yet read from the board.
2. **Balance reference.** `VBAT_GND` (pin 24) is tied to module `GND`, which is robot system ground. The pack negative `B-` is outside the module. The netlist connects `B-` to `GND` only through the protection FET `Q2`. Verify that `GND` and `B-` are at the same potential during charging. Otherwise the balance and the cell sense read against the wrong reference.
3. **Status pin.** Pin 5 `BAT_STAT` is not connected, so the chip's charge state is not available on the module. `VSTA` shows USB-plus-battery presence as a divider (consequence 6). Measure `GPIO18` with USB connected and disconnected, and confirm the level. The electronics reference lists the same signal as a charge input, so that entry needs updating too.
4. **Inductor value.** The project lists `L1` as 4.7 µH and `L2` as 33 µH. The photo shows `2R2` markings, and the datasheet says 2.2 µH. Verify which parts are fitted. A different inductance changes the switching ripple and the efficiency.
5. **Module revision.** The project title block says 2025-10-13 and the file name says 2026-01-14. Confirm that the built module matches project version V1.0.

## Sources

- [IP2326 datasheet V1.6, English](../assets/ip2326-datasheet-v1.6-en.pdf) (Injoinic, © 2020).
- [Board photo](../assets/charger-module-top.jpg), 2026-10-24.
- Module EasyEDA project `cad/robot/ProPrj_NavBot-Bat-ChargeModule-2S.epro`: schematic `SHEET/…/2.esch`, PCB pad-net list.
- [PCB netlist](netlist.md): `M1`, `CN2`, `R35`, `R36`, `C34`, `C35`, `U8`, `Q2`, `D5`, `J1`.
