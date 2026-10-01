# Elektronik-Hardware: Einstieg für Agenten

**Zweck:** Diese Seite in etwa einer Minute lesen. Für Pinbelegung, Strompfade und Quellen nur den benötigten Abschnitt in [hardware-reference.md](hardware-reference.md) öffnen. Für einzelne Bauteile und unbenannte Netze gibt es zusätzlich die durchsuchbare [Bauteil-Netzliste](hardware-netlist.md). Die Angaben beziehen sich auf das eingecheckte EasyEDA-Projekt und die aktuelle Firmware, nicht auf eine Messung jeder Leiterbahn der aufgebauten Platine.

## System auf einen Blick

```text
2 × 18650 in Serie ── Schutz / Schalter ── VIN (2S)
                                         ├─ 2 × MP6536 → 2 × dreiphasiger 2208-Radmotor
                                         ├─ RT8289 → +5 V → XC6210 → +3,3 V
                                         └─ wählbare Servoversorgung (+5 V oder VIN)

ESP32-S3-WROOM-1 (MAIN)
  ├─ 2 getrennte I²C-Busse → je 1 AS5600-Platine (CODER), Radwinkel für SimpleFOC
  ├─ SPI → ICM-42688P, Beschleunigung und Drehrate für Lage/Balanceregelung
  ├─ 6 PWM + 2 Enable → 2 Motor-Endstufen; 4 PWM → Beinservos
  ├─ UART2 → invertierter SBUS-Empfänger; UART1 → Touch-TTL
  ├─ UART0 → CH340X → USB-C für Flash und Serial
  ├─ GPIO17/ADC → 2S-Akkuspannung über 10-kΩ/1-kΩ-Teiler
  └─ BLE → Steuerung ohne zusätzlichen Funkchip
```

Das EasyEDA-Projekt enthält **MAIN** (Steuerung, Versorgung, Motortreiber, Anschlüsse) und **CODER** (AS5600-Encoderplatine; zwei Exemplare laut BOM). Die Mechanik hat zwei bürstenlose Radmotoren und vier Beinservos. Die resistive Touchfläche wird laut BOM über eine separate TTL-Treiberplatine angebunden. RC-Steuerung erfolgt über einen externen Empfänger und dessen SBUS-Ausgang.

## Schnelle Orientierung

| Aufgabe | Relevante Hardware | Direkt weiterlesen |
| --- | --- | --- |
| Radmotor dreht nicht / FOC | MP6536 `U6/U7`, drei Motorphasen je Rad, AS5600 `H5/H6` | [Motoren und Encoder](hardware-reference.md#motoren-und-encoder) |
| Lage, Balance oder Fahrverhalten prüfen | ICM-42688P `U5` an SPI, Akku-ADC `GPIO17` | [Fallübersicht](problem-map.md), [Sensoren](hardware-reference.md#sensoren-und-signale) |
| Servo, Versorgungseinbruch | Vier Servoausgänge `H9/H10`, Versorgungsauswahl `H11`, `VIN` aus 2S-Akku | [Stromversorgung](hardware-reference.md#stromversorgung) und [Anschlüsse](hardware-reference.md#anschluesse) |
| Fernsteuerung | `H4`: GND, +5 V, SBUS; Inverter `Q1`; ESP32 `GPIO1` | [Anschlüsse](hardware-reference.md#anschluesse) |
| Touch oder UART | `H2`/`CN4`: UART1; `H3`: UART2; `H1`: UART0 | [Serielle Schnittstellen](hardware-reference.md#serielle-schnittstellen) |
| Einzelner Widerstand, IC-Pin oder unbenanntes Netz | Alle elektrisch verbundenen Pads beider PCBs | [Bauteil-Netzliste](hardware-netlist.md) |
| Flash / USB-Diagnose | USB-C → CH340X → UART0; nicht als natives USB-CDC voraussetzen | [usb-serial.md](usb-serial.md) |

## Wichtige Feststellungen und Konsequenzen

- **Kein externer RAM:** Laut Nutzerbestätigung vom 28.09.2026 ist ESP32-S3-WROOM-1 bestückt, ohne externen RAM; die Pinbelegung ist korrekt. Die N8R8-Angabe der Netzliste weicht von dieser Hardwareangabe ab. Nur internen SRAM für Logging einplanen; siehe [Hardwarebefunde für Logging](wireless-logger-hardware.md).

- **Servo-Stromschiene:** `H11` verbindet den gemeinsamen Versorgungs-Pin der Servostecker wahlweise mit `+5V` oder `VIN`. Vor Anschluss und Belastung die tatsächlich gesteckte Brücke und die Spannungsverträglichkeit der Servos prüfen. Die Servos hängen nicht am 3,3-V-Regler. Quelle: MAIN-PCB-Netzliste (`H9`, `H10`, `H11`).
- **Akkumessung:** Die Firmware errechnet aus dem GPIO17-ADC einen Akkuwert. Es ist **keine Messung der 3,3-V-Schiene**. Die dokumentierten Messwerte fallen unter Balance-Last stark; Akku, Kontakt, Regler und ADC-Verhalten sind damit noch nicht getrennt beurteilt. Quelle: MAIN-Netzliste `R1/R2`, `ReadVoltage()` und [diagnostic-mode.md](diagnostic-mode.md).
- **Encoder:** Die gleichen AS5600-I²C-Adressen sind möglich, weil die zwei Radencoder an getrennten ESP32-I²C-Controllern hängen: `GPIO4/5` und `GPIO41/42`. Quelle: MAIN-/CODER-Netzliste und Firmware-Initialisierung.
- **SBUS:** Der Empfänger wird über `H4` mit 5 V versorgt; das Signal läuft über den Transistor `Q1` invertiert zu `GPIO1`/UART2. Den Empfänger-Ausgang und die Pin-Reihenfolge vor dem Stecken prüfen. Quelle: MAIN-Netzliste und [ES02 SBUS connect.pdf](../docs/ES02%20SBUS%20connect.pdf).
- **UART1-Zuordnung:** Die PCB-Netze `RX1`/`TX1` liegen laut ESP32-Symbol an GPIO19/20, die Firmware verwendet für UART1 jedoch RX=GPIO20 und TX=GPIO19. Das ist eine echte Abweichung zwischen Netzbenennung und Softwarebelegung; die Kabelbelegung nicht allein aus den Namen ableiten. Quelle: `U4` in der [Bauteil-Netzliste](hardware-netlist.md) und `touchscreen.h`.
- **5-V-Regler:** `U1`-Pin 5 (`EN`) erscheint im PCB-Export als unverbunden. Daraus allein lässt sich sein Betriebsverhalten nicht bestimmen; bei 5-V-Problemen diesen Pin und das Datenblatt prüfen. Quelle: [Bauteil-Netzliste](hardware-netlist.md).
- **Datenlage:** README, BOM und Schaltplan nennen bei einzelnen Zukaufteilen unterschiedliche Modelle: Die BOM nennt Servo `GX-S7445`, das README `Pro-Tronik PTK 7452 MG-D`. Das README erwähnt außerdem an einer Stelle `MPU6050`, während Schaltplan, BOM und Firmware `ICM-42688P` belegen. Für Pin- und Schaltungsfragen die PCB-Netzliste, für die aktuell ausgeführte Funktion die Firmware heranziehen; konkrete bestückte Kaufteile am Roboter prüfen.

## Quellen und Geltung

- **Schaltplan/Netzliste:** [`hardware/pcb/ProPrj_NavBot-ES02.epro`](../hardware/pcb/ProPrj_NavBot-ES02.epro), darin `project.json`, MAIN-/CODER-`.esch` und `.epcb`. Die hier genannten Steckerpins und Netznamen wurden aus `PAD_NET` der jeweiligen `.epcb` gelesen.
- **Zukaufteile:** [`docs/BOM.xlsx`](../docs/BOM.xlsx), Tabelle `Sheet1`, Stand des Repositories.
- **Firmware-Zuordnung:** [`src/ES-02/OllieFOCdrive/OllieFOCdrive.ino`](../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino), [`ICM42688.h`](../src/ES-02/OllieFOCdrive/ICM42688.h), [`touchscreen.h`](../src/ES-02/OllieFOCdrive/touchscreen.h), [`FUTABA_SBUS.h`](../src/ES-02/OllieFOCdrive/FUTABA_SBUS.h).
- **Beobachtungen am vorhandenen Board:** [usb-serial.md](usb-serial.md) und [diagnostic-mode.md](diagnostic-mode.md). Diese beschreiben Messungen und lokale Tests; die Netzliste beschreibt das Design.
