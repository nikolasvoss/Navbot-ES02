# Elektronik-Hardware: technische Referenz

Diese Seite ergänzt [hardware-overview.md](hardware-overview.md); alle Bauteil-Pads und Netze stehen in [hardware-netlist.md](hardware-netlist.md). **Pins sind physische Steckerkontakte laut EasyEDA-PCB-Netzliste**, GPIOs sind ESP32-S3-Nummern laut Schaltplansymbol und Firmware. Die Kontaktansicht und Kabelrichtung sind damit noch nicht festgelegt; vor dem Einstecken die Platinenbeschriftung prüfen. `MAIN` und `CODER` sind die zwei Boards in `project.json` des [EasyEDA-Projekts](../hardware/pcb/ProPrj_NavBot-ES02.epro).

## Stromversorgung

**Bestückung laut Nutzer, 28.09.2026:** ESP32-S3-WROOM-1 ohne externen RAM; vorhandene Pinbelegung korrekt. Die N8R8-Angabe der Netzliste beschreibt nicht die bestätigte RAM-Ausstattung. Mit internem SRAM planen, PSRAM nicht aktivieren. Quellenabgrenzung und separate IMU-Interruptfrage: [wireless-logger-hardware.md](wireless-logger-hardware.md).

| Stufe | Bauteile / Netze | Funktion und praktische Folge |
| --- | --- | --- |
| Akku | Externer 2S-18650-Halter → `CN2`: 1 `B2`, 2 `B1`, 3 `B-` | Mittelabgriff `B1`; nicht mit einer einfachen zweipoligen Akkuleitung verwechseln. BOM: ein 2S-Halter und zwei Zellen. |
| Schutz und Ein/Aus | `U8` HY2120-LB, `Q2` WSD3056DN, `SW4`/`SW1`; `SW1` führt `B2` nach `VIN` | Zellschutz und geschaltete Hauptversorgung sind im Design vorhanden. `VIN` versorgt die Motortreiber direkt. Die tatsächliche Abschaltschwelle wurde hier nicht gemessen. |
| Laden / USB | `M1` „2S充电模块“ mit `VBUS`, `B2`, `B1`, `GND`, `VSTA`; USB-C `USB1`; `D5`/`J1` im Pfad zu `+5V` | Laden und USB-Versorgung sind im Design vorgesehen. Kein Ladeverhalten oder gleichzeitiger Betrieb unter Last wurde verifiziert. `VSTA` geht zum ESP32 (`GPIO18`). |
| 5 V | `U1` RT8289GSP, `L1` FXL0630-100-M, Eingangsnetz `VIN`, Ausgang `+5V` | Schaltregler für 5-V-Verbraucher/Anschlüsse. |
| 3,3 V | `U2` XC6210B332MR von `+5V` nach `+3.3V` | Versorgt ESP32, IMU, Encoder und Logik; nicht die Servolast. |
| Servo-Versorgung | `H11`: 1 `+5V`, 2 Servo-Versorgungsnetz, 3 `VIN`; dasselbe Netz an `H9/H10` Pins 3/4 | Dreipolige Auswahl: Brücke 1–2 = 5 V, 2–3 = VIN. Welche Brücke physisch steckt, ist nicht aus dem PCB-Projekt ableitbar. Vor 2S-VIN an Servos deren Zulässigkeit prüfen. |
| Batteriespannung | `R1` 10 kΩ von `VIN` nach `ADC`, `R2` 1 kΩ nach GND, `C1` 100 nF; `ADC` → ESP32 `GPIO17` | Spannungsteiler ca. 1:11. `ReadVoltage()` nutzt eine lokale Kalibrierung `7.77 / 813.43 × ADC`; dieser Schätzwert ist kein direkter 3,3-V- oder Zell-Einzelwert. |

**Prüfpunkt 5-V-Regler:** Im PCB-Netzlistenexport ist Pin 5 (`EN`) des RT8289 `U1` als unverbunden markiert. Die Netzliste allein klärt nicht, ob ein interner Default, eine abweichende reale Bestückung oder ein Designfehler vorliegt. Bei fehlenden 5 V zuerst diesen Punkt am realen Board und mit dem Bauteildatenblatt prüfen; aus der vorhandenen Board-Beobachtung folgt keine Pin-5-Messung.

## Motoren und Encoder

| Kanal | MAIN-Netze / GPIO | Peripherie |
| --- | --- | --- |
| A / `motor1` | `U6` MP6536; `PWM1_A/2_A/3_A` = GPIO15/7/6, `EN_A` = GPIO16; `H7` Pins 1/2/3 = `U_A/V_A/W_A` | Dreiphasiger Radmotor; Firmware `BLDCDriver3PWM(15, 7, 6, 16)`. |
| B / `motor2` | `U7` MP6536; `PWM1_B/2_B/3_B` = GPIO40/39/38, `EN_B` = GPIO37; `H8` Pins 1/2/3 = `U_B/V_B/W_B` | Zweiter dreiphasiger Radmotor; Firmware `BLDCDriver3PWM(40, 39, 38, 37)`. |
| Encoder A | `H5` Pins 1 SDA = GPIO4, 2 SCL = GPIO5, 3 GND, 4 +3,3 V; 5/6 GND laut PCB-Pads | `I2Cone.begin(4, 5, 400000)` und `sensor1` AS5600. Pins 5/6 gehören nicht zum vieradrigen Kabel. |
| Encoder B | `H6` Pins 1 SDA = GPIO41, 2 SCL = GPIO42, 3 GND, 4 +3,3 V; 5/6 GND laut PCB-Pads | `I2Ctwo.begin(41, 42, 400000)` und `sensor2` AS5600. Pins 5/6 gehören nicht zum vieradrigen Kabel. |

Die kleine **CODER**-Platine enthält `U1` AS5600-ASOM, lokale 3,3-V-Abblockung und 10-kΩ-Pull-ups für SDA/SCL. Deren `H1` hat dieselbe logische Reihenfolge: 1 SDA, 2 SCL, 3 GND, 4 3,3 V; weitere Footprint-Pads 5/6 liegen an GND und sind keine zusätzlichen Kabeladern. Der MAIN-Pull-up ist je I²C-Leitung 3,3 kΩ. Die BOM nennt zwei AS5600-Module, zwei 2208-90-kV-Motoren und zwei radiale 6 × 1,5-mm-Magnete. Magnetposition und elektrischer Nullwinkel müssen mechanisch/mit SimpleFOC kalibriert werden; sie folgen nicht aus der Netzliste.

## Sensoren und Signale

| Funktion | Chip / Signal | ESP32-S3 und Firmware |
| --- | --- | --- |
| IMU | `U5` ICM-42688P-HXY, SPI `SCLK/MOSI/MISO/CS1` | GPIO3/9/10/13; `ICM42688.cpp` verwendet SPI Mode 3, Beschleunigung ±8 g und Gyro ±2000 °/s bei 1 kHz. `INT1/INT2` sind im Design vorhanden; die gezeigte Firmware liest per SPI. |
| Vier Beinservos | `SERVO1/2/3/4` | GPIO11/12/21/14, 50-Hz-PWM in `ServoControl`; Servo 2/3 sind in der Standard-Ansteuerung invertiert. Die GPIO-Reihenfolge sagt noch nichts über links/rechts an der Mechanik. |
| Betriebs-LEDs | `LED2/LED3` | GPIO35/36; `BOARD_PIN_LED=35` und LOW schaltet die Firmware-LED ein. |
| Ladesignal | `VSTA` | GPIO18; im Schaltplan an `M1`, ohne hier bestätigte Softwareauswertung. |
| Versionskennung | `VERSION_ADC` | GPIO8, Teiler 180 kΩ/100 kΩ; im Design vorhanden, Nutzung der aktuellen Firmware nicht belegt. |

## Anschlüsse

| Stecker | Physische Pins laut PCB | Zweck / Hinweis |
| --- | --- | --- |
| `CN2` | 1 `B2`, 2 `B1`, 3 `B-` | 2S-Akku mit Mittelabgriff. |
| `H4` | 1 GND, 2 +5 V, 3 SBUS-Eingang über `R12/Q1` | Externer RC-Empfänger; siehe [SBUS-Anschlussblatt](../docs/ES02%20SBUS%20connect.pdf). |
| `H5`, `H6` | 1 SDA, 2 SCL, 3 GND, 4 +3,3 V | AS5600 A/B; zusätzliche GND-Pads 5/6 erscheinen in der Footprint-Netzliste. |
| `H7`, `H8` | 1 U, 2 V, 3 W | Dreiphasige Motoranschlüsse A/B. |
| `H9` | 1 Servo 1, 2 Servo 2, 3/4 gemeinsame Servo-Versorgung, 5/6 GND | Beinservos 1/2. |
| `H10` | 1 Servo 3, 2 Servo 4, 3/4 gemeinsame Servo-Versorgung, 5/6 GND | Beinservos 3/4. |
| `H11` | 1 +5 V, 2 Servo-Versorgung, 3 VIN | Auswahl der Servo-Versorgung; tatsächliche Brücke vor Betrieb ansehen. |
| `CN3` | 1–4 Servo 1–4, 5–7 GND | Signale/GND, keine Servo-Versorgung auf diesen Pins. |
| `H12` | 1 +5 V, 2 GND, 3 SCLK, 4 MOSI, 5 MISO, 6 CS1, 7 unbelegt, 8 GND | Zusätzlicher SPI-Header; seine +5 V nicht mit 3,3-V-Logikpegeln gleichsetzen. |

## Serielle Schnittstellen

| Pfad | Steck-/Netzseite | Firmware und Einschränkung |
| --- | --- | --- |
| USB/Debug | USB-C `USB1` → CH340X `U3` → `RX0/TX0`; `H1`: 1 +5 V, 2 GND, 3 RX0, 4 TX0 | Flash/`Serial`; am vorhandenen Board als CH340 `/dev/ttyUSB*` beobachtet. Normalmodus 2 Mbaud, Sensordiagnose 115200 Baud. [USB-Fakten](usb-serial.md) vor Änderungen lesen. |
| Touch / UART1 | `H2`: 1 +5 V, 2 GND, 3 RX1, 4 TX1; `CN4`: 2 +5 V, 3 GND, 4 RX1, 5 TX1 (1/7 GND) | **PCB-Netznamen:** RX1 an GPIO19, TX1 an GPIO20. **Firmware:** `RXD1=20`, `TXD1=19` – gegenläufig zu den Netznamen. Standard-Zweiradmodus liest die externe Touch-TTL-Platine mit 115200 Baud; im Vierbein-Master-Modus nutzt die Verbindung zum zweiten Controller denselben UART. Vor Verdrahtung physische Leitungsrichtung prüfen. |
| SBUS / UART2 | `H3`: 1 +5 V, 2 GND, 3 RX2, 4 TX2; `H4`: 1 GND, 2 +5 V, 3 SBUS über `Q1` | `RX2/TX2` = GPIO1/2. `FUTABA_SBUS` startet UART2 mit 100000 Baud; der Transistor invertiert den SBUS-Eingang für `RX2`. |

`H1/H2/H3` besitzen laut PCB-Footprint zusätzliche GND-Pads 5/6, aber vier Kabeladern. Bei externen Kabeln die Kontakte 1–4 und die Platinenbeschriftung verwenden. Die BOM nennt drei SH1.0-Kabel mit 200 mm (zwei Encoder, Verbindung zweier Einheiten) und eines mit 100 mm (Touch); für den SBUS-Empfänger ein dreipoliges Kabel.

## Externe Geräte, Modi und Datenqualität

- Die RC-Bedienung ist im [README](../README.md) beschrieben: CH5 Stop/Start/Touch, CH6 Haltung/„Mark“, CH7 Roll manuell/auto, CH8 Standard/Pitch/Ball-Balance, CH9/10 Ballkoordinaten. Diese Zuordnung ist Firmware-/Fernsteuerungs-Konfiguration, keine Eigenschaft des Steckers.
- `SENSOR_DIAGNOSTIC_MODE=1` setzt GPIO16/37 zunächst LOW, überspringt Motor-/Servo-Initialisierung und liefert IMU, Akku-ADC und RC-Zustand als CSV. Für das sichere Diagnoseverfahren und die beobachteten Spannungseinbrüche [diagnostic-mode.md](diagnostic-mode.md) lesen.
- Das README enthält ältere oder abweichende Hardware-Angaben (`MPU6050`, anderes Servo-Modell, unklare Touchgröße). Die **PCB-Netzliste** belegt die elektrischen Verbindungen; die **BOM** listet Kaufteile; die **Firmware** belegt gegenwärtig konfigurierte GPIOs und Betriebsarten. Bestückung, Kabelorientierung, Servospannungsbrücke und Spannungen unter Last müssen am realen Board geprüft werden.
