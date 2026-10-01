# Hardwarebefunde für kabelloses Logging

Stand 28.09.2026; Dokumentenvergleich und anschließende Hardwarekorrektur durch den Nutzer, keine eigene Messung.

## PSRAM und GPIO35–37

Beobachtung: [PCB-Netzliste](hardware-netlist.md), U4, nennt ESP32-S3-WROOM-1-N8R8 und verbindet zugleich IO35/36 mit LED2/3 sowie IO37 mit EN_B. Die Firmware verwendet diese GPIOs ebenfalls. Laut [Espressif-Datenblatt, Abschnitt 3, Fußnote b](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) stehen IO35–37 bei Octal-PSRAM-Modulen nicht für andere Zwecke zur Verfügung.

**Korrektur durch den Nutzer am 28.09.2026:** Bestückt ist ESP32-S3-WROOM-1, es gibt keinen externen RAM, und die Pinbelegung ist korrekt. Dies ist die maßgebliche Angabe zum aufgebauten Roboter. Die N8R8-Bezeichnung der exportierten Netzliste bleibt als abweichende Designangabe erhalten; daraus keinen realen PSRAM-Pinkonflikt ableiten.

Konsequenz: Mit internem SRAM planen, PSRAM deaktiviert lassen und keine Pinänderung vorsehen. Die tatsächlich freie interne Pufferkapazität unter Firmware-/WLAN-Last muss noch gemessen werden. Die Flashgröße ist aus der Basisbezeichnung allein nicht bestimmt.

## IMU-Interrupts

Beobachtung: Die exportierte U5-Belegung enthält INT1/INT2, U4 aber keine entsprechenden Netze; R28 zieht INT2 mit 10 kΩ nach GND. Der vorhandene IMU-Treiber liest Datenregister über SPI und nutzt keinen FIFO-/Interruptpfad.

Konsequenz: Kein vorhandener ESP-Interruptanschluss annehmen. Für FIFO-Erfassung entweder begrenztes Polling entwickeln oder nach Prüfung von Layout und realer Platine einen geeigneten Anschluss planen.

Das [Konzept](../dev_reference/kabelloses-tuning-messlogger-konzept.md) beschreibt Speicherbudget und Entscheidungspunkte.

## WLAN-Aufnahmen am aufgebauten Roboter am 30.09.2026

Beobachtung: Der CH340 erschien vor dem Flash als `/dev/ttyUSB1` und danach als `/dev/ttyUSB0`. `esptool chip-id` erkannte einen ESP32-S3 Revision v0.2. `scripts/upload_firmware.py` verifizierte die Hashes aller geschriebenen Images. Nach dem Start meldete Serial `Wi-Fi connected.` und `Wi-Fi tuning API started.`. Die API meldete einen internen Ringpuffer mit 512 Datensätzen.

Zwei gesicherte Standaufnahmen über je 20 Sekunden lieferten 4.905 und 4.931 Datensätze. Der Decoder bestätigte für beide `DURATION`, vollständige Datensätze und null CRC-, Sequenz- und Zahlenfehler. Die höchsten Pufferstände waren 9 und 6 von 512. Die beobachtete Abtastung lag bei etwa 245 Datensätzen pro Sekunde; das 99. Perzentil des Abstands zwischen Datensätzen lag bei 4,925 und 4,928 ms. Ein nach etwa 5 Sekunden unterbrochener Lauf lieferte 1.218 gültige Datensätze mit `USER_STOP`; die API meldete anschließend `COMPLETE` und einen leeren Puffer. `release` setzte die Sitzung danach auf `IDLE` mit leerem Puffer. Dateien und Berichte liegen unter `/tmp/navbot-wifi-live-20260930*.nblog` und `/tmp/navbot-wifi-live-20260930*.json`.

Konsequenz: Der TCP-Transfer, die Dateiprüfung, der ACK, die Wiederverwendung des Puffers und der Abbruch funktionieren in diesen drei Standläufen. Die Messung enthält weder einen Vergleich mit einem Build ohne WLAN-Logging noch Fahrbewegungen oder einen absichtlich blockierten Sender. Sie belegt daher keine Regelkreisreserve unter repräsentativer Last. Die aufgezeichneten Motorziele erreichten auch am gesicherten Roboter etwa ±82; einen weiteren Bewegungstest nur mit gesicherter Mechanik durchführen.

Abweichung: Das Flash-Werkzeug meldete `Embedded PSRAM 8MB`, während die Nutzerangabe oben keinen externen RAM nennt. Dieser Lauf prüfte die Bestückung nicht. Die Firmware fordert den Aufnahmepuffer weiterhin ausdrücklich mit `MALLOC_CAP_INTERNAL` an; aus der Werkzeugmeldung keine verfügbare PSRAM-Kapazität für den Logger ableiten.

### Regelzykluszeit während der Aufnahme am 30.09.2026

Beobachtung: Bei CH5 aus zeigte die WLAN-Aufnahme mit dem ursprünglichen Erfassungspfad `control_dt` median 3,590 ms, Mittel 3,757 ms und p95 4,394 ms. Ein K58-Lauf ohne WLAN-Aufnahme hatte zuvor median etwa 2,104 ms. Die Aufnahme setzte keinen Gain-Modus um; die aufgezeichneten Zustandsflags waren durchgehend 48 (CH5 aus, gültige RC- und IMU-Daten, Regler nicht berechnet). Im Firmwarepfad wurde trotzdem bei niedriger Batteriespannung für jeden frischen SBUS-Frame die Spannung als Fließkomma-Text über Serial ausgegeben, obwohl `print_data()` während der Aufnahme bereits unterdrückt war.

Korrektur und Evidenz: Die serielle Spannungsausgabe ist während einer aktiven Aufnahme jetzt unterdrückt, weil die Spannung bereits im Datensatz enthalten ist. Der Median sank damit auf 3,388 ms. Die Sample-Erzeugung ist auf 100 Hz begrenzt. Eine erste Batch-Benachrichtigung allein ergab Ring high-water 2 und keine Verbesserung, weil der Sender weiterhin spätestens alle 20 ms aufwachte. Die getestete Korrektur wartet im Sender bis zu 60 ms und benachrichtigt ihn regulär erst nach fünf Samples. Der bestätigte 20-s-Lauf `/tmp/navbot-verify/20260930-batch60/ch5-off.nblog` enthielt 1.846 Datensätze; Ring high-water war 5, `control_dt` lag bei median 2,032 ms, Mittel 2,214 ms und p95 2,813 ms. Decoder: null CRC-, Sequenz- und Zahlenfehler. Die IMU-ODR-Metadaten lauten nach Korrektur 1.000 Hz; zuvor wurde versehentlich die Funktionsadresse von `ImuRATE_HZ` serialisiert.

Zusätzliche Beobachtung: Im 20-s-Lauf mit CH5=1 auf dem 100-Hz-Build vor Sender-Bündelung waren alle 1.778 Datensätze aktiv (Flags 57), ohne Tumble- oder Zahlenfehler. `control_dt` lag bei median 2,481 ms und p95 3,221 ms. Die Regelung erreichte Rollwerte von etwa -4,94 bis +5,04 Grad und Motorziele um ±39,7. Der Nutzer meldete anschließend, dass die Messung das Verhalten weiterhin beeinflusse, und schaltete CH5 aus.

Nachweis mit Sender-Bündelung: Am 30.09.2026 lief auf demselben Build und derselben Boot-ID ein weiterer gesicherter 20-s-Lauf mit CH5=1. Er enthielt 1.768 Datensätze, durchgehend Flags 57, Ring high-water 5 und null CRC-, Sequenz-, Zahlen- oder Überlauffehler. `control_dt` lag bei median 2,128 ms, Mittel 2,316 ms, p95 2,948 ms und Maximum 4,236 ms. Die Rollwerte lagen zwischen -5,13 und +4,36 Grad. Der Vergleichslauf mit CH5=0 auf demselben Build und derselben Boot-ID enthielt 1.846 Datensätze; `control_dt` lag bei median 2,032 ms und p95 2,813 ms. Der CH5-Zustand ändert zugleich, ob der Regler berechnet wird. Der Unterschied isoliert daher nicht den Einfluss des Loggings. Rohdaten und Auswertung liegen unter `/tmp/navbot-verify/20260930-ch5-1-batch60/`; der CH5=0-Vergleich liegt unter `/tmp/navbot-verify/20260930-batch60/`. Nach dem Lauf bestätigte der WLAN-Status `ch5_off=true` und gültige RC-Daten.

Gepaarter Timingvergleich: Später am selben Tag lief ein CH5=1-K58-Baselinefenster mit inaktivem WLAN-Recorder und 20-Hz-Serialausgabe. Es enthielt 393 gültige Zeilen über 19,981 s. `control_dt` lag bei median 2,122 ms, Mittel 2,246 ms, p95 2,762 ms und Maximum 3,223 ms. Rund 4,3 Minuten danach startete auf derselben Boot-ID `eac66ebaef436e41` die aktive WLAN-Aufnahme. Sie enthielt 1.777 gültige Datensätze über 20 s, durchgehend Flags 57, Ring high-water 5 und null CRC-, Sequenz-, Zahlen- oder Überlauffehler. `control_dt` lag bei median 2,117 ms, Mittel 2,296 ms, p95 2,892 ms und Maximum 4,081 ms. Der Median sank um 5 µs, das p95 stieg um 130 µs und das Maximum um 858 µs. Die Messraten unterschieden sich mit 20 Hz und etwa 89 Hz. Die Läufe waren nicht unmittelbar aufeinanderfolgend. Der Vergleich erlaubt daher keine sichere Aussage über kleine Lastunterschiede. Er prüfte außerdem keine Bewegung und nicht die subjektiv wahrgenommene Balance. Dateien und Kennzahlen liegen unter `/tmp/navbot-verify/20260930-ch5-comparison/`.

Konsequenz: Mit CH5 aus bringt Sender-Bündelung die Regelzykluszeit nahe an die frühere Ohne-Aufnahme-Baseline. Der gleichartige CH5=1-Vergleich zeigt einen fast unveränderten Median, aber höhere p95- und Maximalwerte während WLAN-Aufzeichnung. Wegen verschiedener Messraten und des zeitlichen Abstands ist das nur ein Hinweis, kein Beweis für oder gegen eine wahrnehmbare Änderung. Aktives Balancieren unter Bewegung bleibt ungeprüft. Weitere CH5=1-Tests nur auf mechanischer Sicherung und mit gleichem Gain-Zustand durchführen. Die tatsächlichen Sample-Zeitstempel bleiben maßgeblich, weil die Aufnahmerate zeitlich nicht exakt gleichförmig ist.
