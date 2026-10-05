# HMMD-Protokollreferenz

## Quelle und Geltung

Quelle ist das [Waveshare HMMD-Wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor), am 04.10.2026 heruntergeladen. Eine [lokale Kopie](vendor/HMMD-waveshare-wiki.html) und [durchsuchbarer Text](vendor/HMMD-waveshare-wiki.txt) liegen vor. Die Herstellerwerte sind von Messungen am vorhandenen Modul getrennt. Die Herstellerseite nennt 115200 Baud, ein Stoppbit, keine Parität und Little-Endian-Daten. Die Parameterzugriffe unten wurden zusätzlich am vorhandenen Modul mit Firmware v1.6.1 geprüft.

## Debug-Datenframe

| Feld | Herstellerangabe |
| --- | --- |
| Header | `AA BF 10 14` |
| Matrix | 20 Doppler-Bins × 16 Entfernungszellen |
| Wertbreite | 4 Bytes, Quadrat der Amplitude |
| Nutzdaten | 1.280 Bytes |
| Footer | `FD FC FB FA` |
| Abgeleitete Gesamtlänge | 1.288 Bytes einschließlich Header und Footer |

Die Herstellerbeschreibung nennt die Sendereihenfolge anhand von Chirps und jeweils 16 Entfernungszellen. Die erste Darstellung behält die vom Hersteller beschriebene Sensorreihenfolge bei. Die Implementierung verwendet Little-Endian-`uint32` gemäß Herstellerprotokoll. Datentyp, Indexformel und physikalische Bedeutung der Doppler-Bins bleiben am Mitschnitt des vorhandenen Moduls zu bestätigen.

## Befehl für Debug-Modus

Das Wiki zeigt diese Bytefolge für den Debug-Modus.

```text
FD FC FB FA 08 00 12 00 00 00 00 00 00 00 04 03 02 01
```

Die vollständige Konfigurationssequenz, erforderliche vorherige Befehle und ACK-Auswertung sind hier noch nicht bestätigt. Die Beispielbytes allein belegen keine betriebsfähige Initialisierung.

## Versorgung und UART

Das Raspberry-Pi-Beispiel nennt `3V3`, `GND`, Sensor-TX an Host-RX und Sensor-RX an Host-TX. Diese Angabe ersetzt keine Prüfung des konkreten Moduls und des CM5-Trägerboards. Steckerorientierung, Host-Pins und Portname sind nicht dokumentiert.

## Grenzen der Aussage

Es liegt keine belegte Sensorzeitbasis vor. Ein Empfangszeitstempel beschreibt den Empfang beim Host. Aus Header und Footer allein lässt sich nicht jede Verfälschung der Nutzdaten erkennen. Eine Prüfsumme ist in der hier belegten Debug-Framebeschreibung nicht angegeben. Bildrate, Entfernungsskalierung, Geschwindigkeitsskalierung und Vorzeichen sind noch offen.

## HMMD-Konfiguration, am Modul geprüft

Das Waveshare-Wiki beschreibt Parameter-ID 1 als Maximum Distance Gate mit Bereich 0–15 und die Target Disappearance Delay Time mit Bereich 0–65535 Sekunden. Am Modul war der aktuelle Delay-Wert unter Parameter-ID 4 lesbar. Die geprüften Antworten setzen das obere Byte des Antwortcodes auf `01` und liefern danach den Erfolgsstatus `00 00`. Parameter-Leseantworten enthalten den Antwortcode, den Status und den 32-Bit-Wert; sie enthalten keine Parameter-ID. Der Treiber ordnet deshalb nur eine Antwort dem jeweils einzigen laufenden Lesevorgang zu.

Config-Modus öffnen:

```text
Anfrage: FD FC FB FA 02 00 FF 00 04 03 02 01
ACK:     FD FC FB FA 08 00 FF 01 00 00 02 00 20 00 04 03 02 01
```

Maximum Gate lesen (Parameter-ID 1), Antwortwert 12:

```text
Anfrage: FD FC FB FA 04 00 08 00 01 00 04 03 02 01
Antwort: FD FC FB FA 08 00 08 01 00 00 0C 00 00 00 04 03 02 01
```

Target Disappearance Delay lesen (Parameter-ID 4), Antwortwert 30 Sekunden:

```text
Anfrage: FD FC FB FA 04 00 08 00 04 00 04 03 02 01
Antwort: FD FC FB FA 08 00 08 01 00 00 1E 00 00 00 04 03 02 01
```

Die folgenden unveränderten Schreibvorgänge wurden am Modul geprüft. Sie belegen die Protokollsequenz und ACKs für die aktuellen Werte, nicht das Verhalten beim Ändern auf andere Werte.

```text
Maximum Gate 12 schreiben:
FD FC FB FA 08 00 07 00 01 00 0C 00 00 00 04 03 02 01
Delay 30 Sekunden schreiben:
FD FC FB FA 08 00 07 00 04 00 1E 00 00 00 04 03 02 01
Write ACK für beide:
FD FC FB FA 04 00 07 01 00 00 04 03 02 01
```

Config-Modus mit Save/Exit schließen:

```text
Anfrage: FD FC FB FA 02 00 FE 00 04 03 02 01
ACK:     FD FC FB FA 04 00 FE 01 00 00 04 03 02 01
```

Beim vollständigen Lesevorgang folgten auf das Öffnen die Lesevorgänge für ID 1 und ID 4 sowie ein bestätigter Save/Exit. Beim Schreibvorgang folgten auf die Öffnung ein einzelner Write, ein Readback derselben ID mit identischem Wert und ein bestätigter Save/Exit. Ein Save ACK bestätigt die Befehlsannahme. Eine Prüfung nach Aus- und Einschalten wurde nicht durchgeführt; Persistenz über einen Neustart ist daher nicht belegt.
