# HMMD-Protokollreferenz

## Quelle und Geltung

Quelle ist das [Waveshare HMMD-Wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor), am 04.10.2026 heruntergeladen. Eine [lokale Kopie](vendor/HMMD-waveshare-wiki.html) und [durchsuchbarer Text](vendor/HMMD-waveshare-wiki.txt) liegen vor. Die folgenden Werte sind Herstellerangaben, keine Messungen am vorhandenen Modul. Die Herstellerseite nennt 115200 Baud, ein Stoppbit, keine Parität und Little-Endian-Daten. Diese Einstellungen und die Befehlsbestätigung sind am vorhandenen Modul noch nicht geprüft.

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
