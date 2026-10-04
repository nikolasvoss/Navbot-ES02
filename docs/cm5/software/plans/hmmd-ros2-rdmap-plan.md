# HMMD-Radar in ROS 2 anzeigen

Setze [Roadmap-Punkt 1](../roadmap.md) als stationären Sensortest am CM5 um.
Arbeite im Branch `codex/hmmd-ros2-rdmap`, ausgehend von `main` bei `8c160d9`.
Dieser Plan ist die Vorbereitung. In diesem Schritt entsteht keine Implementierung.
Beginne die Umsetzung erst nach dem Auftrag zur Ausführung.

## Schritt für Schritt vorgehen

1. **Prüfe den vorhandenen Aufbau.** Identifiziere das konkrete HMMD-Modul, das CM5-Trägerboard, das Betriebssystem und die Version der bereits installierten ROS-2-Distribution. Setze ROS 2 auf dem CM5 als bereits installiert voraus. Prüfe die Versorgung und den Signalpegel anhand der Modulunterlagen und des tatsächlichen Anschlusses. Wähle eine separate UART-Verbindung oder einen passenden USB-UART-Adapter. Prüfe bei UART die Pinbelegung des Trägerboards und die Belegung durch eine serielle Konsole. Halte Anschlussdaten und Softwareversionen in `agent_notes/cm5/sensors/hmmd/observations.md` fest. Belege die Prüfung mit Modulbezeichnung, Pinbelegung und Messung vor dem Einschalten. Verwende keine UART der Fahrsteuerung.

2. **Sichere echte Rohdaten.** Prüfe Baudrate, Befehlsfolge, Bestätigung und Debug-Modus anhand der Waveshare-Unterlagen und des Sensors. Führe direkten seriellen Zugriff über den konfigurierten Serial-MCP mit `list_ports`, `open`, RTS/DTR niedrig, `read`, erforderlichem `write` und `close` aus. Lies vorher `docs/robot/software/development/usb-serial-flashing.md` und den Skill `serial-debug`. Falls der Sensor nur am CM5 erreichbar ist, kläre zuerst einen dort erreichbaren Serial-MCP. Speichere einen kurzen binären Mitschnitt mit Empfangszeiten. Belege Header, Payloadlänge, Footer, Byte-Reihenfolge und Verhalten nach einem Neustart. Protokolliere nicht bestätigte Annahmen ausdrücklich.

3. **Bestätige die Matrixdarstellung.** Prüfe am Mitschnitt die 320 Werte und die Reihenfolge der 20 Doppler-Bins mit je 16 Entfernungszellen. Vergleiche einen leeren Messbereich, eine ruhende Person und eine bewegte Person. Trenne die nachgewiesene Übertragungsreihenfolge von der physikalischen Interpretation. Verwende zunächst ausschließlich Bin-Indizes. Bestimme die beobachtete Frame-Rate und die Pausen zwischen Frames. Bleibt die physikalische Anordnung unklar, dokumentiere diese Grenze und erhalte die Sensorreihenfolge.

4. **Lege den ROS-2-Datenvertrag fest.** Definiere eine Nachricht mit unveränderten Matrixwerten, Empfangszeitstempel, Dimensionen und dokumentierter Reihenfolge. Wähle Nachrichten- und Topic-Namen bei der Umsetzung. Prüfe, dass rosbag2 die Daten aufzeichnen und wiedergeben kann.

5. **Baue und prüfe den Parser.** Begrenze den Empfangspuffer. Prüfe vollständige Frames und die Wiederaufnahme nach beschädigten oder unvollständigen Frames mit dem Rohmitschnitt und passenden Fehlerfällen. Veröffentliche keine unvollständige Matrix.

6. **Implementiere den Sensorknoten am CM5.** Verbinde Sensorinitialisierung, Parser und ROS-2-Veröffentlichung. Zeige Empfangsrate, verworfene Frames und ausbleibende Daten. Prüfe eine unterbrochene Verbindung und die erneute Aufnahme des Empfangs. Lege Paketstruktur und Konfiguration bei der Umsetzung fest.

7. **Erstelle die Live-Heatmap.** Zeige 20 Doppler-Bins und 16 Entfernungszellen mit beschrifteten Achsen und Farbskala. Ergänze eine logarithmische Ansicht der Rohwerte. Prüfe die Orientierung mit einer eindeutig beschrifteten Testmatrix und echten Daten. Kennzeichne ausbleibende Daten sichtbar, damit eine alte Matrix nicht aktuell erscheint.

8. **Prüfe rosbag2 ohne Sensor.** Zeichne den leeren Messbereich, eine ruhende Person und eine bewegte Person auf. Dokumentiere Aufbau und Dauer und vergleiche die Beobachtungen. Beende den Sensorknoten und spiele die Aufzeichnungen erneut ab. Die Heatmap muss ohne Hardware erscheinen. Prüfe das Ende der Wiedergabe auf veraltete Daten.

9. **Prüfe die Fehleranzeigen.** Unterbrich die Verbindung und prüfe den Timeout der Anzeige. Prüfe mit beschädigten oder unvollständigen Testframes, dass der begrenzte Empfangspuffer wieder gültige Frames erkennt. Protokolliere Empfangsrate, verworfene Frames und ausbleibende Daten. Nach Wiederherstellung der Verbindung müssen neue Sensordaten wieder erscheinen.

10. **Dokumentiere und prüfe die Abnahme.** Schreibe eine Anleitung unter `docs/cm5/software/how-to/` für Anschluss, Start, Aufnahme und Wiedergabe. Dokumentiere Nachrichten, Parameter, Topic-Namen, Matrixreihenfolge und Messwerte unter `docs/cm5/software/reference/`. Halte Sensorbeobachtungen und ungelöste Skalierungsfragen in `agent_notes/cm5/sensors/hmmd/observations.md` fest. Ändere den Roadmap-Status erst nach der folgenden Abnahme. Führe vor Auslieferung den Scope-Guard und die passenden PStack-Prüfungen aus.

## Prüfe die Abnahme

- [ ] Echte Sensordaten erscheinen fortlaufend als 20 × 16 Heatmap.
- [ ] Leerer Messbereich, ruhende Person und bewegte Person sind aufgezeichnet und verglichen.
- [ ] Eine Bag-Aufzeichnung lässt sich ohne Sensor erneut anzeigen.
- [ ] Eine unterbrochene Verbindung ist sichtbar. Veraltete Daten erscheinen nicht als aktuell.
- [ ] Datenrate, Matrixreihenfolge und offene Skalierungsfragen sind mit ihrer Evidenz dokumentiert.
- [ ] Parserfehler, Puffergrenzen und die Anzeigeorientierung sind reproduzierbar geprüft.

## Koordiniere die Umsetzung

- Kläre Hardware, Rohdaten und Datenvertrag zuerst. Diese Schritte blockieren die Live-Implementierung.
- Implementiere den Parser vor dem Sensorknoten. Entwickle die Heatmap nach dem eingefrorenen Datenvertrag mit Testmatrizen.
- Halte Schreibzugriffe auf Pakete und Nachrichten bei einem verantwortlichen Entwickler. Die begrenzte erste Integration benötigt keinen automatischen Swarm.
- Arbeite in überprüfbaren Einheiten. Prüfe jeweils Parser, ROS-Veröffentlichung, Anzeige und Wiedergabe, bevor die nächste Einheit abgeschlossen wird.
- Nutze den PStack-Feature-Ablauf für die spätere Umsetzung. Erstelle in dieser Planungsrunde weder Code noch PR.

## Referenzen und offene Voraussetzungen

[Waveshare HMMD-Wiki](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor) beschreibt 20 × 16 Werte mit vier Bytes pro Wert. Die Nutzdaten umfassen damit 1.280 Bytes. Die Beschreibung verwendet zugleich den Begriff Chirp für die Sendereihenfolge. Prüfe die Interpretation am Sensor.

Am Planungsstand wurde kein vorhandenes ROS-2- oder HMMD-Paket im Repository gefunden. ROS 2 ist laut Nutzer auf dem CM5 installiert. Die konkrete Distribution, das CM5-Betriebssystem, das Trägerboard, der Anschluss und Live-Sensordaten sind noch nicht geprüft. Paketstruktur, Nachrichtentyp und Topic-Namen werden erst bei der Umsetzung festgelegt.

Roboterintegration, Balancebetrieb, Fahrentscheidungen, LiDAR, SLAM und Navigation bleiben außerhalb dieses Meilensteins. Die ESP32-Firmware und ihre Kontrollschleifen werden für den stationären CM5-Test nicht geändert.

## Umsetzungsstand am 04.10.2026

Der Nutzer hat die Ausführung mit "Implementiere soweit möglich" beauftragt. Schritte 4–7 und die Softwareteile von 9–10 sind implementiert und lokal mit synthetischen Daten geprüft. 15 Tests bestanden; der ROS-Graph-Test ist mangels lokaler ROS-Laufzeit übersprungen. Schritt 8 ist über getrennte Topics, denselben Anzeigeknoten und rosbag2-Bedienanleitung vorbereitet, aber nicht live geprüft.

Schritte 1–3 sowie der Build und die Aufnahme/Wiedergabe auf dem CM5 bleiben offen, da kein Zugang zum konkreten Aufbau vorliegt. Die Roadmap-Abnahme bleibt unvollständig. Einstieg ist [Start, Aufnahme und Wiedergabe](../how-to/hmmd-ros2.md).
