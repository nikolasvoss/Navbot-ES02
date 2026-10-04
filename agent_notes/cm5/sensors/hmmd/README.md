# HMMD-Sensor für Agenten

## Auftrag und aktueller Stand

Der erste [Roadmap-Meilenstein](../../../../docs/cm5/software/roadmap.md) zeigt die HMMD-Range-Doppler-Map stationär in ROS 2 auf dem CM5. ROS 2 ist laut Nutzer bereits auf dem CM5 installiert. Die genaue Distribution ist noch nicht erfasst. Sensoranschluss und Live-Daten sind noch nicht geprüft.

## Dokumente finden

- [Lokale Herstellerunterlagen](vendor/README.md) enthält Wiki, Radome-Leitfaden, Beispielpaket und Prüfsummen.
- [Start, Aufnahme und Wiedergabe](../../../../docs/cm5/software/how-to/hmmd-ros2.md) beschreibt die Bedienung der Software.
- [ROS-2-Schnittstelle](../../../../docs/cm5/software/reference/hmmd-ros2.md) dokumentiert Topics, Matrix und QoS.
- [Protokollreferenz](protocol.md) enthält Herstellerangaben und ihre Grenzen.
- [Beobachtungen und offene Punkte](observations.md) enthält den belegten Stand des Aufbaus.
- [Schritt-für-Schritt-Plan](../../../../docs/cm5/software/plans/hmmd-ros2-rdmap-plan.md) beschreibt die spätere Umsetzung.
- [Hardwareübersicht](../../../../docs/robot/hardware/system-overview.md) beschreibt die bestehende Navbot-Elektronik. Der stationäre Radaraufbau ist davon getrennt.
- [USB- und Serial-Regeln](../../../../docs/robot/software/development/usb-serial-flashing.md) gelten vor seriellem Zugriff.

## Hinweise für ausführende Agenten

Lies zuerst diesen Einstieg, danach nur die für den Auftrag benötigte Referenz. Unterscheide Herstellerangaben, Nutzerangaben, Vorschläge und gemessene Beobachtungen. Die Quelle des jeweiligen Eintrags entscheidet über seine Aussagekraft.

Verwende für direkte serielle Zugriffe den konfigurierten Serial-MCP. Ermittle den Port mit `list_ports`. Setze nach `open` RTS und DTR niedrig, sofern der Auftrag nichts anderes erfordert. Sende Befehle nur für einen entsprechenden Auftrag. Auf dem CM5 muss der Sensor über den verwendeten MCP erreichbar sein.

Prüfe vor Versorgung oder Verkabelung die tatsächliche Modulvariante, Pinbelegung, Spannung und Signalpegel. Übertrage die Pinbelegung eines Raspberry-Pi-Beispiels nicht ungeprüft auf das CM5-Trägerboard.

Notiere neue Hardwarebefunde in `observations.md` mit Datum, Beobachtung, Evidenz und praktischer Konsequenz. Aktualisiere `protocol.md` nur bei belegten Protokollbefunden. Bewahre Rohmitschnitte und Messwerte als Evidenz auf. Dokumentiere Annahmen ausdrücklich.

Nutze Bin-Indizes für die erste Heatmap. Gib keine Meter- oder Geschwindigkeitsachsen ohne nachgewiesene Skalierung an. Die Map liefert keine Links-rechts-Ortung.

Ändere für den stationären Test weder ESP32-Firmware noch Balance-, Motor- oder Fahrsteuerung. LiDAR, SLAM und Navigation gehören zu späteren Aufgaben.
