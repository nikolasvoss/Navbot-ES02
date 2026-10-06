# Roadmap

## 1. HMMD-Radar als 20 × 16 Range-Doppler-Map in ROS 2 anzeigen

Status: Software implementiert; Live-Abnahme noch offen.

Der CM5-Sensorknoten, die ROS-Topics, die Browseranzeige und der gemeinsame lokale oder entfernte Start sind implementiert. Die vorhandene Evidenz umfasst echte HMMD-Frames und Browserzugriff auf dem CM5. Die vollständige Abnahme steht noch aus.

Vor Abschluss sind der aktuelle ROS-2-Build auf dem CM5, die physische UART- und Spannungsprüfung, Aufzeichnungen eines leeren Messbereichs sowie einer ruhenden und einer bewegten Person und die Wiedergabe ohne Live-Sensor zu bestätigen. Die [HMMD ROS 2 developer guide](hmmd-ros2.md) enthält die aktuellen Befehle und Grenzen.

## 2. Roborock-LiDAR in ROS 2 integrieren

Status: geplant. Zweiter ROS-2-Meilenstein.

Ein LiDAR-Sensor aus einem Roborock-Staubsaugerroboter wird an den CM5 angebunden.
Das erste sichtbare Ergebnis ist ein Live-Laserscan in RViz. Der erste Test erfolgt
stationär außerhalb des Navbot. Die Integration schafft eine Grundlage für spätere
Kartierung und Navigation.

### Grundlage und offene Fragen

Das genaue Roborock-Modell und die Sensorbezeichnung sind noch offen.
Vor dem Anschluss müssen Pinbelegung, Versorgungsspannung, Signalpegel,
Motoransteuerung und Datenprotokoll des konkreten Moduls geklärt werden.
Ein vorhandener ROS-2-Treiber wird erst nach Abgleich mit diesem Modul ausgewählt.

### Geplanter Umfang

- Identifikation des Moduls und Dokumentation der belegten Anschlussdaten.
- Stationärer Betrieb mit geeigneter Versorgung und der erforderlichen Ansteuerung
  des Scanmotors, sofern das Modul diese extern benötigt.
- Anbindung an den CM5 und Veröffentlichung als `sensor_msgs/msg/LaserScan`
  auf dem Topic `/scan`.
- Dokumentation von Winkelrichtung, Winkelnullpunkt, Entfernungseinheiten,
  Zeitstempeln und ungültigen Messwerten.
- Darstellung in RViz mit einem eigenen Sensor-Koordinatensystem.
- Aufzeichnung und Wiedergabe mit rosbag2 sowie Anzeige ausbleibender Scans.
- Vorbereitung der Montage am Navbot einschließlich Gewicht, Versorgung,
  freiem Sichtbereich und Transformation zum Roboter-Koordinatensystem.

### Fertig, wenn

- Echte Scans erscheinen fortlaufend in RViz.
- Abstände und Winkel stimmen bei mehreren bekannten Positionen im Testaufbau
  innerhalb der für das identifizierte Modul festgelegten Toleranzen.
- Eine Aufzeichnung lässt sich ohne angeschlossenen Sensor erneut anzeigen.
- Eine unterbrochene Verbindung wird erkannt; alte Scans erscheinen nicht als aktuell.
- Sensorbezeichnung, Anschlussdaten, Treiber und beobachtete Scanrate sind dokumentiert.

Vor dem Einsatz während der Fahrt werden die Auswirkungen von Neigung,
Balancebewegungen, zusätzlichem Gewicht und Strombedarf geprüft.
SLAM und autonome Navigation sind nachfolgende Projekte.
