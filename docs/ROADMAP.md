# Roadmap

## 1. HMMD-Radar als 20 × 16 Range-Doppler-Map in ROS 2 anzeigen

Status: geplant. Erster ROS-2-Meilenstein.

Der CM5 liest den HMMD-Radarsensor und veröffentlicht seine Range-Doppler-Map
als ROS-2-Daten. Eine Live-Heatmap zeigt, wie sich die Radarantwort bei Anwesenheit
und Bewegung verändert. Der Einstieg erfolgt mit stationärem Sensor unabhängig
von der Fahrsteuerung des Navbot.

### Grundlage und offene Fragen

Das [Waveshare-Wiki zum HMMD](https://www.waveshare.com/wiki/HMMD_mmWave_Sensor)
beschreibt im Debug-Modus eine RDMAP mit 20 Doppler-Bins und 16 Entfernungszellen.
Jeder Wert belegt vier Bytes und beschreibt das Quadrat der Amplitude.
Damit enthält eine Map 1.280 Bytes Nutzdaten.

Das Wiki beschreibt die Übertragungsreihenfolge zugleich anhand von Chirps.
Die Anordnung der Matrix, die Bedeutung der Doppler-Bins und die tatsächlich
erreichbare Bildrate müssen deshalb am Sensor geprüft werden.
Die erste Ansicht verwendet Bin-Indizes. Physikalische Achsen in Metern oder
Metern pro Sekunde folgen erst mit belegter Skalierung und Vorzeichenkonvention.
Die Map enthält keine Links-rechts-Ortung und ist keine räumliche Umgebungskarte.

### Geplanter Umfang

- Anbindung an den CM5 über eine separate UART-Verbindung oder einen USB-UART-Adapter
  mit passenden 3,3-V-Signalpegeln.
- ROS-2-Sensorknoten für den dokumentierten Debug-Modus, mit begrenztem Empfangspuffer
  und Wiederaufnahme nach unvollständigen oder beschädigten Frames.
- Veröffentlichung der unveränderten Matrixwerte mit Empfangszeitstempel,
  Matrixdimensionen und dokumentierter Reihenfolge.
- Live-Heatmap mit beschrifteten Achsen und Farbskala. Eine logarithmische Ansicht
  ergänzt die Rohwerte. Die Anzeige kann zunächst in einem eigenen ROS-2-Fenster laufen.
- Aufzeichnung und Wiedergabe mit rosbag2 für wiederholbare Vergleiche.
- Anzeige von Empfangsrate, verworfenen Frames und ausbleibenden Daten.

### Fertig, wenn

- Echte Sensordaten erscheinen fortlaufend als 20 × 16 Heatmap.
- Beobachtungen mit leerem Messbereich, ruhender Person und bewegter Person
  sind aufgezeichnet und miteinander verglichen.
- Eine Aufzeichnung lässt sich ohne angeschlossenen Sensor erneut anzeigen.
- Eine unterbrochene Verbindung wird angezeigt; alte Daten erscheinen nicht als aktuell.
- Beobachtete Datenrate, Matrixanordnung und offene Skalierungsfragen sind dokumentiert.

Spätere Messungen auf dem Roboter vergleichen Ruhe, Balancebewegungen und Fahrt.
Sie klären, wie stark Eigenbewegung die Radarantwort verändert, bevor diese Daten
Fahrentscheidungen beeinflussen.

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
