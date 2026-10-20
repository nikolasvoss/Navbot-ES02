# Roadmap

## 1. HMMD-Radar als 20 × 16 Range-Doppler-Map in ROS 2 anzeigen

Status: Software implementiert; Live-Abnahme noch offen.

Der ros2_pc-Sensorknoten, die ROS-Topics, die Browseranzeige und der gemeinsame lokale oder entfernte Start sind implementiert. Die vorhandene Evidenz umfasst echte HMMD-Frames und Browserzugriff auf dem ros2_pc. Die vollständige Abnahme steht noch aus.

Vor Abschluss sind der aktuelle ROS-2-Build auf dem ros2_pc, die physische UART- und Spannungsprüfung, Aufzeichnungen eines leeren Messbereichs sowie einer ruhenden und einer bewegten Person und die Wiedergabe ohne Live-Sensor zu bestätigen. Die [HMMD ROS 2 developer guide](../software/hmmd-ros2.md) enthält die aktuellen Befehle und Grenzen.

## 2. Roborock-LiDAR in ROS 2 integrieren

Status: Echte Browser-Scans und ROS-2-Topic `/scan` am 15.10.2026 bestätigt; RViz, Kalibrierung und Bag-Wiedergabe noch offen. Zweiter ROS-2-Meilenstein.

Das `lds08rr_lidar`-Paket bietet eine Demo ohne Radar, einen unabhängigen Browserstatus
und einen ROS-2-Adapter. Es verwendet den bestätigten Camsense-Treiber aus
`lds2d[viz]==0.7.0` und veröffentlicht vollständige Scans auf `/scan` sowie den Status
auf `/lidar/status`. Kein Host-PWM-Ausgang wird aktiviert. Die Hardware- und
Protokollgrenzen stehen in der [LDS08RR-Hardwaredokumentation](../hardware/lds08rr-lidar.md).
Die [Softwareanleitung](../software/lds08rr-lidar.md) enthält Build-, Demo- und
Startbefehle.

Das erste sichtbare Hardwareergebnis soll ein Live-Laserscan in RViz sein. Der erste
Test erfolgt stationär außerhalb des Navbot. Die Software allein bestätigt weder
Wiring noch Sensorprotokoll, Orientierung, Reichweite oder Scanrate.

### Grundlage und offene Fragen

Der TX-Mitschnitt `acq0003.csv` bestätigt das LDS08RR-Camsense-Format bei 115200 Baud,
8N1. Ein direkter Mitschnitt auf `/dev/ttyAMA0` lieferte 450 gültige Pakete ohne
Prüfsummenfehler; Browser und ROS empfangen echte Scans bei etwa 5 Hz. Winkelnullpunkt,
Drehrichtung, Messgenauigkeit und elektrische Abnahme bleiben offen.

- [x] Nach Protokollbestätigung und gültigen Scans die unbenutzte Delta-Anbindung,
  Protokollauswahl und zugehörigen Tests und Anleitungen entfernt.

### Geplanter Umfang

- Identifikation des Moduls und Dokumentation der belegten Anschlussdaten.
- Stationärer Betrieb mit geeigneter Versorgung und der erforderlichen Ansteuerung
  des Scanmotors, sofern das Modul diese extern benötigt.
- Anbindung an den ros2_pc und Veröffentlichung als `sensor_msgs/msg/LaserScan`
  auf dem Topic `/scan`. Die Software verwendet einen festen Winkelraster und
  positive Unendlichkeit für Messwerte ohne Rücklauf.
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

## 3. Telemetrie und Logging über den ros2_pc

Status: Geplant, noch nicht begonnen.

Hochratige Telemetrie und Aufzeichnungen des ESP32 laufen künftig über die UART-Verbindung zum ros2_pc und werden dort aufgezeichnet, zum Beispiel mit rosbag2. Der ESP32 betreibt keinen eigenen WLAN-Logging-Dienst; die früheren WLAN-Logging-Vorschläge sind entfernt. Format, Rate und Aufzeichnungsumfang sind noch festzulegen.
