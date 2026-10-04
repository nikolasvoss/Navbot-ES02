# HMMD ROS-2-Schnittstelle

## Pakete und Daten

Die Python-Pakete benötigen Python ab Version 3.10. ROS 2 Jazzy ist laut Benutzerangabe auf dem CM5 installiert. Distribution, rosbridge-Version und tatsächliche CM5-Konfiguration wurden in diesem Arbeitslauf nicht per SSH bestätigt.

`src/cm5/ros2/src/hmmd_interfaces` enthält die ROS-Nachricht `RangeDopplerMap`. `src/cm5/ros2/src/hmmd_radar` enthält Parser, seriellen Sensorknoten und Heatmap. Der Sensorknoten ist unabhängig von der ESP32-Firmware und ihrer Fahrsteuerung.

| Topic | Typ | Bedeutung |
| --- | --- | --- |
| `/hmmd/rdmap` | `hmmd_interfaces/msg/RangeDopplerMap` | Vollständige rohe Matrix mit Host-Empfangszeitstempel. |
| `/hmmd/status` | `diagnostic_msgs/msg/DiagnosticArray` | Zustand, Empfangsrate, Fehlerzähler und Alter des letzten Frames. |

| Matrixfeld | Bedeutung |
| --- | --- |
| `header.stamp` | ROS-Zeit beim Host-Empfang, kein Sensor-Messzeitstempel. |
| `header.frame_id` | Sensorbezeichnung im ROS-System. |
| `doppler_bins` | 20 gemäß dokumentiertem Debug-Frame. |
| `range_gates` | 16 gemäß dokumentiertem Debug-Frame. |
| `amplitude_squared` | 320 unveränderte `uint32`-Werte. |

Der flache Index ist `doppler_bin * 16 + range_gate`. Die Heatmap enthält 20 Zeilen und 16 Spalten. Die erste Darstellung erhält die Hersteller-Sendereihenfolge ohne Umsortierung. Physikalische Skalierung, Doppler-Vorzeichen und die Interpretation der beschriebenen Chirps sind am vorhandenen Sensor noch nicht bestätigt.

## Startparameter

Die Anwendungsparameter werden beim Start gesetzt. Änderungen benötigen einen Neustart des Knotens.

| Sensorknoten-Parameter | Standard | Bedeutung |
| --- | --- | --- |
| `port` | leer | Ohne Port findet kein serieller Zugriff statt. |
| `baud_rate` | 115200 | UART-Baudrate. |
| `poll_period_sec` | 0.01 | Abstand der begrenzten Empfangsabfragen. |
| `stale_timeout_sec` | 1.0 | Timeout ohne vollständigen Frame und für unvollständige Frames. |
| `diagnostics_period_sec` | 1.0 | Abstand der Statusveröffentlichungen. |

| Heatmap-Parameter | Standard | Bedeutung |
| --- | --- | --- |
| `stale_timeout_sec` | 1.0 | Maximale Zeit seit lokaler Frame-Zustellung vor der Kennzeichnung als veraltet. |
| `render_hz` | 10.0 | Maximale angeforderte Renderfrequenz. |

Das Tastenkürzel `l` schaltet die Heatmap zwischen Rohwerten und `log1p` um. Die Farbgrenzen werden je dargestellter Matrix berechnet. Ein konstantes Feld erhält eine nichtleere Farbspanne. Vergleiche deshalb bei unterschiedlichen Aufnahmen die Zahlen der Farbskala.

## Empfangsgrenzen und Statusfelder

Jede Abfrage liest höchstens 4.096 Bytes. Bei größerem beobachtetem Eingangsbestand wird der serielle Eingang verworfen und der Parser zurückgesetzt; `input_backlog_overflows` zählt das Ereignis. Der Parser behält höchstens 2.576 Bytes im Puffer. Schreibvorgänge für die Initialisierung haben einen Timeout von 0,5 Sekunden. Wiederverbindungsversuche beginnen nach 0,25 Sekunden und verlängern sich bis höchstens 5 Sekunden. Dies sind Softwaregrenzen, keine am CM5 gemessenen Leistungswerte.

| Statusfeld | Bedeutung |
| --- | --- |
| `connected` | Der Host-Port ist geöffnet. Dies belegt noch keine Matrixdaten. |
| `stale` | Es liegt kein kürzlich empfangener vollständiger Frame vor. |
| `last_frame_age_sec` | Monotone Host-Zeit seit dem letzten vollständigen Frame, oder `unknown`. |
| `frames_received` | Anzahl vollständig dekodierter Frames. |
| `frame_rate_hz` | Empfangene Frames pro Sekunde seit der vorherigen Statusveröffentlichung. |
| `malformed_candidates` | Kandidaten mit Header und ungültigem Footer. Keine vollständige Zählung aller verlorenen Sensorframes. |
| `discarded_bytes` | Vom Parser verworfene Bytes einschließlich Rauschen und unvollständiger Daten. |
| `reconnects` | Verbindungsabbrüche und fehlgeschlagene Öffnungsversuche. |
| `input_backlog_overflows` | Überschreitungen der seriellen Eingangsgrenze. |
| `last_io_error` | Letzter gemeldeter serieller Fehler. |

## Drahtformat

Die [lokalen Herstellerunterlagen](../../../../agent_notes/cm5/sensors/hmmd/vendor/README.md) nennen Header `AA BF 10 14`, 1.280 Nutzdatenbytes und Footer `FD FC FB FA`. Die Implementierung interpretiert jeweils vier Bytes als Little-Endian-`uint32`. Ein vollständiger Frame umfasst 1.288 Bytes. Ohne Prüfsumme wird keine vollständige Erkennung von Payloadverfälschungen behauptet.

Der Debug-Befehl lautet `FD FC FB FA 08 00 12 00 00 00 00 00 00 00 04 03 02 01`. Eine zum vorhandenen Modul bestätigte ACK-Sequenz liegt nicht vor. Der erste vollständige Debug-Datenframe ist der Nachweis für Datenempfang.

## QoS und Wiedergabe

Die Matrix verwendet Keep-Last mit Tiefe 5, Best Effort und Volatile. Die Anzeige verwendet kompatible Best-Effort-QoS. Der Status verwendet Reliable mit begrenzter History. rosbag2 erkennt die angebotene QoS normalerweise über den Publisher. Die tatsächliche Aufnahme ist vor der Abnahme durch `ros2 bag info` zu prüfen.

Ein Override für eine Aufnahme kann diese Datei enthalten.

```yaml
/hmmd/rdmap:
  history: keep_last
  depth: 5
  reliability: best_effort
  durability: volatile
```

Die Option dafür ist `--qos-profile-overrides-path`. Die [rosbag2-Referenz](https://github.com/ros2/rosbag2) beschreibt Aufzeichnung, Wiedergabe und QoS-Overrides. Die installierte Version liefert die konkreten Optionen über `ros2 bag record --help` und `ros2 bag play --help`.

Die Anzeige bewertet Frische nach lokaler monotoner Empfangszeit. Originalzeitstempel bleiben in der Nachricht erhalten. Eine pausierte oder beendete Wiedergabe führt nach dem Timeout zur Kennzeichnung veralteter Daten, auch wenn `/clock` steht.

## Darstellung

Die Rohansicht zeigt die ganzzahligen Amplitudenquadrate. Die logarithmische Ansicht verwendet `log1p(value)` und kann deshalb Nullwerte darstellen. Farbskalierung verändert keine Matrixnachricht. Beide Achsen verwenden Bin-Indizes.

## Browser-Datenagent

`src/cm5/ros2/src/hmmd_radar/web/` enthält eine statische, abhängigkeitenfreie Browseransicht. `topic_registry.mjs` begrenzt die dargestellten Topics auf `/hmmd/rdmap` (`hmmd_interfaces/msg/RangeDopplerMap`, Best Effort) und `/hmmd/status` (`diagnostic_msgs/msg/DiagnosticArray`, Reliable). Der Browser spricht rosbridge über die [Standard-JSON-Nachrichten](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md) `subscribe`, `unsubscribe` und `publish` an; er implementiert keine Publish-, Service- oder Action-Aufrufe. Neue Topics benötigen eine explizite Registry-Änderung und eine passende serverseitige Abonnementfreigabe.

Der Standard-Endpunkt ist `ws://127.0.0.1:9090`. Auf dem CM5 muss rosbridge an `127.0.0.1` gebunden sein; der PC stellt den Port über `ssh -N -L 127.0.0.1:9090:127.0.0.1:9090 cm5` bereit. Die statische Seite kann auf dem CM5 im LAN über Port 8080 ausgeliefert werden. Das ist eine statische Dateiauslieferung, keine zusätzliche ROS-Schnittstelle.

Der Client fordert maximal 10 Hz mit `throttle_rate: 100` Millisekunden, Keep-Last-Tiefe 1 und Volatile-Durability an. Für `/hmmd/rdmap` ist die Reliability Best Effort; für `/hmmd/status` Reliable. Er hält nur den zuletzt empfangenen Wert pro Topic, verwendet monotone lokale Empfangszeiten und verwirft die Frische alter Samples nach Bridge-Neuverbindungen, bis neue Daten eintreffen. HMMD-Portverbindung, Sensor-Frame-Frische und rosbridge-Verbindung werden als getrennte Zustände dargestellt.

Die Seite filtert lokal auf die freigegebene Registry, ist aber keine serverseitige Sicherheitsgrenze. rosbridge stellt zusätzliche Protokolloperationen bereit. Verwende Topic-Freigaben, binde den Bridge-Server an Loopback und greife per SSH-Tunnel zu. Launch-Argumente und Filtersemantik hängen von der installierten `rosbridge_server`-Version ab; vor dem Start sind `ros2 launch rosbridge_server rosbridge_websocket_launch.xml --show-args` und die Paketversion zu prüfen. Sind getrennte Topic-Filter nicht verfügbar, rosbridge nicht mit diesem Browser verbinden, bis eine begrenzte Konfiguration bereitsteht.

`test/web/synthetic_rosbridge.py` ist ausschließlich ein lokaler QA-Simulator mit synthetischen Nachrichten. Er simuliert weder den CM5 noch den HMMD oder die produktive rosbridge-Sicherheitskonfiguration.

## Verifikationsgrenze

Tests mit synthetischen Frames belegen Softwareverhalten. Sie belegen keine tatsächliche Sensor-Bildrate, Montageorientierung, Anschlusssicherheit oder ROS-2-Integration auf dem CM5. Hardwarebefunde stehen getrennt in [observations.md](../../../../agent_notes/cm5/sensors/hmmd/observations.md).
