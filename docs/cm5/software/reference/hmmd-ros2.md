# HMMD ROS-2-Schnittstelle

## Pakete und Daten

Die Python-Pakete benötigen Python ab Version 3.10. Beim CM5-Setup am 4. Oktober 2026 wurden Ubuntu 24.04.5 LTS, ROS 2 Jazzy und `ros-jazzy-rosbridge-server` `2.7.1-1noble.20260902.151943` und `ros-jazzy-rosbridge-library` `2.7.1-1noble.20260902.151608` beobachtet. Der HMMD-Workspace wurde unter Python 3.12.3 gebaut und aktiviert.

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

`src/cm5/ros2/src/hmmd_radar/web/` enthält eine statische, abhängigkeitenfreie Browseransicht. `topic_registry.mjs` begrenzt die dargestellten Topics auf `/hmmd/rdmap` (`hmmd_interfaces/msg/RangeDopplerMap`, Best Effort) und `/hmmd/status` (`diagnostic_msgs/msg/DiagnosticArray`, Reliable). Der Browser sendet die [Standard-JSON-Nachrichten](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md) `subscribe` und `unsubscribe` und empfängt `publish`-Nachrichten; er implementiert keine Publish-, Service- oder Action-Aufrufe. Neue Topics benötigen eine explizite Registry-Änderung und eine passende serverseitige Abonnementfreigabe.

Der Standard-Endpunkt ist `ws://127.0.0.1:9090`. Auf dem CM5 ist rosbridge an `127.0.0.1:9090` gebunden. Der PC-Tunnel beim Setup lautet `ssh -N -L 127.0.0.1:9090:127.0.0.1:9090 niko@192.168.178.28`. Die statische Seite wird über `0.0.0.0:8080` ausgeliefert; die beim Setup beobachtete Browser-URL ist `http://192.168.178.28:8080/`. Diese WLAN-Adresse ist DHCP-abhängig.

Der Client fordert maximal 10 Hz mit `throttle_rate: 100` Millisekunden, Keep-Last-Tiefe 1 und Volatile-Durability an. Für `/hmmd/rdmap` ist die Reliability Best Effort; für `/hmmd/status` Reliable. Er hält nur den zuletzt empfangenen Wert pro Topic, verwendet monotone lokale Empfangszeiten und verwirft die Frische alter Samples nach Bridge-Neuverbindungen, bis neue Daten eintreffen. HMMD-Portverbindung, Sensor-Frame-Frische und rosbridge-Verbindung werden als getrennte Zustände dargestellt.

Beim eingerichteten rosbridge 2.7.1 sind `topics_glob`, `topics_pub_glob`, `topics_sub_glob` und `services_glob` als Stringparameter implementiert. Übergib Listen als Stringwerte, z. B. `'topics_sub_glob:="[/hmmd/rdmap,/hmmd/status]"'`; ungequotete Listen werden als ROS-`STRING_ARRAY` gewertet und lassen Launch-Nodes mit einem Typfehler abbrechen. Ein leerer String bedeutet keine Filterung. Der aktive Start setzt `topics_glob:=""`, `topics_pub_glob:="[]"`, `topics_sub_glob:="[/hmmd/rdmap,/hmmd/status]"`, `services_glob:="[]"` und `params_glob:="[]"`.

Die Topic-Filter wurden am laufenden Server geprüft: Abos auf `/rosout` und Publish auf `/hmmd/browser_filter_probe` wurden abgewiesen; die beiden HMMD-Topics wurden erfolgreich abonniert. Die rosbridge-Version ergänzt bei gesetztem `services_glob` automatisch `/rosapi/*` als Muster. `/rosapi/topics` antwortete mit der gefilterten HMMD-Liste, während `/hmmd_sensor/get_parameters` abgewiesen wurde; rosapi-Dienste bleiben unter dem pauschalen Muster erreichbar. Zusätzlich deklariert der Server `actions_glob`, aber die installierte Launch-Datei bietet kein gleichnamiges Argument; beim dokumentierten Start bleiben Action-Operationen ungefiltert. Der Browserclient bleibt empfangsorientiert, doch rosbridge ist keine nachgewiesene reine Empfangsschnittstelle. Loopback-Bindung und SSH-Tunnel sind notwendig; Port 9090 ist nicht im LAN zu öffnen.

`test/web/synthetic_rosbridge.py` ist ausschließlich ein lokaler QA-Simulator mit synthetischen Nachrichten. Er simuliert weder den CM5 noch den HMMD oder die produktive rosbridge-Sicherheitskonfiguration. Zusätzlich wurde am CM5 ein realer HMMD-Frame direkt mit dem gleichen Frame über rosbridge verglichen. Die automatisierte Chromium-Prüfung auf dem CM5 gegen den echten Server bestand für Roh-/Logansicht, weiteres Topic, fehlende Frames, Publisher-Stopp, Bridge-Abbruch und Wiederverbindung ohne frische alte Samples. Die Ausfallphasen verwendeten ausdrücklich synthetische ROS-Nachrichten; davor und danach waren echte HMMD-Frames sichtbar. Ein PC-Browser lud HTML, CSS und JavaScript von `192.168.178.28:8080` mit HTTP 200. Der Nutzer bestätigte den laufenden Zugriff nach dem SSH-Tunnelstart; der Server zeigte die Weiterleitung durch `sshd` und die beiden HMMD-Abos nach der Wiederverbindung. Die lokale Chromium-Prüfung ist kein automatisierter PC-Browsertest. Das [Prüfprotokoll](../../../../agent_notes/cm5/sensors/hmmd/browser-integration-2026-10-04.json) trennt diese Evidenz.

## Verifikationsgrenze

Die Live-Verifikation empfing am CM5 echte Sensorframes über `/dev/ttyAMA0`; der Status meldete `connected=true`, `stale=false` und etwa 5 Hz. Das belegt tatsächliche Frames durch den vorhandenen Sensorknoten, aber keine elektrische Sichtprüfung, genaue Verkabelung, Sensororientierung oder physikalische Skalierung. Der PC-Zugriff über LAN und SSH-Tunnel ist durch HTTP-Zugriffe, serverseitige SSH-/Abonnementbeobachtungen und die Nutzerrückmeldung belegt; eine automatisierte DOM-Prüfung auf dem PC wurde nicht durchgeführt. Hardwarebeobachtungen und Evidenz stehen in [observations.md](../../../../agent_notes/cm5/sensors/hmmd/observations.md).
