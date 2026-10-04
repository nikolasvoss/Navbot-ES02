# HMMD-Aufbau und Beobachtungen

## Belegter Stand

| Datum | Beobachtung | Evidenz | Praktische Konsequenz |
| --- | --- | --- | --- |
| 04.10.2026 | ROS 2 ist auf einem CM5 installiert. | Nutzeranweisung in der Planungsrunde. | Keine ROS-2-Neuinstallation planen. Distribution und Arbeitsumgebung vor Paketbau erfassen. |
| 04.10.2026 | Der laufende CM5 verwendet Ubuntu 24.04.5 LTS, Kernel `6.8.0-1064-raspi`, ROS 2 Jazzy und Python 3.12.3. `/dev/ttyAMA0` war vorhanden; der Prozess `hmmd_sensor` las diesen Port und veröffentlichte `/hmmd/rdmap` als Best Effort sowie `/hmmd/status`. | `uname -a`, `/etc/os-release`, `echo $ROS_DISTRO`, `ros2 topic info /hmmd/rdmap --verbose`, `ros2 topic echo /hmmd/status --once`, `/dev/ttyAMA*` und Prozessliste am CM5 am 04.10.2026. | ROS nicht neu installieren. Für Browserzugriff rosbridge Jazzy passend installieren und das vorhandene `hmmd_sensor` nicht parallel starten. |
| 04.10.2026 | Ein echter HMMD-Datenstatus meldete `connected=true`, `stale=false`, `frames_received=15508`, `frame_rate_hz=5.000`, `last_frame_age_sec=0.140` und keine I/O-Fehler. | Ausgabe von `ros2 topic echo /hmmd/status --once` am CM5; anschließend erhielten sowohl `roslibpy` 2.1.0 als auch ein ROS-vs-WebSocket-Vergleich echte `/hmmd/rdmap`-Nachrichten. | Der UART ist nicht nur geöffnet: vollständige HMMD-Frames wurden empfangen. Die nominale Browsergrenze 10 Hz ist ein Darstellungsmaximum, keine Sensorfrequenz. |
| 04.10.2026 | Das aktive WLAN war `192.168.178.28/24`; ein PC-Browser mit Quelladresse `192.168.178.22` lud HTML, CSS und JavaScript vom statischen Server erfolgreich. | `ip -br addr`; HTTP-Serverzugriffe auf `/`, `/style.css`, `/app.mjs`, `/topic_registry.mjs`, `/model.mjs` und `/rosbridge_client.mjs` mit HTTP 200 am 04.10.2026. | Browser-URL zum Zeitpunkt der Prüfung: `http://192.168.178.28:8080/`. Die HMMD-WebSocket-Anzeige vom PC ist erst nach lokalem SSH-Tunnel auf dem PC erreichbar. |
| 04.10.2026 | Nutzer meldet den HMMD-Sensor am J8 angeschlossen und nennt „GPIO10, GPIO8 usw.“; ob GPIO-Nummern oder physische J8-Pinnummern gemeint sind und welche Moduldrähte dort liegen, ist noch ungeklärt. | Nutzerbericht in der Anleitungskorrektur. Keine Sichtprüfung oder elektrische Messung liegt vor. | Vor dem Einschalten physische J8-Pinnummern, Modul-TX/RX, Masse und 3,3-V-Versorgung anhand der realen Markierungen prüfen. GPIO8/GPIO10 sind kein UART3-TX/RX-Paar. |
| 04.10.2026 | Roadmap-Punkt 1 verlangt einen stationären HMMD-Test mit Heatmap und Bag-Wiedergabe. | `docs/cm5/software/roadmap.md`, Abschnitt 1. | Radar zunächst getrennt von der Fahrsteuerung betreiben. |

## Noch nicht beobachtet

- Konkrete HMMD-Modul- und Firmwareversion.
- CM5-Trägerboard, tatsächliche J8-Verdrahtung, Versorgung und Signalpegel.
- Baudrate, erfolgreiche Debug-Initialisierung und ACK.
- Byte-Reihenfolge, physikalische Skalierung und Sensororientierung über die vorhandenen Softwarebefunde hinaus.
- Lastverhalten unter längerer oder höherer Bridge-Last.

## Schema für neue Befunde

Jeder neue Eintrag enthält Datum, konkrete Beobachtung, Evidenzdatei oder Messprotokoll und die praktische Konsequenz. Kennzeichne Herstellerangaben und Annahmen separat. Ein fehlender Versuch ist kein Nachweis für ein Sensorverhalten.

## Softwareprüfung am 04.10.2026

Die Pakete `hmmd_interfaces` und `hmmd_radar` sind implementiert. Die Prüfung im Entwicklungsworktree verwendet ausschließlich synthetische Frames und nachgebildete serielle Anschlüsse. 15 Tests bestanden; ein ROS-Graph-Test wurde wegen fehlender lokaler ROS-Laufzeit übersprungen. Der reale Renderer erzeugte Roh-, Log- und veraltete Ansicht, einschließlich Farbskala und Bin-Achsen. Evidenz sind die Tests unter `src/cm5/ros2/src/hmmd_radar/test/` und die ausgeführten Matplotlib-Renderprüfungen.

## CM5-Browserzugang am 04.10.2026

Im separaten Worktree auf Commit `f93ab3f34ee273914629e4e4c0d851866f819ae2` wurden `hmmd_interfaces` und `hmmd_radar` mit ROS 2 Jazzy gebaut. `ros-jazzy-rosbridge-server` und `ros-jazzy-rosbridge-library` Version 2.7.1 wurden ergänzt. Der Server lauschte auf `127.0.0.1:9090`; die statische Browserdatei wurde auf `0.0.0.0:8080` bereitgestellt. Filterstrings mussten wegen der ROS-Stringparameter mit inneren Anführungszeichen übergeben werden. Siehe die HOWTO und Referenz für die verifizierte Filtersemantik; Service-/Action-Capabilities sind keine reine Empfangsgrenze.

Ein roslibpy-Client ohne ROS oder generierte Nachrichtentypen empfing 20×16, 320 Rohwerte, Header/Zeitstempel und Status. Ein direkter ROS-Subscriber und WebSocket-Client sahen einen identischen Frame inklusive Zeitstempel. Der PC-Browser lud die Seite über WLAN; der Nutzer startete den erforderlichen SSH-Tunnel und antwortete auf die Frage nach dem laufenden Zugriff mit „läuft“. Danach zeigte `sudo ss -tnp` eine SSH-Weiterleitung zu `127.0.0.1:9090`; der Bridge-Log zeigte die Abonnements beider HMMD-Topics und deren Wiederverbindung nach dem Neustart. Der echte HMMD-Empfang auf dem CM5 wurde durch synthetische Daten nicht ersetzt.

Die lokale Chromium-Prüfung gegen den echten rosbridge bestand für Roh-/Logansicht, 20×16/320 Werte einschließlich `uint32`-Maximum, Topic-Auswahl und alle Fehlerzustände. Für diese Phasen war PID 3606 (`hmmd_sensor`) kurz pausiert; die ausdrücklich synthetische ROS-Quelle griff nicht auf den UART zu. Danach wurde derselbe Sensorknoten fortgesetzt und echte Frames wieder beobachtet. `sudo fuser -v /dev/ttyAMA0` zeigte anschließend weiterhin nur PID 3606. `sudo pinctrl get 14,15` zeigte GPIO14=TXD0, GPIO15=RXD0; dies bestätigt den Software-Mux, nicht die physische Verdrahtung. Das [Prüfprotokoll](browser-integration-2026-10-04.json) enthält die einzelnen Zustände. Alle 16 Python-Tests einschließlich des isolierten ROS-Graph-Tests und alle sechs Node-Tests bestanden; der Jazzy-Test benötigte eine minimale Annahme von `numbers.Integral` statt nur Python-`int` im Matplotlib-Anzeigemodell.
