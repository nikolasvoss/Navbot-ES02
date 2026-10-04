# HMMD-Aufbau und Beobachtungen

## Belegter Stand

| Datum | Beobachtung | Evidenz | Praktische Konsequenz |
| --- | --- | --- | --- |
| 04.10.2026 | ROS 2 ist auf einem CM5 installiert. | Nutzeranweisung in der Planungsrunde. | Keine ROS-2-Neuinstallation planen. Distribution und Arbeitsumgebung vor Paketbau erfassen. |
| 04.10.2026 | Nutzer meldet den HMMD-Sensor am J8 angeschlossen und nennt „GPIO10, GPIO8 usw.“; ob GPIO-Nummern oder physische J8-Pinnummern gemeint sind und welche Moduldrähte dort liegen, ist noch ungeklärt. | Nutzerbericht in der Anleitungskorrektur. Keine Sichtprüfung oder elektrische Messung liegt vor. | Vor dem Einschalten physische J8-Pinnummern, Modul-TX/RX, Masse und 3,3-V-Versorgung anhand der realen Markierungen prüfen. GPIO8/GPIO10 sind kein UART3-TX/RX-Paar. |
| 04.10.2026 | Roadmap-Punkt 1 verlangt einen stationären HMMD-Test mit Heatmap und Bag-Wiedergabe. | `docs/cm5/software/roadmap.md`, Abschnitt 1. | Radar zunächst getrennt von der Fahrsteuerung betreiben. |

## Noch nicht beobachtet

- Konkrete HMMD-Modul- und Firmwareversion.
- CM5-Trägerboard, Betriebssystem und ROS-2-Distribution.
- Anschluss, Versorgung, Signalpegel und tatsächlicher MCP-Port.
- Baudrate, erfolgreiche Debug-Initialisierung und ACK.
- Echte Frames, Byte-Reihenfolge, Datentyp und Matrixreihenfolge.
- Datenrate, Lastverhalten und physikalische Skalierung.

## Schema für neue Befunde

Jeder neue Eintrag enthält Datum, konkrete Beobachtung, Evidenzdatei oder Messprotokoll und die praktische Konsequenz. Kennzeichne Herstellerangaben und Annahmen separat. Ein fehlender Versuch ist kein Nachweis für ein Sensorverhalten.

## Softwareprüfung am 04.10.2026

Die Pakete `hmmd_interfaces` und `hmmd_radar` sind implementiert. Die Prüfung im Entwicklungsworktree verwendet ausschließlich synthetische Frames und nachgebildete serielle Anschlüsse. 15 Tests bestanden; ein ROS-Graph-Test wurde wegen fehlender lokaler ROS-Laufzeit übersprungen. Der reale Renderer erzeugte Roh-, Log- und veraltete Ansicht, einschließlich Farbskala und Bin-Achsen. Evidenz sind die Tests unter `src/cm5/ros2/src/hmmd_radar/test/` und die ausgeführten Matplotlib-Renderprüfungen.

Ein ROS-2-Build auf dem CM5, tatsächliche Sensordaten, rosbag2-Aufnahme/Wiedergabe und die drei Beobachtungssituationen sind weiterhin offen. Es wurden keine Hardwarebefunde durch synthetische Tests ersetzt.
