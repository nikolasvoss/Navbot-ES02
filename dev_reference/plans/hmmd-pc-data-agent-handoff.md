# Auftrag: CM5-Daten auf dem PC ohne ROS-Installation anzeigen

Implementiere einen wiederverwendbaren Datenzugang vom CM5 zu einem PC im lokalen Netzwerk. Auf dem PC soll ein Browser genügen. Verwende den vorhandenen HMMD-Radarknoten als ersten Datenlieferanten.

## Bestätigter Ausführungsumfang

- CM5-SSH-Ziel: `ssh cm5`.
- Implementiere die Lösung und richte sie auf dem CM5 ein. Der Nutzer hat diese Einrichtung ausdrücklich beauftragt.
- Das Repository ist bereits auf dem CM5 geklont. Ermittle dort den tatsächlichen Projektpfad und Git-Zustand.

Prüfe vor der Umsetzung die tatsächliche CM5-Umgebung. Führe unabhängige lokale Arbeiten fort, falls der SSH-Zugang vorübergehend nicht erreichbar ist. Erfinde keine Zugangsdaten oder Fernzugriffsergebnisse.

## Arbeitsort und Workflow

Arbeite in diesem vorhandenen Worktree:

```text
/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02/.worktrees/hmmd-ros2-rdmap
```

Der Branch heißt `codex/hmmd-ros2-rdmap`. Prüfe den tatsächlichen Git-Zustand und erhalte vorhandene Änderungen. Lies die geltenden `AGENTS.md`-Anweisungen. Lade vor der Implementierung `/home/niko/.codex/skills/custom_skills/workflow/SKILL.md`, erstelle einen eigenen Scope Contract für diesen Auftrag und verwende das dort vorgeschriebene PStack-Verfahren. Alte HMMD-Scope-Verträge dokumentieren frühere Aufträge und sind keine pauschale Freigabe für diesen Auftrag.

Halte die Projektvorgaben zu Scope-Prüfungen und Gemini-Reviews ein. Alle Subagents verwenden GPT-6 Luna. Verwende bei normalen Aufgaben bevorzugt einen oder zwei Subagents, bei einem Architekturvergleich drei. Rufe Swarm nicht automatisch auf. Installiere ROS 2 nicht auf dem PC. Führe keine Firmware-Uploads, Fahrsteuerungsänderungen, Verdrahtungsänderungen oder andere Hardwareprojekte aus. Ein Push, PR oder Merge gehört nicht zu diesem Auftrag.

## Belegter Ausgangspunkt

Der HMMD ist laut Nutzer direkt an der CM5-IO-Platine angeschlossen. Der CM5 ist im lokalen Netzwerk und hat laut Nutzer bereits ROS 2. Betriebssystem, ROS-Distribution, tatsächliche UART-Einrichtung und Live-Empfang sind noch nicht belegt. Die bisherige Softwareprüfung verwendete synthetische Daten.

Der lokale PC läuft mit Linux Mint 22. Bei der bisherigen Prüfung waren dort keine Installation unter `/opt/ros` und kein gebauter HMMD-Workspace vorhanden.

Die bestehenden Pakete liegen unter `src/cm5/ros2/src/`. `hmmd_sensor` liest den UART und veröffentlicht:

- `/hmmd/rdmap`: `hmmd_interfaces/msg/RangeDopplerMap`, 20 Doppler-Bins und 16 Entfernungszellen, 320 rohe `uint32`-Werte. Index: `doppler_bin * 16 + range_gate`. QoS: Best Effort.
- `/hmmd/status`: `diagnostic_msgs/msg/DiagnosticArray` mit Verbindungszustand, Frische, Empfangsrate und Fehlerzählern.

`hmmd_heatmap` ist ein ROS-abhängiger Matplotlib-Viewer. Eine Browser-Anzeige und rosbridge-Einrichtung sind noch nicht enthalten. Der bestehende Sensorknoten bleibt der einzige Leser des UART.

Lies vor dem Entwurf diese Projektquellen:

- `docs/cm5/software/how-to/hmmd-ros2.md`
- `docs/cm5/software/reference/hmmd-ros2.md`
- `agent_notes/cm5/sensors/hmmd/observations.md`
- `src/cm5/ros2/src/hmmd_radar/hmmd_radar/sensor_node.py`
- `src/cm5/ros2/src/hmmd_radar/hmmd_radar/viewer_node.py`
- `src/cm5/ros2/src/hmmd_radar/hmmd_radar/view_model.py`

## Ziel und gewählter Ansatz

Verwende das vorhandene ROS-2-Paket `rosbridge_server` auf dem CM5 als allgemeinen Datenzugang über WebSocket. Verwende dessen Standardprotokoll. Baue keinen eigenen Transport für jeden Sensor und keinen zweiten UART-Parser.

Prüfe die passende rosbridge-Version und deren Verhalten anhand der tatsächlich installierten ROS-Distribution. Unterstelle nicht, dass der CM5 Jazzy verwendet. Die ROS-Umgebung und der Workspace mit den benutzerdefinierten Nachrichten müssen vor dem Start der Brücke aktiviert sein.

Offizielle Referenzen:

- https://github.com/RobotWebTools/rosbridge_suite/tree/ros2
- https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md
- https://github.com/RobotWebTools/roslibjs
- https://github.com/RobotWebTools/roslibpy

Ergänze eine kleine statische Browser-Anzeige. Der CM5 soll die Seite bereitstellen. Die Seite empfängt Nachrichten über rosbridge. Halte Abhängigkeiten klein und verwende lokal bereitgestellte, festgelegte Versionen, damit der Betrieb im LAN keinen öffentlichen CDN benötigt. Ein großes Frontend-Framework oder ein Dashboard-Baukasten ist nicht erforderlich.

Die erste Anzeige umfasst:

1. Eine HMMD-Heatmap mit 16 Entfernungszellen auf der x-Achse, 20 Doppler-Bins auf der y-Achse, Bin-Indizes und Farbskala.
2. Eine Umschaltung zwischen Rohwerten und `log1p`-Darstellung.
3. Sensorstatus und klare Kennzeichnung ausbleibender Daten. Zeige den Verbindungszustand zum CM5 getrennt vom Empfang echter Sensorframes.
4. Eine einfache Auswahl freigegebener Topics mit einer Ansicht ihrer letzten Nachricht als lesbare Daten. Diese Ansicht macht den Zugang auch für spätere ROS-Sensordaten nutzbar. Zusätzliche spezielle Visualisierungen für LiDAR, Kameras oder Navigation sind nicht beauftragt.

Beschreibe außerdem, wie ein gewöhnliches Python-Programm mit `roslibpy` denselben Datenzugang abonnieren kann. Ein kurzes Empfangsbeispiel genügt. Eine eigene Recorder-Anwendung oder Exportverwaltung ist nicht erforderlich.

## Anforderungen an Daten und Netzwerk

Bewahre rohe Matrixwerte, Dimensionen und Zeitstempel. Verändere Daten nur für die Darstellung. Übernimm keine unbelegte physikalische Skalierung in Meter oder Geschwindigkeit.

Halte für die Live-Anzeige die neueste Nachricht und begrenze die Aktualisierung, zunächst auf höchstens 10 Hz. Vermeide einen Rückstau alter Frames. Prüfe die Best-Effort-Kompatibilität des rosbridge-Abonnements. Trenne eine maximal gewünschte Anzeigerate von der noch unbekannten tatsächlichen Sensor-Bildrate.

Eine unterbrochene Netzwerkverbindung, ein ausgefallener Sensorknoten und ein geöffneter UART ohne Frames müssen als unterschiedliche Zustände erkennbar sein. Wiederholte alte Daten dürfen nicht frisch erscheinen. Die Frischeprüfung darf keine synchronisierten PC- und CM5-Uhren voraussetzen. Der Browser soll sich nach Verbindungsunterbrechungen wieder verbinden und erneut abonnieren.

Plane den normalen Zugriff über das lokale Netzwerk. Verwende explizite Host- und Portangaben und dokumentiere die tatsächlich gewählte Konfiguration. Port 9090 für WebSocket und 8080 für die Webseite sind Vorschläge, keine bereits eingerichteten Endpunkte.

Der Auftrag ist Datenempfang. Ergänze keine Steuerbefehle, Topic-Publisher, Motorbedienung oder Service-Aufrufe für Aktoren im Client. Prüfe die Zugriffsbeschränkungen der installierten rosbridge-Version. Ein nur lesender Client macht den Server nicht automatisch nur lesbar. Leere Glob-Einstellungen dürfen nicht ungeprüft als Zugriffssperre behandelt werden. Wenn eine reine Empfangsschnittstelle mit der Version nicht erzwingbar ist, dokumentiere die Grenze und wähle einen begrenzten Zugang, etwa Loopback mit SSH-Tunnel, statt einen unbelegten Schutz zu behaupten.

## Implementierung und CM5-Einrichtung

Implementiere zunächst im vorhandenen Worktree und dokumentiere reproduzierbare Startbefehle für Sensor, Brücke und Webseite. Nutze bestehende Paket- und Startstrukturen, soweit sie den Auftrag tragen. Plane keine ROS-Neuinstallation auf dem CM5.

Prüfe über `ssh cm5` Betriebssystem, ROS-Distribution, installierte Pakete, dortigen Projektpfad und vorhandene Änderungen. Übertrage nur die für diesen Auftrag benötigten Dateien. Überschreibe keine fremden oder ungesicherten Änderungen. Installiere passende Abhängigkeiten und richte den dokumentierten Start ein. Bestätige den Empfang auf dem lokalen PC über das LAN. Ein Autostartdienst ist kein eigenes Abnahmekriterium.

Lies vor tatsächlichem Sensorbetrieb die einschlägige Hardwareübersicht und die HMMD-Anschlussanleitung. Stelle fehlende Versorgung, Pin-Funktionen oder echte Frames nicht aus einem geöffneten UART fest. Änderungen an Verdrahtung oder Bootkonfiguration sind nicht Bestandteil dieses Datenzugangsauftrags. Dokumentiere einen solchen Blocker, wenn der vorhandene Sensorknoten noch keine Daten erhält.

Aktualisiere die betroffene CM5-Anleitung und Referenz. Halte neue Hardwarebeobachtungen mit Evidenz in `agent_notes/` fest. Ergänze `document-release` nicht zum Workflow.

## Abnahme

- Ein PC ohne ROS-Installation empfängt über den allgemeinen Zugang eine ROS-Nachricht und zeigt die HMMD-Matrix im Browser.
- Ein weiteres freigegebenes Topic lässt sich mit demselben Zugang abonnieren, ohne einen neuen Transport zu implementieren. Eine Testquelle ist zulässig, wenn weitere echte Sensortopics fehlen.
- Die HMMD-Anzeige besitzt richtige Matrixdimensionen, Roh- und Log-Ansicht sowie sichtbare Frische- und Verbindungszustände.
- Ein Verbindungsabbruch und ein Stopp des Publishers werden angezeigt. Wiederanlauf liefert neue Daten ohne veralteten Rückstau.
- Ein gewöhnlicher Python-Client kann eine Nachricht abonnieren, ohne ROS oder generierte ROS-Nachrichtenpakete auf dem PC zu installieren.
- Die Anleitung nennt konkrete Startbefehle und die Browser-Adresse. Diese stimmen mit der tatsächlichen Einrichtung auf dem CM5 überein.

Prüfe den tatsächlichen Browser und den vollständigen Datenweg. Verwende synthetische Frames, falls echte Hardwaredaten fehlen, und kennzeichne diese Prüfung ausdrücklich. Behaupte Live-Sensorempfang nur bei beobachteten echten Frames. Führe gezielte Prüfungen für Fehlerfälle und den vollständigen Scope-Check vor Abschluss durch.

Berichte zum Abschluss knapp, was implementiert ist, welche Prüfung tatsächlich stattgefunden hat, welche CM5-Einrichtung erfolgt ist und welche Arbeit noch durch fehlenden Zugang oder fehlenden Sensorbetrieb blockiert ist.
