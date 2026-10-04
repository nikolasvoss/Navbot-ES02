# HMMD-Heatmap auf dem CM5 starten und wiedergeben

Verwende den stationären HMMD-Aufbau für [Roadmap-Punkt 1](../roadmap.md). ROS 2 ist auf dem CM5 bereits installiert. Prüfe vor dem Anschluss das Modul, das Trägerboard, die Versorgung und die UART-Signalpegel anhand der [Sensornotizen](../../../../agent_notes/cm5/sensors/hmmd/README.md).

## Baue die Pakete

1. Aktiviere auf dem CM5 die vorhandene ROS-2-Umgebung. Prüfe `echo "$ROS_DISTRO"`, `ros2 --help` und `colcon --help`. Verwende den Python-Interpreter dieser ROS-Installation ab Version 3.10.
2. Wechsle in den Projektcheckout mit dem Branch `codex/hmmd-ros2-rdmap`.
3. Installiere die Paketabhängigkeiten über das vorhandene rosdep.

```bash
rosdep install --from-paths src/cm5/ros2/src --ignore-src -r -y
```

4. Baue und aktiviere den Workspace.

```bash
colcon --log-base src/cm5/ros2/log build --base-paths src/cm5/ros2/src --build-base src/cm5/ros2/build --install-base src/cm5/ros2/install
source src/cm5/ros2/install/setup.bash
```

5. Prüfe Parser, Verbindungsablauf und Anzeigenmodell.

```bash
PYTHONPATH=src/cm5/ros2/src/hmmd_radar python3 -m unittest discover -s src/cm5/ros2/src/hmmd_radar/test -v
```

Diese Tests verwenden synthetische Frames und einen nachgebildeten seriellen Anschluss. Sie ersetzen keine Live-Prüfung am Sensor. Die grafischen Tests benötigen Matplotlib.

## Starte den Sensor

1. Ermittle den tatsächlich zum HMMD gehörenden Port. Verwechsle den USB-Adapter der ESP32-Fahrsteuerung nicht mit dem Radar. Verwende nach Möglichkeit einen stabilen Pfad unter `/dev/serial/by-id/`.
2. Prüfe den freien Port und die Zugriffsrechte. Schließe andere Leser dieses Ports. Bei einer direkt benutzten CM5-UART muss die serielle Konsole dort deaktiviert sein.
3. Starte den Knoten mit dem geprüften Pfad. Ersetze den Beispielpfad durch den tatsächlichen HMMD-Port.

```bash
ros2 run hmmd_radar hmmd_sensor --ros-args -p port:=/dev/serial/by-id/HMMD_ADAPTER
```

Der Port muss ausdrücklich angegeben werden. Der Knoten sendet beim Öffnen den dokumentierten Befehl für den Debug-Modus. Er schreibt keine persistenten Sensorparameter. Das Verhalten des vorhandenen Moduls ist noch nicht live bestätigt.

4. Prüfe in einem zweiten Terminal mit aktivierter Workspace-Umgebung die Daten und den Status.

```bash
ros2 topic info /hmmd/rdmap --verbose
ros2 topic hz /hmmd/rdmap
ros2 topic echo /hmmd/status
```

Eine Meldung über das Öffnen des Ports beweist noch keine Sensordaten. Prüfe vollständige Frames, Matrixdimensionen und den Status bei fehlenden Daten.

## Zeige die Heatmap

1. Starte die Anzeige in einem Terminal mit grafischer Sitzung und aktivierter Workspace-Umgebung.

```bash
ros2 run hmmd_radar hmmd_heatmap
```

2. Drücke im Heatmap-Fenster `l`, um zwischen Rohwerten und logarithmischer Ansicht umzuschalten.
3. Prüfe 16 Entfernungszellen auf der x-Achse und 20 Doppler-Bins auf der y-Achse. Die Beschriftung verwendet Indizes. Physikalische Skalierung und Orientierung sind noch nicht bestätigt.
4. Unterbrich den Sensorprozess oder die Verbindung. Prüfe, dass die Anzeige ausbleibende Daten sichtbar kennzeichnet.
5. Starte den Empfang erneut. Prüfe, dass ein neuer Frame die Kennzeichnung für veraltete Daten aufhebt.

Starte die Heatmap auf einem Rechner mit grafischem Display und Zugriff auf dieselbe ROS-Domäne, wenn der CM5 ohne Display läuft. Dort müssen der Nachrichtentyp und das Anzeigepaket ebenfalls gebaut und aktiviert sein.

## Zeichne die drei Situationen auf

1. Starte Sensor und Heatmap.
2. Zeichne den leeren Messbereich auf.

```bash
ros2 bag record -o hmmd-empty /hmmd/rdmap /hmmd/status
```

3. Beende die Aufnahme mit `Ctrl+C`.
4. Wiederhole die Aufnahme mit einer ruhenden Person als `hmmd-stationary` und einer bewegten Person als `hmmd-moving`.
5. Notiere für jede Aufnahme Aufbau, Position, Dauer, beobachtete Datenrate und Auffälligkeiten in [observations.md](../../../../agent_notes/cm5/sensors/hmmd/observations.md).

Die Frame-Veröffentlichung verwendet Best-Effort-QoS. Prüfe nach jeder Aufnahme mit `ros2 bag info`, dass die Matrixnachrichten enthalten sind. Verwende die [QoS-Referenz](../reference/hmmd-ros2.md), falls die verwendete rosbag2-Version die QoS nicht automatisch passend erkennt.

## Spiele eine Aufnahme ohne Sensor ab

1. Beende den Sensorknoten, damit er keine Live-Daten parallel veröffentlicht.
2. Starte die Heatmap mit der aktiven Workspace-Umgebung.
3. Spiele eine Aufnahme ab.

```bash
ros2 bag info hmmd-empty
ros2 bag play hmmd-empty
```

Die Heatmap benutzt die lokale Zeit seit dem letzten empfangenen Frame für die Kennzeichnung veralteter Daten. Sie benötigt keine Simulation der ROS-Zeit für die reine Anzeige. Eine Wiedergabe mit `--clock` darf die Originalzeitstempel für andere Verbraucher verfügbar machen.

4. Pausiere die Wiedergabe und prüfe die Kennzeichnung veralteter Daten.
5. Prüfe die gleiche Kennzeichnung am Dateiende.
6. Spiele die beiden anderen Aufnahmen ab und vergleiche die Radarantwort.

## Prüfe den aktuellen Implementierungsstand

Der Parser, das Anzeigenmodell und die Paketquellen können lokal geprüft werden. Ein ROS-2-Build, der echte Sensorempfang und die rosbag2-Aufnahme/Wiedergabe auf dem CM5 müssen mit erreichbarer Hardware separat bestätigt werden. Der Roadmap-Meilenstein ist erst nach den dort genannten Live-Kriterien abgeschlossen.
