# Roadmap

## Sensordaten und Filter verifizieren

Der ESP32 erfasst IMU-Daten mit monotonen Zeitstempeln. Im sicheren Diagnosemodus bleiben die Motortreiber deaktiviert; die Firmware sendet IMU-, Batterie- und SBUS-Werte als CSV-Zeilen über die vorhandene CH340-UART-Verbindung. SerialPlot übernimmt die Live-Darstellung und kann CSV-Snapshots speichern. Sein ASCII-Reader liest unpräfixierte, kommagetrennte Zahlenwerte; der Präfixfilter bleibt deaktiviert. Die gespeicherten CSV-Daten können anschließend mit einem FFT-Werkzeug ausgewertet werden; SerialPlot selbst bietet laut Funktionsliste keine FFT-Plots. Der Diagnosemodus berechnet keine Reglerausgänge. Die Bewertung von Filterung und Resonanzen erfolgt anhand der erfassten Zeitreihen; Regelzyklus und Verhalten unter Motorlast bleiben gesondert zu messen.

Die Filterung läuft digital auf dem ESP32 und teilweise im ICM42688. Gyro- und Accelerometer-Filter sowie Complementary- oder Sensorfusion werden anhand der Aufzeichnungen bewertet. Die konkreten Filterwerte und Abnahmekriterien werden später festgelegt. Für hohe Datenraten wird die Übertragung gepuffert. Binäre Frames kommen zum Einsatz, falls Messungen zeigen, dass CSV über den verfügbaren seriellen Anschluss die Zielrate nicht zuverlässig erreicht. Ein DAC ist dafür normalerweise nicht nötig.

## Nächste Schritte

1. Grenzwerte für Sollrate, Zeitabweichung, Datenverluste und zulässige Verzögerung des Regelzyklus festlegen. Vor Messungen im Regelbetrieb ein sicheres Testverfahren bestimmen. Der aktuelle Diagnosemodus hält die Motortreiber deaktiviert und berechnet keine Reglerausgänge.
2. Mit längeren CSV-Aufzeichnungen tatsächliche Rate, Zeitabstände und Datenlücken gegen diese Grenzwerte prüfen. Den Einfluss auf den Regelzyklus erst mit dem festgelegten Testverfahren messen.
3. Entscheiden, ob Reglerausgänge aufgezeichnet werden müssen. Dafür ist ein Verfahren nötig, das die Motortreiber deaktiviert lässt oder einen ausdrücklich sicheren Test im Regelbetrieb erlaubt.
4. Ungefilterte und gefilterte IMU-Werte sowie Sensorfusion anhand wiederholbarer Aufzeichnungen und Offline-FFT vergleichen. Danach Filterwerte und Abnahmekriterien festlegen.
5. Natives USB-CDC auf dem vorhandenen ESP32-S3-Board prüfen. Am USB-C-Anschluss wurde bisher nur der CH340-USB-UART beobachtet. Das Datenformat bleibt vom Transport getrennt.
6. Pufferung oder binäre Frames erst einführen, wenn Messungen zeigen, dass CSV die festgelegte Datenrate nicht zuverlässig erreicht.
7. Später ein Werkzeug prüfen, das aus CSV-Aufzeichnungen die FFT eines ausgewählten Messkanals anzeigt. Eine Live-FFT würde einen eigenen seriellen Client benötigen, da SerialPlot keine FFT-Plots bietet.
8. Logging-Kommandos bereinigen. Auswahl von Messwerten, Ausgabeformate und Verhalten im Diagnose- und Normalmodus eindeutig dokumentieren.

## Regler-Startwerte und Diagnose-Schalter bereinigen

- `DIAGNOSTIC_LIVE_TUNING_DEFAULTS` bündelt Verhalten für die normale Fahrt: Gain-Auswahl und Live-Tuning, CH3-Skalierung sowie die Zeitbasis der Raddrehzahl. Bei der späteren Bereinigung diese Zuständigkeiten explizit machen. Bestehendes Fahrverhalten beibehalten, bis Vergleichsmessungen Änderungen stützen. Dieser Schalter ist unabhängig von `SENSOR_DIAGNOSTIC_MODE`.

## IMU-Diagnosemessungen

- Nach der Entkopplung der IMU-Verarbeitung von der 100-Hz-CSV-Ausgabe messen: `IMUtime_dt`-Verteilung im normalen und sensor-diagnostischen Modus; Zeitstempelabstände, Sequenzlücken und Logger-Drop-/Schreibfehler im Diagnose-CSV; sowie Filterantwort bei stationären und kontrollierten bekannten Frequenzen. Für Frequenzmessungen eine zur Zielbandbreite passende Erfassungsrate und Anti-Alias-Filterung verwenden. Die aktuelle Diagnose bleibt motorfrei und belegt kein Verhalten unter Motorlast.

Diese Messungen sind wichtig, weil unterschiedliche Aktualisierungsraten und Filtereinstellungen Glättung, Verzögerung und Fusionswerte verändern können. Die 100-Hz-CSV-Ausgabe kann rohe Signale über 50 Hz aliasen, sodass schnelle Vibrationen falsch erscheinen oder unsichtbar bleiben. Ohne den Vergleich belegen Diagnoseaufzeichnungen weder die normale IMU-Verarbeitung noch das Verhalten unter Motorlast.

### Software-IMU-Grenzfrequenz (50 Hz)

Die tatsächliche IMU-Aktualisierungsrate im normalen Regelbetrieb messen und die aktuelle Software-Grenzfrequenz von 50 Hz mit passenden Alternativen vergleichen. Roh- und gefilterte Gyro- und Beschleunigungsdaten bei kontrollierten Frequenzen aufzeichnen und Dämpfung sowie Verzögerung der Lagefusion bewerten. Die Grenzfrequenz anhand des gemessenen Bewegungs- und Störspektrums festlegen, nicht allein anhand der nominalen Sensor-Abtastrate.

## Fahrstabilität bei hoher Geschwindigkeit

Beim schnellen Fahren kippt der Roboter weiterhin um. Das Fehlerbild in einem sicheren, reproduzierbaren Fahrtest mit Rollwinkel, Soll- und Ist-Raddrehzahl, effektivem Geschwindigkeitssollwert und Balance-Reglerausgang erfassen. Prüfen, ob die vorhandene Neigungs-Reduktion im beobachteten Fall greift und früh genug einsetzt. Schwellenwerte und Reglerabstimmung erst nach dieser Auswertung ändern.

## Lidar bei einem Sturz abschalten

Den M-Pin des Lidars (Lidar Enable) voraussichtlich zum Roboter-PCB führen und so ansteuern, dass das Lidar bei einem Sturz abgeschaltet wird.

## Kommunikation zwischen ROS2-PC und ESP32

UART als Kommunikationsschnittstelle zwischen ROS2-PC und ESP32 vorsehen, da auf dem Roboter-PCB bereits ein Stecker am ESP32 vorhanden ist.

Stand: Protokoll (COBS, CRC-16, Protobuf) und Firmware-Sender für `WheelOdometry` und `Imu` mit 50 Hz sind umgesetzt. Die ROS-2-Brücke `navbot_link_bridge` ist umgesetzt und wurde auf dem ros2_pc nur mit simulierten Frames über ein Pseudo-Terminal geprüft. Details: [UART-Protokoll](software/interfaces/ros2-uart-protocol.md).

### Offene Punkte vor dem Merge nach `main`

- [x] `feature/ros2-odometry-bridge` in `feature/esp32-ros2-uart-interface` mergen.
- [ ] UART des ros2_pc für die ESP32-Verbindung festlegen, Pins und 3,3-V-Pegel am Board prüfen und verdrahten. Danach den Platzhalter für `port` in der [ros2_pc-Softwareanleitung](../ros2_pc/software/README.md#navbot-link-odometry-bridge) ersetzen.
- [ ] Firmware flashen und die 500-kBd-Verbindung am echten Roboter prüfen: `/odom` und `/imu/data` mit 50 Hz, keine `bad_frames` oder `lost_frames` im Log der Brücke.
- [ ] Radradius und Spurbreite messen und als Parameter `wheel_radius_m` und `wheel_separation_m` dokumentieren. Die Brücke hat dafür keine Standardwerte.
- [ ] Vorzeichen prüfen: Roboter von Hand vorwärts rollen. Positive Radwinkel müssen Vorwärtsfahrt bedeuten, und links und rechts müssen stimmen. Die Firmware übernimmt die Vorzeichen ungeprüft aus der Raddrehzahl des Reglers (links +Sensor 1, rechts −Sensor 2).
- [ ] Statische TF `base_link` → `imu_link` aus der Einbaulage der IMU festlegen. `/imu/data` liefert Werte im Sensor-Koordinatensystem.
- [ ] SLAM-Integration mit `feat/slam` abstimmen: Dort ist `odom` statisch an `lds08rr` gebunden. Mit der Brücke wird daraus eine statische TF `base_link` → `lds08rr`, und die statische TF `odom` → `lds08rr` entfällt.
