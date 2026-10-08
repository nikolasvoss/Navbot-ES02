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

- `DIAGNOSTIC_LIVE_TUNING_DEFAULTS` bündelt Verhalten für die normale Fahrt: Gain-Auswahl und Live-Tuning, CH3-Skalierung sowie die Zeitbasis der Raddrehzahl. Bei der späteren Bereinigung diese Zuständigkeiten explizit machen und prüfen, ob der Gain-Preload beim Start noch nötig ist, um den kurzen CH5-Wechsel über Modus 1 abzufangen. Bestehendes Fahrverhalten beibehalten, bis Vergleichsmessungen Änderungen stützen. Dieser Schalter ist unabhängig von `SENSOR_DIAGNOSTIC_MODE`.

## IMU-Diagnosemessungen

- Nach der Entkopplung der IMU-Verarbeitung von der 100-Hz-CSV-Ausgabe messen: `IMUtime_dt`-Verteilung im normalen und sensor-diagnostischen Modus; Zeitstempelabstände, Sequenzlücken und Logger-Drop-/Schreibfehler im Diagnose-CSV; sowie Filterantwort bei stationären und kontrollierten bekannten Frequenzen. Für Frequenzmessungen eine zur Zielbandbreite passende Erfassungsrate und Anti-Alias-Filterung verwenden. Die aktuelle Diagnose bleibt motorfrei und belegt kein Verhalten unter Motorlast.

Diese Messungen sind wichtig, weil unterschiedliche Aktualisierungsraten und Filtereinstellungen Glättung, Verzögerung und Fusionswerte verändern können. Die 100-Hz-CSV-Ausgabe kann rohe Signale über 50 Hz aliasen, sodass schnelle Vibrationen falsch erscheinen oder unsichtbar bleiben. Ohne den Vergleich belegen Diagnoseaufzeichnungen weder die normale IMU-Verarbeitung noch das Verhalten unter Motorlast.
