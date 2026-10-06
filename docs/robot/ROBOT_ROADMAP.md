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
