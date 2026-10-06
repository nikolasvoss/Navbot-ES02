# Roadmap

## Sensordaten und Filter verifizieren

Der ESP32 erfasst IMU- und Reglerdaten mit monotonen Zeitstempeln. Die Aufzeichnung kann bis zu fünf gleichzeitig ausgewählte Werte enthalten, darunter rohe und gefilterte IMU-Werte sowie relevante Reglergrößen. Auf dem PC übernimmt Serial Studio die Live-Darstellung und CSV-Aufzeichnung. Dafür wird der Projektdatei-Modus verwendet: Eine Projektdatei legt die ausgewählten Datensätze, ihre Einheiten und die Widgets fest. Der Modus unterstützt FFT-Plots; dafür wird die FFT-Abtastrate auf den gemessenen Wert gesetzt. Quick Plot unterstützt keine FFT und stellt automatisch alle empfangenen Kanäle dar. So lassen sich ausgewählte Signale live vergleichen, als CSV speichern und auf Rauschen oder Resonanzen untersuchen.

Die Filterung läuft digital auf dem ESP32 und teilweise im ICM42688. Gyro- und Accelerometer-Filter sowie Complementary- oder Sensorfusion werden anhand der Aufzeichnungen bewertet. Die konkreten Filterwerte und Abnahmekriterien werden später festgelegt. Für hohe Datenraten wird die Übertragung gepuffert. Binäre Frames kommen zum Einsatz, falls Messungen zeigen, dass CSV über den verfügbaren seriellen Anschluss die Zielrate nicht zuverlässig erreicht. Ein DAC ist dafür normalerweise nicht nötig.

Später zu klären:

- Prüfen, ob natives USB-CDC auf dem vorhandenen ESP32-S3-Board funktioniert. Bisher wurde am USB-C-Anschluss nur der CH340-USB-UART beobachtet. Das Datenformat soll vom seriellen Transport getrennt bleiben, damit sich der Anschluss später bei Bedarf wechseln lässt.
- Bei der Erfassung tatsächliche Abtastrate, Zeitabstände, Datenverluste und Regelzyklus messen. Der Datenstrom darf die Regelung nicht unvertretbar verzögern oder verändern. Die zulässigen Grenzwerte werden vor der Abnahme festgelegt.
