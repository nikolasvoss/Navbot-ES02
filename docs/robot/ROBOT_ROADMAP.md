# Roadmap

## Sensordaten und Filter verifizieren:

Für den Navbot ES02 würde ich die IMU- und Reglerdaten direkt auf dem ESP32 erfassen, roh und gefiltert mit Zeitstempel über **native USB-CDC** an den PC senden und dort z. B. mit Serial Studio live plotten, als CSV aufzeichnen und per FFT auf Rauschen oder Resonanzen untersuchen. Die Filterung sollte digital auf dem ESP32 bzw. teilweise im ICM42688 erfolgen, bevorzugt mit leichtem Gyro-Filter, stärkerem Accelerometer-Filter und Complementary-/Sensor-Fusion; für hohe Datenraten binär und gepuffert übertragen statt vieler `Serial.print()`-Aufrufe – ein DAC ist dafür normalerweise nicht nötig.
