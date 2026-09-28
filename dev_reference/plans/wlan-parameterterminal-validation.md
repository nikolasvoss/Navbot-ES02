# WLAN-Parameterterminal: Validierungsprotokoll

Stand: 28.09.2026. Geprüft wurde der isolierte Branch `wifi-parameterterminal` im Worktree `/tmp/Navbot-ES02-wifi-tuning`, auf Basis des Snapshots `03310d2`.

## Builds

Alle drei Builds verwenden `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`, Arduino CLI 1.5.1, ESP32-Core 3.0.7, SimpleFOC 2.3.4 und ArduinoJson 7.4.3. PSRAM-Einstellung blieb unverändert.

| Variante | Ergebnis | Flash | Globale Daten |
| --- | --- | ---: | ---: |
| Snapshot-Baseline | erfolgreich | 1,100,329 B (83 %) | 51,756 B (15 %) |
| Feature aus | erfolgreich | 1,101,601 B (84 %) | 52,196 B (15 %) |
| Feature an, nur Dummy-Credentials | erfolgreich | 1,073,117 B (81 %) | 50,832 B (15 %) |

Zusatzchecks: Feature-on ohne lokale Secrets bricht mit verständlichem Include-Fehler ab. Der unveränderte Beispiel-Token `REPLACE_ME` scheitert an der Mindestlänge von 32 Zeichen. Die letzten SSID-/Passwort-Längenprüfungen und der aktuelle Feature-on Build nach dem Replay-/Boot-ID-Korrekturfix sind enthalten. Keine dieser Prüfungen hat ein Gerät geflasht.

## Automatisierte und CLI-Prüfungen

- `python3 -m unittest discover -s scripts -p 'test_wifi_tune.py' -v`: 10 Tests bestanden.
- `python3 -m unittest discover -s scripts -p 'test_analyze_drive_trace.py' -v`: 3 Tests bestanden.
- `g++ -std=c++17 -Wall -Wextra -pedantic tests/tuning_parameters_test.cpp src/ES-02/OllieFOCdrive/TuningParameters.cpp -o /tmp/tuning_parameters_test && /tmp/tuning_parameters_test`: bestanden. Enthält Registry, Parser, Grenzwerte, doppelte JSON-Schlüssel, Batchregeln, SBUS-Frische bei 249/250/251 ms sowie Schreibschutzfälle.
- `python3 scripts/wifi_tune.py --help`: Exit 0. Ungültiger lokaler Einzelbefehl: Exit 2. REPL mit EOF und `quit`: Exit 0.
- Client-Transport-/Timeouttests verwenden einen Mock-HTTP-Transport. Ein lokaler TCP-Mockserver war in dieser Sandbox wegen verweigerter Socket-Erstellung nicht ausführbar.
- `git diff --check`: sauber.

## Unabhängiger Review und Sicherheitsbefund

Ein zweiter Agent prüfte den Feature-Diff unabhängig. Er meldete Queue-Antwort-Lebensdauer, Begrenzung des JSON-Scanners, Größenlimit vor Body-Allokation, Replay-Vergleich, Profil-Laden sowie einen Kontrollflussfehler, der Konflikt-/Boot-ID-Ablehnungen nicht vom Anwenden trennte. Die gemeldeten Punkte wurden behoben. Die Abschlussprüfung fand keine kritischen oder hohen Probleme. Der Hinweis zum vorhersagbaren Beispiel-Token wurde durch den absichtlich zu kurzen, compile-time abgewiesenen Platzhalter `REPLACE_ME` erledigt. Die Firmware hält nur die letzte Write-Anfrage im Replay-Cache; ein älterer Request-ID nachfolgend auf eine andere erfolgreiche Änderung kann erneut verarbeitet werden. Werte sind absolute Setzungen und jede erneute Anwendung durchläuft weiterhin Boot-ID-, SBUS- und CH5-Prüfungen.

Der HTTP-Dienst ist weiterhin unverschlüsseltes HTTP mit Bearer-Token und ist ausschließlich für ein vertrauenswürdiges privates WLAN gedacht. Keine Portfreigabe und kein Gastnetz verwenden. Die Sicherheitsprüfung belegt weder Vertraulichkeit auf dem Funkweg noch physische Sicherheit beliebiger Gains.

## Nicht ausgeführt

Es gab keinen HTTP-Laufzeittest gegen die ESP32-Firmware, keinen Funk-/Last-/Reglerjittertest, keine Boardprüfung und keinen Fahrtest. Hardwareabnahme gemäß [Testplan](wlan-parameterterminal-test-plan.md) bleibt vor dem ersten Einsatz erforderlich. Es wurde weder geflasht noch hochgeladen.
