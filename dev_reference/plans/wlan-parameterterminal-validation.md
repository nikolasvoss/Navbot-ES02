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

Der damalige tokenbasierte Stand verwendete unverschlüsseltes HTTP mit Bearer-Token. Dieser Stand wurde am 30.09.2026 auf Wunsch des Nutzers durch einen unauthentifizierten Entwicklungsdienst ersetzt; der aktuelle Status folgt am Dokumentende.

## Nicht ausgeführt

Es gab keinen HTTP-Laufzeittest gegen die ESP32-Firmware, keinen Funk-/Last-/Reglerjittertest, keine Boardprüfung und keinen Fahrtest. Hardwareabnahme gemäß [Testplan](wlan-parameterterminal-test-plan.md) bleibt vor dem ersten Einsatz erforderlich. Es wurde weder geflasht noch hochgeladen.

## Token-Authentifizierung entfernt, 30.09.2026

Auf Wunsch des Nutzers ist die Terminal-Bearer-Token-Funktion vollständig aus Firmware und Linux-Clients entfernt. Die HTTP-API startet nach WLAN-Verbindung und Requestqueue-Initialisierung ohne NVS-Token. `wifi_tune.py` und `wifi_record.py` senden keinen Authorization-Header und erwarten nur die Roboteradresse. Der Parameterclient fragt beim ersten Start nach der IP und speichert eine Hostkonfiguration mit Modus `0600`; vorhandene alte `token`-Felder werden ignoriert.

Die WLAN-Funktion bleibt opt-in. `WifiTuningConfig.h` setzt beide Makros standardmäßig auf `0`; zum Deaktivieren für einen normalen Firmwarebuild `WIFI_TUNING_ENABLE 0` und `WIFI_RECORDING_ENABLE 0` setzen oder die lokale ignorierte `wifi_tuning_build.h` entfernen. Die kurzlebige Recording-Stream-Ticket-Prüfung bleibt bestehen, weil sie den TCP-Datenstrom einer laufenden Aufnahme zuordnet und kein Terminal-Bearer-Token ist.

- `python3 -m unittest discover -s scripts -p 'test_*.py' -v`: 27 Tests bestanden, darunter 13 für `wifi_tune.py` und 6 für `wifi_record.py`.
- `python3 -m py_compile scripts/wifi_tune.py scripts/wifi_record.py scripts/test_wifi_tune.py scripts/test_wifi_record.py`: bestanden.
- `python3 scripts/wifi_tune.py --help`: Exit 0; keine Token- oder USB-Setup-Optionen.
- `git diff --check`: sauber.
- Feature-on Build mit Arduino CLI 1.5.1 und ESP32-Core 3.3.11: erfolgreich, 1,355,376 Byte Programm (66 %), 57,692 Byte globale Daten (17 %). Ausgabe: `/tmp/navbot-no-token-on-output`.
- Feature-off Build ohne lokale Build-Konfigurationsdatei: erfolgreich, 731,624 Byte Programm (36 %), 36,208 Byte globale Daten (11 %). Dafür wurde eine temporäre Sketch-Kopie unter `/tmp/navbot-wifi-off.6pHhiK` verwendet; die lokale ignorierte Konfiguration blieb unverändert.
- Sicherheits-/Diffreview: Keine API-Bearer-Prüfung oder Tokenablage mehr. Das bewusste Restrisiko ist der offene HTTP-Zugriff für erreichbare WLAN-Clients; der Nutzer akzeptiert das für Entwicklung. Parameter-Allowlist sowie SBUS-/CH5-Prüfungen bleiben bestehen. Im normalen Build muss das Feature ausgeschaltet sein.
- Kein Flash, Upload oder echter HTTP-/WLAN-/Reglerlaufzeittest.

## Hardware-Reproduktion und Stack-Fix, 30.09.2026

Der Feature-on Build `12c5901168e5bea1...` wurde auf dem ESP32-S3 getestet. Ein einzelner WLAN-Schreibrequest mit unverändertem `PP=5` lief in einen Timeout und löste diesen Panic aus:

```text
Guru Meditation Error: Core 0 panic'ed (Unhandled debug exception).
Debug exception reason: Stack canary watchpoint triggered (wifi-tuning)
```

Der Backtrace wurde mit genau diesem ELF dekodiert. Er führt durch `hasDuplicateJsonObjectKeys()` in `TuningParameters.cpp`, den HTTP-Handler und den `wifi-tuning`-Task. Der Scanner hielt zuvor `char keys[5][24][40]` auf dem Task-Stack. Das lokale Host-Compiler-Stackmaß der Funktion sank nach der Änderung von 4.944 auf 480 Byte. Der Scanner speichert jetzt Key-Offsets und -Längen; doppelte Schlüssel werden weiterhin abgelehnt. C++-Grenztests für 18 Parameter, doppelte Keys und Keylängen 39/40 bestehen.

Der korrigierte WLAN-Build kompiliert mit 1.355.360 Byte Programm und 57.692 Byte globalen Daten. Python-WLAN-Tests: 22 bestanden. Der korrigierte Build wurde noch nicht auf das Gerät geflasht und der Live-Schreibpfad deshalb noch nicht erneut geprüft. Die vorherige Beobachtung `PP=5` nach dem Reset gehört zum fehlerhaften Build und ersetzt keine Hardwareabnahme des Fixes.
