# WLAN-Parameterterminal für Navbot-ES02

Phase 1 bietet im vertrauenswürdigen Heimnetz ein authentifiziertes HTTP-API und ein Python-Terminal für Linux. Parameteränderungen gelten bis zum nächsten Neustart. Das API kann Werte auch während der Fahrt lesen; Änderungen werden nur im unterstützten Zweirad-Master-Betrieb, bei gültigem frischem SBUS und CH5 OFF übernommen. Ein erfolgreicher Ack bestätigt den gespeicherten Konfigurationswert, nicht dessen Wirkung auf einen aktiven Regler.

Die Firmware verwendet Bearer-Token über unverschlüsseltes HTTP. Nur in einem vertrauenswürdigen privaten Netz nutzen; keine Router-Portfreigabe und keinen Gastnetz-Zugriff einrichten. Das Gerät erstellt weder Access Point noch Provisionierungs- oder OTA-Dienst.

## WLAN-Firmware bauen

1. Im Sketchverzeichnis `src/ES-02/OllieFOCdrive` die Dateien `wifi_tuning_build.example.h` und `wifi_tuning_secrets.example.h` jeweils als `wifi_tuning_build.h` und `wifi_tuning_secrets.h` kopieren. Beide lokalen Dateien sind ignoriert und gehören nicht in Git.
2. `wifi_tuning_build.h` schaltet mit `WIFI_TUNING_ENABLE 1` nur diesen Build frei. Setze in `wifi_tuning_secrets.h` die echte SSID, das WLAN-Passwort und einen zufälligen Token mit mindestens 128 Bit, zum Beispiel generiert mit `python3 -c 'import secrets; print(secrets.token_hex(32))'`. Keine echten Zugangsdaten in Compileargumente oder Ausgaben kopieren.
3. Mit den dokumentierten ESP32-S3-Einstellungen kompilieren:

   ```bash
   arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default' --output-dir build/wifi-tuning src/ES-02/OllieFOCdrive
   ```

   PSRAM bleibt aus. WLAN startet asynchron als Station. Der normale Build ohne `wifi_tuning_build.h` enthält die bisherige BLE-Initialisierung; der WLAN-Tuning-Build initialisiert BLE nicht und verwendet weiter den SBUS-Fahrpfad. Ein fehlendes/falsches Heimnetz verhindert den Start nicht.

## Linux-Client einrichten

Lege `~/.config/navbot/wifi-tuning.json` an und setze die Datei auf Modus 0600. Inhalt:

```json
{
  "host": "192.168.1.50",
  "token": "DEIN_LOKALER_TOKEN"
}
```

Zum Beispiel:

```bash
mkdir -p ~/.config/navbot
chmod 700 ~/.config/navbot
chmod 600 ~/.config/navbot/wifi-tuning.json
```

Verwende die lokale IPv4-Adresse des Roboters. Der Client nutzt ausschließlich Python-Standardbibliotheken und sendet den Token im Authorization-Header, nie in einer URL oder einem CLI-Argument.

```text
python3 scripts/wifi_tune.py --host 192.168.1.50
navbot> status
navbot> U
navbot> U1
navbot> PP5
navbot> PD0.12
navbot> SP0.045
navbot> V0.2
navbot> PP
navbot> show
navbot> save profiles/stand.json
navbot> load profiles/stand.json
navbot> quit
```

Ein Parametername ohne Zahl liest den Wert. `help` zeigt den erlaubten Umfang. Einzelaufrufe funktionieren mit `--command PP` und `--command SP0.045`; `--json` gibt die Antwort als JSON aus. `--host` überschreibt die IP aus der Konfiguration, benötigt aber weiterhin deren Token. Exitcodes: 0 Erfolg, 2 lokaler Eingabe-/Profilfehler, 3 Transport oder unklarer Timeout-Ausgang, 4 Geräteablehnung.

Profile enthalten eine Version, optional einen Kommentar und Parameterwerte, niemals WLAN-Zugangsdaten oder Token. Speichern holt einen frischen Snapshot und überschreibt vorhandene Dateien nur mit `save --force DATEI`. Profile im manuellen Modus enthalten U=1 und Gains; beim Laden wird das gesamte Profil vorab geprüft und als ein Batch angewendet. Ein Timeout wird nicht automatisch wiederholt; bei unklarem Ausgang zuerst `show` oder `status` aufrufen.

## API und Grenzen

Das API lauscht auf lokalem IPv4/Port 80 unter `/api/v1/`. `GET /status`, `GET /parameters` und `POST /parameters` verlangen alle den Bearer-Token. Schreibbare Namen sind `PP/PI/PD/PL`, `SP/SI/SD/SL`, `YP/YI/YD/YL`, `RP/RI/RD/RL`, `U` und `V`. Es gibt keine Weiterleitung an SimpleFOC Commander. `L` bezeichnet das im vorhandenen Pfad verwendete Integral-Limit; Bedeutung und Skalierung hängen vom Modus ab. Eingabegrenzen begrenzen nur die technische Eingabe und belegen keine physische Sicherheit der Gain-Werte.

Der HTTP-Callback parst und authentifiziert Requests, während die Hauptschleife Snapshots erstellt und Schreibvorgänge ausführt. Queue, Body (2048 Byte), Batch (18 Namen), Wartedauer und Request-Gültigkeit sind begrenzt. Serverauthentifizierung schützt das Heimnetz nicht gegen Lauschen oder Manipulation auf dem Netzpfad, da TLS nicht aktiviert ist.

Vor einer ersten Fahrt sind die Hardwareprüfungen aus dem [Testplan](plans/wlan-parameterterminal-test-plan.md) durchzuführen. Dieses Implementierungsergebnis enthält keinen Funk- oder Fahrtest und wurde nicht auf ein Gerät geflasht.

Build-, Test- und unabhängige Review-Ergebnisse stehen im [Validierungsprotokoll](plans/wlan-parameterterminal-validation.md).
