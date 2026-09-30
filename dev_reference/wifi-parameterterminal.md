# WLAN-Provisionierung und Parameterterminal

Der WLAN-Tuning-Build enthält keine WLAN-Zugangsdaten. Ohne gespeicherte WLAN-Daten startet der ESP32 einen offenen SoftAP für Espressif-Provisioning. Nach erfolgreicher Einrichtung verwendet er die in NVS gespeicherten WLAN-Daten automatisch bei weiteren Neustarts.

Der SoftAP und das Provisioning verwenden Security 0 ohne Kopplung. Das ist für die Entwicklung vorgesehen. Während der kurzen Einrichtung kann ein Gerät in Funkreichweite Provisionierungsdaten übertragen oder verändern. Die Firmware gibt keine SSID oder kein Passwort über Serial aus; der Build erzwingt außerdem Core Debug Level `None`, da `WiFiProv` bei aktivierten Debug-Ausgaben sensible Provisionierungswerte ausgeben kann.

## Firmware bauen

Im Sketchverzeichnis `src/ES-02/OllieFOCdrive` muss die lokale, ignorierte Datei `wifi_tuning_build.h` `WIFI_TUNING_ENABLE 1` setzen. Wenn die Datei fehlt, lege sie mit diesem Inhalt an:

```cpp
#pragma once
#define WIFI_TUNING_ENABLE 1
```

Überschreibe keine vorhandene lokale Datei. `wifi_tuning_build.example.h` aktiviert zusätzlich die WLAN-Aufzeichnung. Um das WLAN-Interface beim nächsten Build vollständig abzuschalten, setze in `wifi_tuning_build.h` beide Makros auf `0` oder entferne die lokale Datei. Ohne diese Datei stehen beide Feature-Schalter standardmäßig auf `0`:

```cpp
#define WIFI_TUNING_ENABLE 0
#define WIFI_RECORDING_ENABLE 0
```

WLAN-Zugangsdaten werden nicht zum Kompilieren benötigt.

Aus dem Repository-Ordner kompilieren:

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,DebugLevel=none,PartitionScheme=no_fs' --output-dir build/wifi-provisioning src/ES-02/OllieFOCdrive
```

Das Schema `no_fs` stellt zwei 2-MB-App-Partitionen bereit und enthält kein Dateisystem. Es passt zum aktuellen Programm, das 1,36 MB belegt. Die ESP32-S3-Einstellungen verwenden die vorhandene USB-Seriell-Verbindung und schalten Core-Debug-Ausgaben explizit aus.

## Firmware flashen

Schließe serielle Monitore und SerialPlot. Prüfe vor jedem Flash den CH340-Port mit `arduino-cli board list`. Arduino CLI prüft beim Verbindungsaufbau, ob der Chip ein ESP32-S3 ist. Die Upload-Hilfe prüft den CH340 und verifiziert den Flash nach dem Upload. Lies die [USB- und seriellen Hinweise](../agent_notes/usb-serial.md), bevor du flashst.

Führe im Repository-Ordner aus:

```bash
python3 scripts/upload_firmware.py build/wifi-provisioning
```

Wenn mehrere CH340-Adapter angeschlossen sind, gib den zuvor geprüften Port explizit an:

```bash
python3 scripts/upload_firmware.py build/wifi-provisioning --port /dev/ttyUSB0
```

Ersetze `/dev/ttyUSB0` durch den von `arduino-cli board list` angezeigten Port. Die Hilfe kompiliert nicht; führe den Compile-Schritt nach jeder Quellcode- oder Board-Einstellungsänderung erneut aus.

## WLAN unter Linux provisionieren

Verbinde dich mit dem seriellen Monitor mit 115200 Baud. Beim ersten Start gibt die Firmware den Namen des offenen Provisioning-SoftAPs aus, zum Beispiel `PROV_XXXXXX`. Notiere den Namen. Lade den Client und seine Abhängigkeiten herunter, bevor du das Linux-WLAN mit dem Roboter verbindest.

Lege alle heruntergeladenen Quellen und die Python-Umgebung in einem Ordner ab. Du kannst den gesamten Ordner später entfernen, um diese Dateien und Pakete zu löschen:

```bash
export PROV_HOME="$HOME/Dokumente/Bastelei/roboter/wifi-provisioning"
mkdir -p "$PROV_HOME"
git clone --depth 1 --branch v5.5.5 https://github.com/espressif/esp-idf.git "$PROV_HOME/esp-idf-v5.5.5"
git clone --depth 1 https://github.com/espressif/idf-extra-components.git "$PROV_HOME/idf-extra-components"
python3 -m venv "$PROV_HOME/.venv"
source "$PROV_HOME/.venv/bin/activate"
python3 -m pip install --no-cache-dir bleak protobuf cryptography
export IDF_PATH="$PROV_HOME/esp-idf-v5.5.5"
test -f "$IDF_PATH/components/protocomm/python/constants_pb2.py"
```

Die installierte Arduino-ESP32-Version 3.3.11 basiert auf ESP-IDF 5.5.5. `esp_prov` liest die Protobuf-Dateien unter `$IDF_PATH/components/protocomm/python`. Wenn du die Befehle erneut ausführst und die Klonordner oder die virtuelle Umgebung bereits existieren, überspringe deren Erzeugung. Verwende dann die vorhandenen Pfade unter `PROV_HOME`.

Verbinde jetzt das Linux-WLAN mit dem zuvor notierten SoftAP. Aktiviere die virtuelle Umgebung und starte den Client:

```bash
export PROV_HOME="$HOME/Dokumente/Bastelei/roboter/wifi-provisioning"
source "$PROV_HOME/.venv/bin/activate"
export IDF_PATH="$PROV_HOME/esp-idf-v5.5.5"
python3 "$PROV_HOME/idf-extra-components/network_provisioning/tool/esp_prov/esp_prov.py" --transport softap --sec_ver 0
```

Das Werkzeug verbindet sich mit dem ESP32 unter `192.168.4.1`, sucht erreichbare WLAN-Netze und fragt interaktiv nach dem gewünschten Netz und dessen Passwort. Gib WLAN-Daten nur in diese Prompts ein. Übernimm sie nicht in Kommandozeilenargumente, Shell-Skripte oder Logs. Espressif dokumentiert interaktive Provisionierung und SoftAP im [`wifi_prov`-Beispiel](https://components.espressif.com/components/espressif/network_provisioning/versions/1.2.0/examples/wifi_prov?language=en); der Quellcode des Werkzeugs liegt im [Espressif-Repository](https://github.com/espressif/idf-extra-components/tree/master/network_provisioning/tool/esp_prov).

Nach erfolgreicher Einrichtung meldet Serial nur `Wi-Fi connected`. Die Zugangsdaten bleiben im WLAN-NVS des ESP32 und werden beim nächsten normalen Neustart wiederverwendet. Sie werden nicht in die Firmware kompiliert.

## Provisioning-Werkzeuge entfernen

Wenn die virtuelle Umgebung aktiv ist, beende sie mit `deactivate`. Entferne danach den gemeinsamen Ordner:

```bash
rm -r "$HOME/Dokumente/Bastelei/roboter/wifi-provisioning"
```

## Provisionierung bewusst zurücksetzen

Sende über den seriellen Commander den Befehl `WRESET`. Die Firmware löscht nur die gespeicherte WLAN-Konfiguration und startet neu. Danach öffnet sie erneut den Provisioning-SoftAP. Das ist auch der Wiederherstellungsweg, falls versehentlich falsche WLAN-Daten provisioniert wurden. Kalibrierungswerte und andere Preferences bleiben erhalten.

## Parameterterminal

Nach `Wi-Fi connected` hat der Roboter seine WLAN-Zugangsdaten gespeichert. Im Repository-Ordner startest du direkt den Client:

```bash
python3 scripts/wifi_tune.py
```

Beim ersten Start fragt der Client nach der IPv4-Adresse des Roboters und speichert sie in `~/.config/navbot/wifi-tuning.json`. Die Datei enthält nur `host` und erhält Modus `0600`. Bei weiteren Starts verbindet sich der Client ohne zusätzliche Zugangsdaten. Du kannst die Adresse auch direkt angeben:

```bash
python3 scripts/wifi_tune.py --host 192.168.1.20
```

Im Terminal liest `PP` den aktuellen Balance-P-Wert. `PP5` schreibt den Wert 5. `show` liest alle Parameter und `status` zeigt Firmware-, SBUS- und CH5-Status. `save DATEI` speichert ein Profil; `load DATEI` schreibt dessen Werte gemeinsam. `--command status --json` führt einen einzelnen Befehl aus und gibt JSON aus.

Gain-Schreibzugriffe brauchen den manuellen Tuning-Modus `U=1`, frische SBUS-Daten und CH5 OFF. Die Firmware prüft SBUS-Frische und CH5 bei jedem Schreibvorgang. Änderungen bleiben bis zum Neustart aktiv.

Bei `Connection refused` hat der Client keine HTTP-Verbindung aufgebaut. Prüfe zuerst, ob die IP noch zum Roboter gehört und ob der serielle Startlog `Wi-Fi connected.` sowie `Wi-Fi tuning API started.` meldet. Meldet die Firmware, dass die API deaktiviert ist, läuft ein Build ohne WLAN-Tuning. Das lokale Build-Override schaltet nur den nächsten Build frei; die Firmware muss anschließend neu gebaut und geflasht werden.

Die WLAN-HTTP-API hat keine Token- oder Passwortabfrage. Jeder Rechner, der den Roboter im WLAN erreicht, kann sie aufrufen. Schreibzugriffe unterliegen weiterhin der Parameter-Allowlist sowie den SBUS-/CH5-Bedingungen. Die Wi-Fi-Implementierung lässt sich mit `WIFI_TUNING_ENABLE 0` beim Bauen abschalten; ohne lokale Build-Override-Datei bleibt das Feature aus.

## API und Grenzen

Das HTTP-API nutzt Port 80 ohne TLS und hat keine Authentifizierung. Es ist als Entwicklungsinterface gedacht und sollte nur in einem Netz verwendet werden, dem du die verbundenen Geräte anvertraust. Das API kann nur die freigegebenen Reglerparameter lesen und bei frischem SBUS sowie CH5 OFF schreiben. Parameteränderungen gelten bis zum nächsten Neustart.
