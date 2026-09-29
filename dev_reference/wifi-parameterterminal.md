# WLAN-Provisionierung und Parameterterminal

Der WLAN-Tuning-Build enthält keine WLAN-Zugangsdaten und keinen Tuning-Token. Ohne gespeicherte WLAN-Daten startet der ESP32 einen offenen SoftAP für Espressif-Provisioning. Nach erfolgreicher Einrichtung verwendet er die in NVS gespeicherten WLAN-Daten automatisch bei weiteren Neustarts.

Der SoftAP und das Provisioning verwenden Security 0 ohne Kopplung. Das ist für die Entwicklung vorgesehen. Während der kurzen Einrichtung kann ein Gerät in Funkreichweite Provisionierungsdaten übertragen oder verändern. Die Firmware gibt keine SSID, kein Passwort und keinen Token über Serial aus; der Build erzwingt außerdem Core Debug Level `None`, da `WiFiProv` bei aktivierten Debug-Ausgaben sensible Provisionierungswerte ausgeben kann.

## Firmware bauen

Im Sketchverzeichnis `src/ES-02/OllieFOCdrive` muss die lokale, ignorierte Datei `wifi_tuning_build.h` den WLAN-Tuning-Build mit `WIFI_TUNING_ENABLE 1` einschalten. Diese Datei enthält nur den Funktionsschalter. WLAN-Zugangsdaten werden nicht zum Kompilieren benötigt.

Aus dem Repository-Ordner kompilieren:

```bash
arduino-cli compile --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,DebugLevel=none,PartitionScheme=no_fs' --output-dir build/wifi-provisioning src/ES-02/OllieFOCdrive
```

Das Schema `no_fs` stellt zwei 2-MB-App-Partitionen bereit und enthält kein Dateisystem. Es passt zum aktuellen Programm, das 1,36 MB belegt. Die ESP32-S3-Einstellungen verwenden die vorhandene USB-Seriell-Verbindung und schalten Core-Debug-Ausgaben explizit aus.

## WLAN unter Linux provisionieren

Verbinde dich mit dem seriellen Monitor mit 115200 Baud. Beim ersten Start gibt die Firmware den Namen des offenen Provisioning-SoftAPs aus, zum Beispiel `PROV_XXXXXX`. Verbinde das Linux-WLAN zuerst mit genau diesem Access Point.

Espressif stellt `esp_prov` im Repository `idf-extra-components` bereit. Klone es und wechsle in das Werkzeugverzeichnis:

```bash
python3 -m venv .venv
. .venv/bin/activate
python3 -m pip install bleak protobuf cryptography
git clone --depth 1 https://github.com/espressif/idf-extra-components.git
cd idf-extra-components/network_provisioning/tool/esp_prov
python3 esp_prov.py --transport softap --sec_ver 0
```

Das Werkzeug verbindet sich mit dem ESP32 unter `192.168.4.1`, sucht erreichbare WLAN-Netze und fragt interaktiv nach dem gewünschten Netz und dessen Passwort. Gib WLAN-Daten nur in diese Prompts ein. Übernimm sie nicht in Kommandozeilenargumente, Shell-Skripte oder Logs. Espressif dokumentiert interaktive Provisionierung und SoftAP im [`wifi_prov`-Beispiel](https://components.espressif.com/components/espressif/network_provisioning/versions/1.2.0/examples/wifi_prov?language=en); der Quellcode des Werkzeugs liegt im [Espressif-Repository](https://github.com/espressif/idf-extra-components/tree/master/network_provisioning/tool/esp_prov).

Nach erfolgreicher Einrichtung meldet Serial nur `Wi-Fi connected`. Die Zugangsdaten bleiben im WLAN-NVS des ESP32 und werden beim nächsten normalen Neustart wiederverwendet. Sie werden nicht in die Firmware kompiliert.

## Provisionierung bewusst zurücksetzen

Sende über den seriellen Commander den Befehl `WRESET`. Die Firmware löscht nur die gespeicherte WLAN-Konfiguration und startet neu. Danach öffnet sie erneut den Provisioning-SoftAP. Das ist auch der Wiederherstellungsweg, falls versehentlich falsche WLAN-Daten provisioniert wurden. Kalibrierungswerte und andere Preferences bleiben erhalten.

## Parameterterminal und Tuning-Token

Das HTTP-Parameterterminal startet nur, wenn ein Laufzeit-Token in NVS vorhanden ist. Es liest dafür den Eintrag `token` aus dem Preferences-Namespace `wifi-tuning`. Ein fehlender oder zu kurzer Token lässt das Terminal geschlossen. Der Token wird nicht als Compilerdefinition oder Quelltextwert eingebunden.

Ein eigener Provisioning-Endpunkt für den Token ist in dieser Änderung noch nicht enthalten. Der nächste Schritt ist, beim Provisioning einen benannten Custom-Endpoint für den Token zu registrieren und dessen Wert in `wifi-tuning/token` abzulegen. Der Linux-Client kann danach den im NVS gespeicherten Laufzeit-Token verwenden. Bis dahin bleiben die HTTP-Endpunkte deaktiviert.

## API und Grenzen

Wenn ein Laufzeit-Token in NVS vorhanden ist, verlangt jede Anfrage den Bearer-Token. Das HTTP-API nutzt Port 80 ohne TLS und gehört ausschließlich in ein vertrauenswürdiges Entwicklungsnetz. Das API kann nur die freigegebenen Reglerparameter lesen und bei frischem SBUS sowie CH5 OFF schreiben. Parameteränderungen gelten bis zum nächsten Neustart.

Die Zugangsdaten werden im ESP32-NVS gespeichert. Diese Änderung aktiviert keine Flash-Verschlüsselung. Sie verhindert, dass WLAN-Zugangsdaten oder der Tuning-Token in Sourcecode, Compilerargumenten oder dem Firmware-Binary liegen.
