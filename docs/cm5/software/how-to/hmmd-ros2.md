# HMMD-Heatmap auf dem CM5 starten und wiedergeben

Verwende den stationären HMMD-Aufbau für [Roadmap-Punkt 1](../roadmap.md). Führe die folgenden Terminalbefehle auf dem CM5 aus, lokal oder über SSH. ROS 2 ist dort bereits installiert.

## Prüfe den J8-Anschluss vor dem Einschalten

Der Hauptablauf verwendet die **physischen J8-Pins 8 und 10**, also GPIO14/15 und UART0. Prüfe diese Zuordnung am ausgeschalteten Aufbau anhand der Pin-1-Markierung. Die [CM5-IO-Board-Pinreferenz](../../hardware/compute-module-5-io-board.md) enthält die vollständige Belegung.

| HMMD-Modulanschluss | CM5 IO Board J8, physischer Pin | Funktion am CM5 |
| --- | --- | --- |
| 3V3 | 1 oder 17 | 3,3-V-Versorgung |
| GND | 6 | Masse |
| TX | 10 | GPIO15 / UART0_RX |
| RX | 8 | GPIO14 / UART0_TX |
| OT2 | Kein Anschluss für diesen Test | Wird vom Sensorknoten nicht verwendet |

Die [Waveshare-Modulunterlagen](../../../../agent_notes/cm5/sensors/hmmd/vendor/HMMD-waveshare-wiki.txt) nennen 3,0–3,6 V Versorgung und 0–3,3-V-UART-Pegel. Prüfe Modulvariante, tatsächliche Versorgung und GPIO_VREF des Trägerboards vor dem Einschalten. Bei 1,8-V-GPIO_VREF ist Pegelwandlung nötig. Verbinde das Modul nicht mit 5 V.

Falls du mit "GPIO8 und GPIO10" tatsächlich GPIO-Nummern meinst, entspricht das J8-Pin 24 und 19. Diese beiden bilden kein UART-TX/RX-Paar. UART3 verwendet GPIO8/TX auf J8-Pin 24 und GPIO9/RX auf J8-Pin 21. Für diese bestätigte alternative Belegung verwende unten `uart3-pi5`, `/dev/ttyAMA3` und bei der Pinprüfung `8,9`. GPIO10 ist dort CTS. Die folgenden Beispiele gelten für UART0 auf den physischen Pins 8/10.

## Finde und aktiviere den J8-UART

### 1. Zeige die vorhandenen Gerätedateien

```bash
cat /etc/os-release
uname -r
ls -l /dev/ttyAMA* /dev/serial* 2>/dev/null
```

Am J8 meldet sich der Sensor nicht selbst beim Betriebssystem an. Die Gerätedatei gehört zum UART des CM5 und existiert unabhängig davon, ob der HMMD angeschlossen ist. UART0 erscheint mit dem Raspberry-Pi-Kernel als `/dev/ttyAMA0`, UART3 als `/dev/ttyAMA3`. Ein fehlender Port bedeutet zunächst, dass dieser UART noch nicht bereitgestellt wird.

`/dev/ttyAMA10` gehört zum separaten Debug-UART. Auch `/dev/serial0` kann darauf zeigen. Prüfe einen vorhandenen Alias mit `readlink -f /dev/serial0`, statt ihn ungeprüft für J8 zu verwenden. `/dev/serial/by-id/` wird für USB-Serial-Geräte benutzt; für den direkten J8-Anschluss wird kein solcher HMMD-Eintrag erwartet. Diese Zuordnung folgt der [Raspberry-Pi-UART-Dokumentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts).

### 2. Aktiviere UART0 auf GPIO14/15

Führe diesen Schritt aus, wenn `/dev/ttyAMA0` fehlt oder die Pinprüfung in Schritt 4 eine falsche Funktion zeigt. Bei bereits korrekt eingerichtetem UART fahre mit der Konsolenprüfung in Schritt 3 fort. Prüfe vor einer Änderung, dass dein System die Raspberry-Pi-Bootkonfiguration und das CM5-Overlay bereitstellt.

```bash
ls -l /boot/firmware/config.txt /boot/firmware/overlays/uart0-pi5.dtbo
```

Falls eine Datei fehlt, ermittle zuerst anhand von `/etc/os-release` die Bootkonfiguration und Firmwarepakete dieses Systems. Lege keine leere `config.txt` an. Raspberry Pi OS und [Ubuntu für Raspberry Pi](https://ubuntu.com/hardware/docs/boards/how-to/special_hardware/rpi-config-txt/) verwenden üblicherweise `/boot/firmware/`; die weitere Anleitung setzt diese vorhandenen Dateien voraus.

Sichere die Konfiguration einmal vor der ersten Änderung und öffne sie.

```bash
sudo cp /boot/firmware/config.txt /boot/firmware/config.txt.before-hmmd
sudo nano /boot/firmware/config.txt
```

Ergänze am Ende, sofern diese Einstellung nicht schon in einem für den CM5 wirksamen Abschnitt steht:

```ini
[all]
dtoverlay=uart0-pi5
```

Das Overlay aktiviert UART0 auf GPIO14/15. Verwende für GPIO8/9 stattdessen `dtoverlay=uart3-pi5`; dieser Anschluss teilt Pins mit SPI0 und darf nicht gleichzeitig dafür belegt sein. Verwende für den HMMD keine `ctsrts`-Option. Die [offizielle Overlay-Referenz](https://github.com/raspberrypi/firmware/blob/master/boot/overlays/README) beschreibt beide CM5-Overlays. Speichere in Nano mit `Ctrl+O`, bestätige mit Enter und beende mit `Ctrl+X`.

### 3. Gib den UART für den Sensor frei und starte neu

```bash
cat /proc/cmdline
systemctl is-active serial-getty@ttyAMA0.service
```

Falls die Kernelzeile `console=ttyAMA0,...` enthält, entferne genau diesen Eintrag aus `/boot/firmware/cmdline.txt` mit `sudo nano /boot/firmware/cmdline.txt`. Bei `console=serial0,...` prüfe zuerst dessen Ziel wie in Schritt 1. Entferne den Eintrag nur, wenn er den verwendeten J8-UART bezeichnet. Erhalte die übrigen Einträge, insbesondere `root=...` und `console=tty1`, und die einzelne Zeile der Datei.

Falls der Dienst `active` meldet, beende und deaktiviere die dortige Login-Konsole:

```bash
sudo systemctl disable --now serial-getty@ttyAMA0.service
```

`inactive` oder ein nicht vorhandener Dienst bedeutet, dass diese Login-Konsole nicht läuft. Falls du die Bootkonfiguration oder Kernelzeile geändert hast, starte neu, damit die Änderungen wirksam werden. Ohne solche Änderungen fahre direkt mit Schritt 4 fort.

```bash
sudo reboot
```

Verbinde dich nach dem Neustart wieder per SSH oder öffne ein lokales Terminal.

### 4. Bestätige den Port und seine Pinbelegung

```bash
ls -l /dev/ttyAMA* /dev/serial* 2>/dev/null
export HMMD_PORT=/dev/ttyAMA0
ls -l "$HMMD_PORT"
sudo dmesg | grep -E 'ttyAMA|serial'
```

Für den beschriebenen UART0-Aufbau muss `/dev/ttyAMA0` nun existieren. Falls nur `/dev/ttyAMA10` erscheint, ist der J8-UART noch nicht aktiviert. Prüfe die gespeicherte Overlayzeile, ihren `[all]`-Abschnitt und die Meldungen beim Booten. Ein Abziehen oder Anstecken des Sensors ändert diese Portliste nicht.

Falls `pinctrl` installiert ist, bestätige zusätzlich die tatsächlich ausgewählten Pin-Funktionen:

```bash
sudo pinctrl get 14,15
```

Die Ausgabe muss für GPIO14 `TXD0` und für GPIO15 `RXD0` zeigen. `input`, `output` oder eine andere Busfunktion bedeutet, dass die Pins noch nicht wie erwartet für UART0 konfiguriert sind. Der [Hersteller beschreibt `pinctrl`](https://github.com/raspberrypi/utils/blob/master/pinctrl/README.md) als Werkzeug zur Anzeige der aktiven GPIO-Funktionen.

### 5. Prüfe Zugriff und andere Portbenutzer

```bash
ls -l "$HMMD_PORT"
id -nG
test -r "$HMMD_PORT" && test -w "$HMMD_PORT" && echo "UART lesbar und schreibbar"
sudo fuser -v "$HMMD_PORT"
```

Zeigt `ls -l` die Gruppe `dialout`, aber `id -nG` enthält sie nicht, füge deinen Benutzer hinzu:

```bash
sudo usermod -aG dialout "$USER"
```

Melde dich nach einer Gruppenänderung vollständig ab und erneut an, damit die Gruppenzugehörigkeit gilt. Setze danach `export HMMD_PORT=/dev/ttyAMA0` erneut und wiederhole die Zugriffsprüfung. Die Meldung `UART lesbar und schreibbar` bestätigt beide Rechte; bleibt sie aus, fehlen Rechte oder die Gerätedatei. `fuser` zeigt Prozesse an, die den Port bereits geöffnet haben. Beende solche Terminalprogramme oder Sensorprozesse, bevor du den Sensorknoten startest. Keine Prozessausgabe bedeutet, dass `fuser` keinen Benutzer dieses Ports gefunden hat.

## Baue die Pakete

1. Aktiviere auf dem CM5 die vorhandene ROS-2-Umgebung. Prüfe `echo "$ROS_DISTRO"`, `ros2 --help` und `colcon --help`. Verwende den Python-Interpreter dieser ROS-Installation ab Version 3.10.
2. Wechsle in den Projektcheckout. Beim CM5-Setup am 4. Oktober 2026 war das Repository `/home/niko/Navbot-ES02-cm5-hmmd`, Worktree-Branch `codex/hmmd-cm5-browser-setup` auf Basis von `f93ab3f34ee273914629e4e4c0d851866f819ae2`.
3. Installiere die Paketabhängigkeiten über das vorhandene rosdep.

```bash
rosdep install --from-paths src/cm5/ros2/src --ignore-src -r -y --skip-keys=ament_python
```

`rosdep` hat für `ament_python` keinen Systemschlüssel. Der Build-Typ kommt mit der vorhandenen ROS-2-Installation; die Paketabhängigkeiten werden weiterhin geprüft und installiert.

4. Baue und aktiviere den Workspace.

```bash
colcon --log-base src/cm5/ros2/log build --base-paths src/cm5/ros2/src --build-base src/cm5/ros2/build --install-base src/cm5/ros2/install
source src/cm5/ros2/install/setup.bash
```

Aktiviere in jedem neuen Terminal die ROS- und Workspace-Umgebung erneut. Wechsle dazu zuerst in das Projektcheckout:

```bash
cd ~/Navbot-ES02
source /opt/ros/$ROS_DISTRO/setup.bash
source src/cm5/ros2/install/setup.bash
```

Passe den `cd`-Pfad an, falls der Checkout an einem anderen Ort liegt. Ohne das Workspace-Setup meldet `ros2 run`, dass `hmmd_radar` nicht gefunden wurde.

5. Prüfe Parser, Verbindungsablauf und Anzeigenmodell.

```bash
PYTHONPATH="src/cm5/ros2/src/hmmd_radar${PYTHONPATH:+:$PYTHONPATH}" python3 -m unittest discover -s src/cm5/ros2/src/hmmd_radar/test -v
```

Die Ergänzung der vorhandenen `PYTHONPATH`-Variable erhält die ROS-Python-Module, damit auch der ROS-Graph-Test ausgeführt wird. Diese Tests verwenden synthetische Frames und einen nachgebildeten seriellen Anschluss. Sie ersetzen keine Live-Prüfung am Sensor. Die grafischen Tests benötigen Matplotlib.

## Starte den Sensor

1. Aktiviere nach dem Paketbau im selben Terminal die ROS-Umgebung und `source src/cm5/ros2/install/setup.bash`. Verwende den oben geprüften J8-Port. Setze die Variable nach einem neuen Login oder in einem neuen Terminal erneut. Für die alternative UART3-Belegung lautet der Pfad `/dev/ttyAMA3`.
2. Starte den Knoten mit 115200 Baud und dem geprüften Pfad.

```bash
export HMMD_PORT=/dev/ttyAMA0
ros2 run hmmd_radar hmmd_sensor --ros-args -p "port:=$HMMD_PORT" -p baud_rate:=115200
```

Der Port muss ausdrücklich angegeben werden. Der Knoten sendet beim Öffnen den dokumentierten Befehl für den Debug-Modus. Er schreibt keine persistenten Sensorparameter. Beim CM5-Setup wurden echte Frames beobachtet; eine ACK-Sequenz oder die Sensor-Firmwareversion wurde dabei nicht bestätigt. Siehe die [Beobachtungen mit Evidenz](../../../../agent_notes/cm5/sensors/hmmd/observations.md). Läuft bereits ein `hmmd_sensor`, starte keinen zweiten UART-Leser.

Alternativ starte den Knoten mit dem geprüften Pfad über das lokale [Startskript](../../../../scripts/start_hmmd_sensor.sh). Das Skript aktiviert ROS und den Workspace, prüft den Port und verwendet standardmäßig 115200 Baud:

```bash
HMMD_PORT=/dev/ttyAMA0 scripts/start_hmmd_sensor.sh
```

Der Gerätepfad kann auch als erstes Argument übergeben werden. Das Skript läuft im Vordergrund und wird mit `Ctrl+C` beendet. Starte es nur, wenn kein anderer Leser den Port verwendet.

Falls du statt des geprüften J8-UART einen USB-Adapter verwendest, ermittle dessen tatsächlichen Gerätepfad mit `ls -l /dev/serial/by-id/` und übergib genau diesen Pfad. Verwechsle den USB-Adapter der ESP32-Fahrsteuerung nicht mit dem Radar. Fehlt das Verzeichnis oder ist es leer, prüfe zuerst die Erkennung des HMMD-Adapters; rate keinen Gerätenamen. Prüfe Port und Zugriffsrechte. Verwende `/dev/ttyAMA*` oder `/dev/ttyS*` nur mit bestätigter Verbindung und Pinbelegung; bei einer direkt benutzten CM5-UART muss die serielle Konsole dort deaktiviert sein.

3. Prüfe in einem zweiten Terminal mit aktivierter ROS- und Workspace-Umgebung die Daten und den Status.

```bash
ros2 topic info /hmmd/rdmap --verbose
ros2 topic echo /hmmd/status
```

Beende die laufende Statusausgabe mit `Ctrl+C` und prüfe anschließend die Datenrate.

```bash
ros2 topic hz /hmmd/rdmap
```

Erfolgreicher Empfang zeigt `connected=true`, `stale=false`, einen steigenden Zähler `frames_received` und den Status `receiving frames`. Erst dann starte die Heatmap im nächsten Abschnitt. Der geöffnete Port allein bestätigt nur den Zugriff auf den CM5-UART.

| Beobachtung | Nächster Schritt |
| --- | --- |
| `last_io_error` enthält `No such file or directory` | Gerätedatei, Overlay und Neustart aus dem UART-Abschnitt prüfen. |
| `last_io_error` enthält `Permission denied` | Gruppe und Lese-/Schreibrechte prüfen; nach Gruppenänderung neu anmelden. |
| `connected=true`, aber `frames_received=0` und `no recent frames` | Versorgung, gemeinsame Masse, gekreuzte TX/RX-Leitungen, Pin-Funktionen und andere Portbenutzer prüfen. Der UART kann sich auch ohne angeschlossenen Sensor öffnen. |
| Frames kommen an, aber die zweite ROS-Sitzung sieht keine Topics | ROS- und Workspace-Umgebung sowie dieselbe `ROS_DOMAIN_ID` in beiden Terminals prüfen. |

## Zeige die Heatmap

1. Starte die Anzeige in einem Terminal mit grafischer Sitzung und aktivierter Workspace-Umgebung.

```bash
ros2 run hmmd_radar hmmd_heatmap
```

2. Drücke im Heatmap-Fenster `l`, um zwischen Rohwerten und logarithmischer Ansicht umzuschalten.
3. Prüfe 16 Entfernungszellen auf der x-Achse und 20 Doppler-Bins auf der y-Achse. Die Beschriftung verwendet Indizes. Physikalische Skalierung und Orientierung sind noch nicht bestätigt.
4. Unterbrich den Sensorprozess oder die Verbindung. Prüfe, dass die Anzeige ausbleibende Daten sichtbar kennzeichnet.
5. Starte den Empfang erneut. Prüfe, dass ein neuer Frame die Kennzeichnung für veraltete Daten aufhebt.

Starte die Heatmap auf einem Rechner mit grafischem Display und Zugriff auf dieselbe ROS-Domäne, wenn der CM5 ohne Display läuft. Dort müssen der Nachrichtentyp und das Anzeigepaket ebenfalls gebaut und aktiviert sein.

## Zeige die HMMD-Daten im PC-Browser

Die statische Browseransicht benötigt auf dem PC keine ROS-Installation und keine Nachrichtenpakete. Sie verbindet sich über das [Standard-JSON-Protokoll von rosbridge](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/ROSBRIDGE_PROTOCOL.md) mit `/hmmd/rdmap` und `/hmmd/status`. Der Browser-Client sendet nur `subscribe` und `unsubscribe`. rosbridge selbst stellt weitere Protokolloperationen bereit; verwende daher die Loopback-Bindung und den SSH-Tunnel als Zugangsgrenze.

1. Prüfe auf dem CM5 die installierte Jazzy- und rosbridge-Version sowie die tatsächlich verfügbaren Launch-Argumente. Beim eingerichteten CM5 sind Ubuntu 24.04.5 LTS, ROS 2 Jazzy und rosbridge-server/rosbridge-library 2.7.1 installiert.

```bash
echo "$ROS_DISTRO"
ros2 pkg prefix rosbridge_server
dpkg-query -W -f='${Version}\n' ros-jazzy-rosbridge-server
ros2 launch rosbridge_server rosbridge_websocket_launch.xml --show-args
```

Setze die ROS-Umgebung wie im Abschnitt „Baue die Pakete“ auf und source danach auch `src/cm5/ros2/install/setup.bash`. rosbridge 2.7.1 stellt `address`, `port`, `topics_glob`, `topics_pub_glob`, `topics_sub_glob`, `services_glob` und `params_glob` als Launch-Argumente bereit. Die Filterparameter sind ROS-Strings, keine ROS-Arrays; innere Anführungszeichen erhalten den Stringtyp. Mit ungequoteten Listen starteten weder Bridge noch rosapi. Verwende:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
  address:=127.0.0.1 port:=9090 \
  'topics_glob:=""' \
  'topics_pub_glob:="[]"' \
  'topics_sub_glob:="[/hmmd/rdmap,/hmmd/status]"' \
  'services_glob:="[]"' \
  'params_glob:="[]"'
```

Ein leerer Glob-String bedeutet in dieser Version keine Filterung. `topics_pub_glob:="[]"` verweigert Publish/Advertise, und `topics_sub_glob` beschränkt Abos auf die beiden HMMD-Topics. Bei `services_glob:="[]"` hängt der Server automatisch `/rosapi/*` an; `/rosapi/topics` wurde mit einer auf die freigegebenen HMMD-Topics beschränkten Liste beantwortet, während `/hmmd_sensor/get_parameters` abgewiesen wurde. ROS-API-Dienste unter `/rosapi/*` bleiben damit erreichbar. `actions_glob` existiert als Serverparameter, wird aber von der Launch-Datei nicht angeboten und bleibt bei diesem Befehl ungefiltert. Der rosbridge-Server ist damit keine nachgewiesene reine Empfangsschnittstelle. Der Loopback-Port darf nicht direkt im LAN geöffnet werden.

2. In einem zweiten CM5-Terminal oder einer SSH-Sitzung starte den statischen Dateiserver aus dem Projektcheckout:

```bash
cd /home/niko/Navbot-ES02-cm5-hmmd
python3 -m http.server 8080 --bind 0.0.0.0 --directory src/cm5/ros2/src/hmmd_radar/web
```

Öffne auf dem PC `http://192.168.178.28:8080/`. Diese WLAN-Adresse war beim Setup aktiv und kann sich durch DHCP ändern. Der HTTP-Server enthält nur statische Browserdateien. Für den rosbridge-Zugriff starte auf dem PC einen SSH-Tunnel, der das lokale Port 9090 an den CM5-Loopback-Port weiterleitet:

```bash
ssh -N -L 127.0.0.1:9090:127.0.0.1:9090 niko@192.168.178.28
```

Die Seite verbindet sich mit `ws://127.0.0.1:9090`. Halte den Tunnel und beide CM5-Prozesse geöffnet. Auf dem PC war der Alias `cm5` nicht auflösbar. Ermittle die aktuelle CM5-Adresse bei Bedarf mit `ip -br addr`.

Alternativ starte auf dem PC das Skript `scripts/start_hmmd_web.py` aus einem lokalen Checkout oder kopiere diese einzelne Datei auf den PC. Es benötigt Python 3 und OpenSSH, öffnet den Tunnel und anschließend den Standardbrowser. Die oben beschriebenen CM5-Prozesse müssen bereits laufen:

```bash
python3 scripts/start_hmmd_web.py
# Bei anderer CM5-Adresse oder anderem SSH-Benutzer:
python3 scripts/start_hmmd_web.py --host 192.168.178.28 --user niko
```

Unter Windows lautet der Python-Aufruf üblicherweise `py scripts/start_hmmd_web.py`. Lass das PC-Terminal geöffnet; `Strg+C` beendet den vom Skript gestarteten Tunnel. Ein bereits belegter lokaler Port 9090 wird gemeldet; beende dann den alten Tunnel oder öffne bei einem bereits laufenden CM5-Tunnel direkt die Browser-URL.

3. Prüfe den Verbindungsstatus getrennt vom HMMD-Status. „Connected“ bestätigt nur die Browser-Bridge-Verbindung. „Receiving sensor frames“ erfordert frische `/hmmd/status`-Daten mit `connected=true` und `stale=false`; die Heatmap wird erst mit einem gültigen 20×16-Frame frisch. Bei unterbrochenen Frames bleibt der zuletzt empfangene Frame sichtbar, ist aber als veraltet markiert. Nach einer Bridge-Neuverbindung wartet die Seite auf neue Nachrichten und markiert alte Samples nicht wieder als frisch.

Die Karte wird höchstens mit 10 Hz aktualisiert, verwendet eine Keep-Last-Tiefe von 1 und fordert Best-Effort-QoS an, passend zum `/hmmd/rdmap`-Publisher. Die Topic-Auswahl zeigt die jüngste Nachricht eines freigegebenen Topics. Für weitere Topics müssen sowohl `topic_registry.mjs` in der Browseranwendung als auch die serverseitige `topics_sub_glob`-Freigabe geändert werden; Nachrichten werden nicht dynamisch über rosapi entdeckt.

Unter „Color maximum“ folgt „Dynamic (each frame)“ dem Minimum und Maximum des jeweiligen Frames. „Fixed“ setzt die Farbskala auf null bis zum eingegebenen festen Maximum. Beim ersten Umschalten wird das aktuelle Frame-Maximum übernommen; danach kannst du einen positiven festen Wert eingeben. Dieser Wert bezieht sich immer auf die rohe Amplitude zum Quadrat, auch in der log1p-Ansicht. Dort wird die feste Grenze ebenfalls mit log1p umgerechnet. Größere Werte erhalten die hellste Farbe; Rohdaten bleiben unverändert. Die Einstellung gilt für die geöffnete Seite und wird beim Neuladen zurückgesetzt.

### Python-Beispiel mit roslibpy

Das folgende Beispiel läuft auf dem PC durch denselben SSH-Tunnel. Es braucht kein lokales ROS und keine generierten `hmmd_interfaces`-Pakete. `roslibpy` 2.1.0 wurde in einer isolierten Umgebung ohne importierbare ROS-Pakete erfolgreich mit rosbridge 2.7.1 geprüft. `roslibpy.Topic` setzt selbst kein QoS-Profil; rosbridge nimmt für Topics ohne QoS-Angabe einen Best-Effort-Subscriber und gleicht vorhandene Publisher ab. Dieser Standardpfad empfing die HMMD-Map mit Best-Effort-QoS.

Installiere `roslibpy` auf Debian-/Ubuntu-Systemen in einer virtuellen Umgebung:

```bash
python3 -m venv .venv
. .venv/bin/activate
python -m pip install roslibpy==2.1.0
```

```python
import roslibpy
from threading import Event

client = roslibpy.Ros(host='127.0.0.1', port=9090)
client.run()

topic = roslibpy.Topic(
    client,
    '/hmmd/rdmap',
    'hmmd_interfaces/msg/RangeDopplerMap',
    throttle_rate=100,
    queue_length=1,
)
topic.subscribe(lambda message: print(
    message['doppler_bins'], message['range_gates'],
    len(message['amplitude_squared']), message['header']['stamp'],
))

wait = Event()
try:
    while client.is_connected:
        wait.wait(1)
finally:
    topic.unsubscribe()
    client.terminate()
```

Der Empfang über rosbridge enthielt 20×16 Dimensionen, 320 ganzzahlige Matrixwerte, Header und Zeitstempel sowie den HMMD-Status. Ein Vergleich desselben Frames über einen direkten ROS-Subscriber und WebSocket bestätigte identische Dimensionen, Rohwerte, `frame_id` und Zeitstempel. Im isolierten `roslibpy`-Client waren weder `rclpy` noch `hmmd_interfaces` importierbar.

Die Browserprüfung auf dem CM5 verwendete den echten rosbridge-Server: Roh-/Logansicht, Topic-Auswahl, fehlende Frames, Publisher-Stopp, Bridge-Abbruch und Wiederverbindung bestanden. Die Fehlerzustände wurden mit ausdrücklich synthetischen ROS-Nachrichten geprüft, anschließend wurde der echte HMMD-Empfang wiederhergestellt. Der PC-Browser lud die Seite über LAN; der Nutzer bestätigte den laufenden Zugriff nach Start des SSH-Tunnels, und der CM5 zeigte die SSH-Weiterleitung samt beiden HMMD-Abonnements auch nach dem Bridge-Neustart. Details stehen im [Prüfprotokoll](../../../../agent_notes/cm5/sensors/hmmd/browser-integration-2026-10-04.json).

Der Browser-Client und dieses Beispiel führen keine Schreiboperationen durch. Das ist keine Einschränkung aller Operationen, die rosbridge serverseitig anbieten kann.

## Zeichne die drei Situationen auf

1. Starte Sensor und Heatmap.
2. Zeichne den leeren Messbereich auf.

```bash
ros2 bag record -o hmmd-empty /hmmd/rdmap /hmmd/status
```

3. Beende die Aufnahme mit `Ctrl+C`.
4. Wiederhole die Aufnahme mit einer ruhenden Person als `hmmd-stationary` und einer bewegten Person als `hmmd-moving`.
5. Notiere für jede Aufnahme Aufbau, Position, Dauer, beobachtete Datenrate und Auffälligkeiten in [observations.md](../../../../agent_notes/cm5/sensors/hmmd/observations.md).

Die Frame-Veröffentlichung verwendet Best-Effort-QoS. Prüfe nach jeder Aufnahme mit `ros2 bag info`, dass die Matrixnachrichten enthalten sind. Verwende die [QoS-Referenz](../reference/hmmd-ros2.md), falls die verwendete rosbag2-Version die QoS nicht automatisch passend erkennt.

## Spiele eine Aufnahme ohne Sensor ab

1. Beende den Sensorknoten, damit er keine Live-Daten parallel veröffentlicht.
2. Starte die Heatmap mit der aktiven Workspace-Umgebung.
3. Spiele eine Aufnahme ab.

```bash
ros2 bag info hmmd-empty
ros2 bag play hmmd-empty
```

Die Heatmap benutzt die lokale Zeit seit dem letzten empfangenen Frame für die Kennzeichnung veralteter Daten. Sie benötigt keine Simulation der ROS-Zeit für die reine Anzeige. Eine Wiedergabe mit `--clock` darf die Originalzeitstempel für andere Verbraucher verfügbar machen.

4. Pausiere die Wiedergabe und prüfe die Kennzeichnung veralteter Daten.
5. Prüfe die gleiche Kennzeichnung am Dateiende.
6. Spiele die beiden anderen Aufnahmen ab und vergleiche die Radarantwort.

## Prüfe den aktuellen Implementierungsstand

Der Parser, das Anzeigenmodell und die Paketquellen können lokal geprüft werden. Ein ROS-2-Build, der echte Sensorempfang und die rosbag2-Aufnahme/Wiedergabe auf dem CM5 müssen mit erreichbarer Hardware separat bestätigt werden. Der Roadmap-Meilenstein ist erst nach den dort genannten Live-Kriterien abgeschlossen.
