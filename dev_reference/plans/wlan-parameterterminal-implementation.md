# Implementierungsauftrag: WLAN-Parameterterminal, Phase 1

Stand 28.09.2026. Ausführbarer Auftrag für GPT-6 Luna, Reasoning **high**. Nur Planung in dieser Sitzung; Worktree und Implementierung erst bei Ausführung dieses Auftrags. Engineering-Review nach `plan-eng-review`: **clean für den hier definierten Scope**, Hardwareabnahme bleibt Ausführungsschritt.

## Ziel und feste Grenzen

Implementiere auf dem vorhandenen Navbot-ES02 ein einfaches Parameterterminal über WLAN im Heimnetz. Client: Python auf Linux Mint. Bedienung wie bisher `PP5`, `PD0.12`, `SP0.045`, `V0.2` sowie Abfragen ohne Zahlen. Gerät bestätigt den übernommenen Wert. Lesen auch bei aktiver Fahrt, Schreiben ausschließlich bei frischem, gültigem SBUS-Signal und CH5 aus. Profile lokal am PC speichern/laden.

**Nicht implementieren:** Logging, Streaming, Recorder, Messstart, Diagramme, Weboberfläche, OTA, einen dauerhaften Steuerungs-Access-Point, BLE-Provisionierung, dauerhaftes Speichern der Gains im Roboter, neue Regleralgorithmen oder veränderte Start-Gains. Ein SoftAP nur für die WLAN-Provisionierung beim Erststart oder nach bewusstem Reset ist vorgesehen. Keine Vorab-Frameworks für das spätere Logging. Bestehende serielle Diagnosemöglichkeiten erhalten.

Hardware laut Nutzer: ESP32-S3-WROOM-1 ohne externen RAM, bestehende Pinbelegung korrekt. PSRAM deaktiviert lassen. Heimnetz ohne Internetzugriff genügt. Netzwerkverbindung darf Fahrquelle, Motorfreigabe und RC-Kanäle nicht verändern. Keine Firmware auf das Gerät flashen, solange nicht separat beauftragt; ein prüfbares Build und eine Uploadanleitung sind das Lieferziel.

## 1. Isolierte und richtige Ausgangsbasis

Original: `/home/niko/Dokumente/Bastelei/roboter/Navbot-ES02`. Vor jedem Quellzugriff `AGENTS.md`, danach für diese Aufgabe `agent_notes/usb-serial.md` und Hardwareübersicht lesen. Relevante technische Basis: Hauptsketch, `filter.*`, `FUTABA_SBUS.*`, `ble.*`, `robot.*`, aktueller README und `agent_notes/drive-instability-handoff.md`. Dieses Dokument ist maßgeblich für Phase 1; das größere Logging-Konzept ist nur Hintergrund.

**Achtung: HEAD allein ist nicht die Basis.** Zum Planungszeitpunkt sind README, Hauptsketch und robot.cpp geändert; wichtige Anleitungen/Skripte sind untracked. Der aktuelle Sketch enthält `V`/`wheelSpeedFeedbackGain`, `balancePidNeedsPriming` und weitere aktuelle Fahranpassungen. Diese nicht durch ältere Versionen ersetzen.

Vorgehen des ausführenden Leitagenten:

1. `git status --short`, Branch und HEAD erfassen. Ausgangsdateien in einem temporären Snapshot sichern und Hashmanifest schreiben. Quellstand während der Übernahme erneut vergleichen; bei gleichzeitiger Änderung neu erfassen statt widersprüchlich zusammenkopieren.
2. Isolierten Git-Worktree aus dem erfassten HEAD erstellen, z. B. neben dem Original als `Navbot-ES02-wifi-tuning` mit einem noch freien Branch `wifi-parameter-terminal`. Vorhandene gleichnamige Arbeit niemals überschreiben. Alternativ verwalteten Codex-Worktree verwenden und dessen Pfad explizit benutzen.
3. Alle getrackten Änderungen einschließlich Löschungen anhand `git diff --binary HEAD` im Worktree übernehmen. Relevante untracked Text-/Quelldateien aus `AGENTS.md`, `agent_notes/`, `dev_reference/`, `scripts/` und dem Sketchverzeichnis ergänzen, ohne ignorierte Dateien/Secrets pauschal zu kopieren. Keine Buildverzeichnisse, Binärimages, Messlogs oder `__pycache__` übernehmen. Falls ein benötigtes Log im Handoff referenziert wird, bei Bedarf im Original nur lesen.
4. Manifestvergleich der übernommenen Quellen gegen Snapshot; `git diff --check`. Keine Secrets in Snapshot-Commit aufnehmen. Im Worktree einen lokalen Basis-Commit der übernommenen Arbeitskopie erstellen, damit der Feature-Diff separat reviewbar ist. Original weder committen, stashen, resetten noch ändern.
5. Vor Featurearbeit diese Basis mit dokumentiertem FQBN kompilieren und genaue Core-/Bibliotheksversionen aufzeichnen. Bei Baselinefehler Ursache dokumentieren und minimal klären; keine Bibliotheken blind aktualisieren. Featuretests vergleichen gegen diesen Basis-Commit, nicht ursprüngliches HEAD.

## 2. Codefakten, die erhalten bleiben müssen

- `Commander command(Serial)` und `CbAnglePid`, `CbSpeedPid`, `CbYawPid`, `CbRollPid` bearbeiten die Konfigurationsobjekte `AnglePid`, `SpeedPid`, `YawPid`, `RollPid`.
- Die aktive Regelung verwendet `Angle_Pid` usw. und übernimmt/transformiert Gains im Reglerpfad. Beispielsweise werden Speed-Gains dort durch 100 geteilt. WLAN-Befehle müssen dieselben **Konfigurationswerte wie Serial** meinen, nicht direkt die effektiven Kp/Ki-Felder.
- `PidParameterTuning == 0` aktiviert automatische Gain-Zuweisung. `U1` schaltet manuelles Tuning ein; niemals automatisch `U1` als Nebeneffekt des Verbindens senden. Gain-Schreiben bei `U0` mit `AUTO_MODE` ablehnen. `U1` separat oder als Bestandteil eines atomaren Profils erlauben.
- `V` ist `wheelSpeedFeedbackGain`; aktueller Code begrenzt die Wirkung auf 0 bis 0,4. Externe Eingaben außerhalb dieses Bereichs ablehnen statt still begrenzen. Keine Änderung seiner Berechnungslogik.
- `balancePidNeedsPriming` und vorhandenes Zurücksetzen der Integratoren bei CH5 aus erhalten. Nicht neue Zustandsresets während aktiver Fahrt einführen.
- BLE-Verbindung schaltet in `CtrlInput()` auf BLE-Fahrdaten; danach existiert zusätzlich SBUS-Verarbeitung. WLAN-Feature nicht über `rp.ble_connected` implementieren. Im explizit aktivierten WLAN-Tuning-Build BLE-Initialisierung/Task auslassen, SBUS-Pfad unverändert lassen. Standardbuild ohne WLAN behält BLE wie bisher. Keine umfassende Fahrquellen-Refaktorierung in dieser Phase.

## 3. Parameterumfang und Semantik

Erstversion unterstützt `PP/PI/PD/PL`, `SP/SI/SD/SL`, `YP/YI/YD/YL`, `RP/RI/RD/RL`, `U`, `V`. Einzelner Name liest, Name plus Zahl schreibt. Keine beliebige Weiterleitung an Commander: Motor-, Kalibrier-, Flash-, Trace- und andere Befehle sind nicht über WLAN erreichbar. Insbesondere `T`, `M`, `E`, `K`, `G`, Filter-/Servo-/Touchparameter bleiben vorerst Serial vorbehalten. `help` listet den Umfang ausdrücklich auf.

Parsing: exakter erlaubter Name, optional endliche Dezimalzahl mit Punkt, optional Exponent. Leerzeichen am Zeilenrand akzeptieren; Restzeichen, NaN/Inf, Überläufe, mehrere Kommandos und überlange Eingaben ablehnen. Keine Python-`eval`- oder Shellausführung. U akzeptiert nur 0 oder 1. Keine Rundung auf Integer für Gains.

Initiale technische Eingabegrenzen für WLAN, nicht als Nachweis physisch sicherer Gains bezeichnen:

| Gruppe | P | I | D | L |
|---|---|---|---|---|
| Winkel P* | 0…20 | 0…500 | 0…2 | 0…1 |
| Speed S* | 0…2 | 0…5 | 0…2 | 0…100 |
| Yaw Y* | 0…200 | 0…100 | 0…5 | 0…100 |
| Roll R* | 0…10 | 0…100 | 0…5 | 0…100 |

V: 0…0,4. Diese bewusst begrenzte Erstversion deckt die im Plan geprüften aktuellen Arbeitswerte ab. Bei geändertem Baselinecode mögliche Abweichungen berichten; vorhandene Werte trotzdem unverändert lesen, niemals beim Booten clampen. Metadaten erläutern L als die im vorhandenen Pfad verwendete Integralgrenze und weisen auf Modus-/Skalierungsabhängigkeit hin.

Zentrale kleine Registry liefert Name, Wertebereich, Lesewert und Schreibziel. Neue Parametereinträge müssen später gezielt ergänzt werden; keine Pluginarchitektur. Netzwerkzugriff arbeitet ausschließlich über diese Registry. Bestehende Serial-Callbacks dürfen erhalten bleiben, da sie im Loop dieselben Konfigurationsobjekte bedienen; keine zweite Kopie der Gains im Netzwerkmodul. Serial-Semantik nicht durch neue WLAN-Grenzen ändern. Gleichzeitige Serial-/WLAN-Schreibfolgen nicht als Transaktion garantieren; PC-Profile und Snapshot müssen den tatsächlichen Zustand lesen.

## 4. Architektur und Threadgrenzen

Neue Dateien vorzugsweise `TuningParameters.{h,cpp}`, `WifiTuning.{h,cpp}` im Sketch; `scripts/wifi_tune.py` als Client. Namen dürfen aus Repositorykonventionen angepasst werden. Hauptsketch enthält nur Anbindung an Registry, SBUS-Freshness und Queue-Verarbeitung.

WLAN-Task startet `WiFiProv` asynchron mit SoftAP und Security 0. Ohne gespeicherte Zugangsdaten provisioniert `esp_prov` das WLAN; nach erfolgreicher Provisionierung und bei normalen Neustarts nutzt der ESP32 die im WLAN-NVS gespeicherten Werte. Fehlendes WLAN verhindert weder Boot noch RC-/Serial-Betrieb. Ein Commander-Befehl setzt nur die WLAN-Konfiguration zurück. Der unauthentifizierte HTTP-Server startet bei WLAN-Verbindung und verfügbaren Requestqueues; maximal eine Mutation gleichzeitig, feste Queuekapazität, keine unbeschränkte Client-/Requestsammlung.

HTTP-Handler authentifiziert und parst, erstellt einen begrenzten POD-Request und wartet nur im Netzwerktask begrenzt auf Antwort. Keine Zeiger auf temporäre JSON-/HTTP-Puffer in der Queue. Loop verarbeitet höchstens eine vollständige Transaktion pro definiertem Kontrollzyklus. JSON-Serialisierung und Socketwrites ausschließlich im Netzwerktask. Auch Lese-Snapshots im Loop erstellen, damit ein HTTP-Callback keine inkonsistente Mischung von Parameterwerten/Modus/RC liest.

**Schreibbedingung zum tatsächlichen Ausführungszeitpunkt erneut prüfen:** unterstützter Zweirad-Master-Balancebetrieb, seit Boot wenigstens ein gültiger SBUS-Frame, letzter gültiger Frame höchstens 250 ms alt, SBUS weder lost noch failsafe, CH5 exakt OFF. Gestürzter Roboter mit CH5 ON ist keine Schreibfreigabe. Offline/BLE-Fahrmodus nicht als deaktivierte Fahrt interpretieren. Freshness nur bei tatsächlich gültigem Frame aktualisieren; vorhandenes `sbus_dt_ms` ist ein Intervall, kein dauerhaftes Altersmaß. Neue Freshness-Metadaten dürfen nicht die bestehende RC-Auswertung verändern.

Aufträge erhalten Request-ID und kurze Gültigkeitsfrist (z. B. 1 s). Veraltete queued Writes vor Ausführung verwerfen, kein spätes überraschendes Anwenden. Atomare Batchverarbeitung: alle Namen/Werte und resultierenden Modus prüfen, danach erst gemeinsam übernehmen. Kein Teilerfolg. Ack nach Anwendung enthält Snapshot der Konfigurationswerte. Das ist keine Bestätigung einer aktuellen Motorwirkung; bei CH5 OFF wird nicht geregelt. Fehler-/Ack-Queue ebenfalls begrenzen.

## 5. Kleiner API-Vertrag (vor Parallelisierung festschreiben)

Nur lokale IPv4-HTTP-Verbindung, Standardport 80, API `/api/v1/`. Vorhandene ESP32-Core-/ArduinoJson-Bibliotheken verwenden; keine neue asynchrone Webserver-Abhängigkeit nötig. Kein TCP-Stream in dieser Phase.

- `GET /api/v1/status`: `{api_version, boot_id, build_id, supported_mode, rc_valid, rc_age_ms, ch5_off, tuning_mode}`.
- `GET /api/v1/parameters`: kohärenter Snapshot mit denselben IDs plus `values` und Metadaten. Für einfache Abfrage darf Client gesamten Snapshot holen.
- `POST /api/v1/parameters`: `{request_id, expected_boot_id, values:{"PP":5,"PD":0.12}}`. Maximal 18 verschiedene erlaubte Parameter, JSON-Body maximal 2048 Byte; doppelte JSON-Keys/mehrdeutige Requests ablehnen. `U:1` plus Gains in einem Batch erlaubt; resultierendes `U:0` plus Gainänderungen ablehnen. V ist vom Auto-Gainmodus unabhängig.
- Erfolg: `{ok:true, request_id, boot_id, values:<angewandter Snapshot>}`; Fehler: `{ok:false, request_id, error:<stabiler Code>}` ohne Stacktrace oder Geheimnisse.
- HTTP 400 ungültiges Schema/Wert/Name; 409 `DRIVE_ACTIVE`, `RC_UNAVAILABLE`, `AUTO_MODE`, `BOOT_CHANGED`, `UNSUPPORTED_MODE` oder Request-ID-Konflikt; 413 zu groß; 503 Queue belegt; 504 Auftrag abgelaufen.

Eine begrenzte Antwortablage für die letzte Mutation ist ausreichend: gleicher Request-ID/Body/Boot liefert denselben Abschluss, dieselbe ID mit anderem Body Fehler. Neue Session nach Neustart anhand Boot-ID erkennen. Client wiederholt Writes nach Timeout **nicht automatisch**, sondern liest Zustand neu und meldet Ausgang unbekannt, falls der Abschluss nicht sicher feststeht. Wenn bereits ausgeführt, darf ein Timeout keine fingierte Rücknahme behaupten. Kein Transaktionssystem über Neustarts hinweg.

## 6. Einrichtung und Terminal

WLAN-Zugangsdaten nicht in Sourcecode, Builddateien oder Firmware-Binary einbetten. Das lokale Buildmakro aktiviert das WLAN-Feature; der normale Build lässt es ausgeschaltet. SoftAP-Provisioning verwendet Security 0 ohne Kopplung und ist für die Entwicklung vorgesehen. `esp_prov` muss WLAN-Zugangsdaten interaktiv abfragen; keine echten Werte in Kommandozeilenargumente, Shell-Skripte oder Logs übernehmen. Core Debug Level bleibt `None`, weil `WiFiProv` bei höheren Debug-Stufen Provisionierungswerte protokollieren kann. Das Parameterterminal verwendet keine zusätzliche Authentifizierung; jeder erreichbare WLAN-Client kann seine freigegebenen Lese- und Schreibendpunkte aufrufen.

Es gibt keine Token-Einrichtung und keinen Token in NVS. Nach dem WLAN-Beitritt startet der HTTP-Dienst direkt. HTTP verwendet Port 80 ohne TLS und ohne Authentifizierung; verwende ihn nur in einem vertrauten Entwicklungsnetz und schalte das WLAN-Feature vor dem Flashen eines Builds für den normalen Betrieb ab.

Client ausschließlich Python-Standardbibliothek, sofern kein konkreter Grund entgegensteht. Folgender Bedienvertrag:

```text
python3 scripts/wifi_tune.py --host ROBOT_IP
navbot> status
navbot> U
navbot> U1
navbot> PP5
navbot> PD0.12
navbot> V0.2
navbot> PP
navbot> show
navbot> save profiles/stand.json
navbot> load profiles/stand.json
navbot> quit
```

Zusätzlich `--command PP` und `--command SP0.045` für automatisierte Einzelaufrufe; `--json` für maschinenlesbare Antworten. `--config` zeigt auf die lokale Hostkonfiguration. `help`, EOF und Ctrl-C sauber behandeln. Exitcodes: 0 Erfolg, 2 lokale Eingabe-/Profilfehler, 3 Transport/Timeout, 4 Geräteablehnung. Keine automatischen Writes beim Start oder Reconnect. Kein automatisches Flashen.

Profile: versioniertes JSON nur mit erlaubten Konfigurationsparametern und optionalem Kommentar, keine Tokens/WLAN-Daten. `save` holt einen frischen Snapshot. `load` validiert vollständig und sendet einen einzigen Batch; keine Serie teilweise angewandter Einzelkommandos. Profil mit manuellen Gains muss U=1 enthalten. Gespeicherte Dateien atomar ersetzen, bestehende Datei nur mit explizitem `--force` bzw. `save --force` überschreiben. Fehler bei fehlendem Verzeichnis lesbar melden. Bestätigung zeigt tatsächliche Werte und Gültigkeit bis Neustart an.

## 7. Arbeitspakete und optionale Agenten

Standard: ein GPT-6 Luna **high** implementiert sequenziell, ein zweiter Luna **high** prüft anschließend unabhängig. Das ist bei Änderungen am monolithischen Sketch meist einfacher als mehrere Firmwareautoren.

Falls parallele Arbeit sinnvoll ist, erst API-Schema und Baseline festschreiben, danach maximal zwei Implementierer:

- **Agent A, Firmware:** Registry, RC-Schreibschutz, Queue/Acks, WLAN/API, Firmwaretests. Besitzt Sketchdateien und Firmwaretestverzeichnis.
- **Agent B, PC:** Python-Terminal, Profile, Mock-HTTP-Server und Clienttests. Besitzt `scripts/wifi_tune.py` und zugehörige Tests. Kein Edit am Hauptsketch.
- **Leitagent/Reviewer:** Baseline/Worktree, API-Vertrag, Integration, README/Bedienanleitung, Sicherheitsreview und Buildvergleich. Integration erst nach beiden Ergebnissen.

Alle späteren Agenten explizit mit Modell `gpt-6-luna`, Reasoning `high` starten. Falls Modellüberschreibung einen begrenzten Kontext verlangt, diesen Auftrag, Testplan, Risikoregister und Worktreepfad vollständig mitgeben. Subagenten können denselben isolierten Worktree bei disjunkten Dateien nutzen. Eigene Agent-Worktrees alternativ nur aus dem gemeinsamen lokalen Basis-Commit, niemals jeweils aus altem HEAD. Kein Agent verändert die ursprüngliche Arbeitskopie.

## 8. Abnahme und Lieferung

Verbindlich: [Testplan](wlan-parameterterminal-test-plan.md) und [Risikoregister](wlan-parameterterminal-risk-register.md). Risiko **high**, weil Netzwerkbefehle physische Regelparameter ändern; deshalb nach Umsetzung `/security-review`, `/cli-qa` für die neue CLI und `/review` für den Feature-Diff. Dokumentation mit `/document-release` synchronisieren. Kein `/ship`, Deployment oder Upload in diesem Auftrag.

Builds: dokumentierter FQBN `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,DebugLevel=none,PartitionScheme=no_fs`, PSRAM unverändert aus. Das 4-MB-Partitionsschema `no_fs` stellt zwei 2-MB-App-Partitionen bereit; ein Dateisystem wird nicht verwendet. Der Build benötigt keine WLAN-Credentials oder Dummy-Secrets. Aktive Arduino-Core-/SimpleFOC-Versionen festhalten. Compile-Erfolg ersetzt keine Hardwareabnahme.

Lieferung: isolierter Worktree/Branch, sauber abgegrenzter Feature-Diff relativ zum Snapshot-Basis-Commit, Protokoll/Setup im README oder eigener referenzierter Anleitung, Beispieldateien, automatische Tests und Buildnachweise, offene Hardwarechecks. Kein Loggingcode. Kein behaupteter erfolgreicher Funk-/Fahrtest ohne Gerät. Abschlussbericht benennt bekannte Grenzen und exakt, ob ein Image nur gebaut oder auch separat autorisiert geflasht wurde.

## Direkt verwendbarer Startauftrag

> Implementiere ausschließlich Phase 1 des Plans `dev_reference/plans/wlan-parameterterminal-implementation.md` mitsamt Testplan und Risikoregister. Lies zuerst AGENTS.md. Arbeite in einem isolierten Worktree und übernimm den aktuellen, teilweise uncommitteten Quellstand nach dem beschriebenen Snapshotverfahren. Erhalte aktuelle Regleranpassungen einschließlich V und balancePidNeedsPriming. Nutze GPT-6 Luna mit Reasoning high; optional separater Clientagent und unabhängiger Reviewer gemäß Dateizuständigkeiten. Ziel ist WLAN-Parameteränderung mit Serial-ähnlichem Linux-Terminal, RC-Schreibschutz, Rücklesen und PC-Profilen. Logging, Streaming, OTA und Flashen gehören nicht dazu. Führe Tests, Builds, CLI-QA, Sicherheitsreview und Diffreview aus. Liefere den überprüfbaren Worktree sowie eine genaue Einrichtung-/Hardwaretestanleitung und kennzeichne nicht ausgeführte Gerätetests ausdrücklich.
