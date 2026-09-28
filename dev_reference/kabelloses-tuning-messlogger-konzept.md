# Konzept: kabelloses Tuning und gepuffertes WLAN-Logging

Stand: 28.09.2026. Recherche der lokalen Arbeitskopie und Herstellerdokumentation; keine Firmwareänderung, kein Flash und keine Hardwaremessung.

## Entscheidung und Anforderungen

**Empfehlung: WLAN im vorhandenen Heimnetz, kleines Python-Werkzeug auf Linux Mint, binäres TCP-Streaming mit kurzem internem RAM-Puffer.** Der Computer ändert Parameter, startet die Aufnahme und schreibt die Daten fortlaufend in eine lokale Datei. Gewünschte Dauer: einstellbar von 20 bis 40 Sekunden, Standard 30 Sekunden. Der Nutzer hat Streaming ausdrücklich als Alternative zur vollständigen lokalen Aufzeichnung zugelassen. Damit ersetzt dieser Entwurf die bisherige Empfehlung für kurze RAM-Aufnahmen mit anschließendem Download.

Zielnutzer ist der Betreiber beim Regler-Tuning und bei der Fehlersuche, insbesondere Neigungsvibrationen. Erfasst werden zunächst alle tatsächlich berechneten Reglerzyklen mit zugehörigen Eingängen und Ausgaben. Ein Neustart darf Messdaten löschen. Heimnetz genügt; kein Access Point, keine Cloud, keine SD-Karte und keine grafische Oberfläche für die Erstversion nötig. Die RC-Fernsteuerung bleibt für die Fahrt zuständig.

Annahme: Firmwareupdates bleiben zunächst über USB; kabellose Updates wurden nicht ausdrücklich beauftragt. Erstinstallation der neuen Firmware benötigt ebenfalls den vorhandenen Flashweg. Das Dauerziel ist 20–40 Sekunden; Kanalumfang und tatsächliche native Reglerfrequenz werden beim technischen Nachweis festgelegt. Die Dauer ist eine Anforderung, noch kein gemessenes Ergebnis.

**Hardware laut Nutzer bestätigt:** ESP32-S3-WROOM-1 ohne externen RAM; die Pinbelegung ist korrekt. PSRAM entfällt. Offen ist nur noch, wie viel interner RAM nach Initialisierung von Regelung und WLAN zuverlässig für den Recorder reserviert werden kann. Dieser Speicher überbrückt Sendepausen; er muss nicht die gesamte Aufnahme enthalten.

## Was bereits vorhanden ist

| Bereich | Befund | Fehlender Baustein |
|---|---|---|
| Tuning | SimpleFOC Commander auf Serial, u. a. Winkel-, Speed-, Yaw-/Roll-PID, Filter | Netzwerkzugang und gemeinsame validierte Parameterverwaltung |
| Persistenz | Preferences speichert IMU-/Gyro-/Servokalibrierung | Tuning-Gains werden darüber nicht dauerhaft gespeichert; für Erstversion genügen PC-Profile plus Rücklesen |
| BLE | GATT Write/Notify, 20-Byte-Frames mit 15 Byte Nutzlast; Fahrbefehle und Geräteinfo | Kein Tuning-/Messdienst; kein zuverlässiger großer Dateitransfer |
| WLAN | ESP32-S3 kann WLAN; `CMD_WIFI` ist definiert | Handler ist auskommentiert, kein implementierter WLAN-Dienst |
| Telemetrie | K58 enthält 26 Zahlenfelder, frühestens alle 50 ms, also etwa 20 Hz | Recorder mit Sendepuffer, Sample-IDs, präzise Zeitbasis und Stream-Zustandsautomat |
| IMU | ICM42688 per SPI, 1-kHz-ODR, ±8 g/±2000 °/s, direkter Registerabruf | Keine FIFO-Nutzung und kein Nachweis lückenloser 1-kHz-Erfassung |
| PC | Capture-, Versuchslauf- und Analyseskripte für Serial vorhanden | WLAN-Transport, Binärdecoder, Prüfung der Analyse auf höhere/variable Raten |

Quellen: [Hauptsketch](../src/ES-02/OllieFOCdrive/OllieFOCdrive.ino), Funktionen `setup`, `CtrlInput`, `ImuUpdate`, `FlashInit`, `print_data`/58 und `loop`; [ble.cpp](../src/ES-02/OllieFOCdrive/ble.cpp), [robot.cpp](../src/ES-02/OllieFOCdrive/robot.cpp), [ICM42688.cpp](../src/ES-02/OllieFOCdrive/ICM42688.cpp), [run_drive_trial.py](../scripts/run_drive_trial.py), [analyze_drive_trace.py](../scripts/analyze_drive_trace.py).

Relevante Fallstricke im Bestand:

- Bei BLE-Verbindung wählt `CtrlInput()` die BLE-Fahrbefehle; anschließend ruft `loop()` zusätzlich `RXsbus()` auf. Diagnoseverbindung und Fahrquelle sind nicht sauber getrennt. Ein BLE-Tuningclient wäre daher keine harmlose Serial-Brücke.
- `time_dt >= 0.001` garantiert keinen 1-kHz-Regeltakt. Frühere Versuche dokumentieren etwa 1,664 ms, entsprechend ungefähr 601 Hz; dies ist kein Benchmark des künftigen WLAN-Builds.
- `print_data()` läuft vor späteren Reglerberechnungen. Ein neuer Recorder muss einen definierten Snapshot nach einem vollständigen Reglerzyklus erstellen und berechnete Motorziele von bereits angewandten Ausgaben unterscheiden.
- Der BLE-Task auf Core 0 wartet aktiv in einer Schleife. Netzwerkverarbeitung braucht blockierendes Warten und begrenzte Arbeit; gemeinsame Daten dürfen nicht ungeschützt geändert werden.

## Speicherfrage: zuerst klären

Der Nutzer bestätigt am 28.09.2026: ESP32-S3-WROOM-1, kein externer RAM, korrekte Pinbelegung. Die N8R8-Bezeichnung in der [Netzliste](../agent_notes/hardware-netlist.md) entspricht damit nicht der bestätigten RAM-Ausstattung des aufgebauten Roboters. Sie begründet keinen Umbau der Pins. Die genaue Flashgröße folgt aus der angegebenen Modulbezeichnung nicht und bleibt für dieses RAM-Konzept unerheblich.

PSRAM bleibt deaktiviert. Vor Dimensionierung freien internen Heap, größten freien Block und Speicherreserve unter WLAN-/Reglerlast messen. Die offene Frage ist die verfügbare Puffergröße, nicht mehr ein PSRAM-Pinkonflikt.

### Datenrate und Pufferbudget

Für ein beispielhaftes vollständiges Profil von 128 Byte pro Regeltakt (endgültiges Schema noch festzulegen):

| Rate | Nutzdaten pro Sekunde | 20 Sekunden am PC | 40 Sekunden am PC |
|---|---:|---:|---:|
| 600 Hz | 76,8 kB/s = 0,6144 Mbit/s | 1,536 MB | 3,072 MB |
| 1 kHz | 128 kB/s = 1,024 Mbit/s | 2,56 MB | 5,12 MB |

Protokollaufwand kommt hinzu. Das liegt deutlich unter Espressifs dokumentierten TCP-Laborwerten ([WLAN-Performance](https://docs.espressif.com/projects/esp-idf/en/v5.3.6/esp32s3/api-guides/wifi.html)); daraus folgt eine plausible Machbarkeit, keine Garantie unter Motor-/Reglerlast im konkreten Heimnetz. Der Engpass ist voraussichtlich die störungsarme Integration, nicht die reine Funkdatenrate. Mit dem real verwendeten Arduino-Core prüfen.

Ziel für den ersten Prototyp: **64 KiB vorab reservierter interner Sendepuffer**, sofern nach WLAN-/Reglerinitialisierung einschließlich Stacks, TCP-Puffern und Reserve verfügbar. Bei 128 Byte pro Tick überbrückt dieser rechnerisch etwa 0,85 s bei 600 Hz oder 0,51 s bei 1 kHz. Das sind Obergrenzen bei leerem Puffer; bereits belegte Daten und Blockmetadaten reduzieren die Reserve. Es wird kein freier 64-KiB-Block vorausgesetzt. Kleinere Kapazität ergibt entsprechend kürzere tolerierte Pausen.

Die nominellen 512 KB internen SRAM sind nicht vollständig für den Recorder frei. Keine Puffergröße auf Kosten der Regelung erzwingen. Ziel für einen Vorab-Lasttest: unter repräsentativer Reglerlast mindestens doppelte benötigte Nutzdatenrate ohne wachsenden Rückstau; anschließend reale Erfassung prüfen. Nach einer Sendepause muss genügend freie Bandbreite zum Aufholen vorhanden sein.

Ein kompaktes 48-Byte-Profil bleibt eine wählbare Alternative: 32-Bit-Zeitoffset in µs relativ zum 64-Bit-Messstart, Tick-ID/Flags sowie zehn float32-Werte (Neigung, Neigungssollwert, Gyro der Neigungsachse, Winkel-P/I/D, Speed-Reglerausgang, mittlere Radgeschwindigkeit, Motorziel links/rechts). Bei 600 Hz benötigt es 28,8 kB/s. Für die Fehlersuche zunächst das vollständigere Profil bevorzugen; Streaming beseitigt den Zwang, wesentliche Regleranteile nur für längere Dauer wegzulassen. Keine automatische Profiländerung während eines Versuchs.

## Einfacher Aufbau

```text
Linux Mint: Python-Client + Profile + Analyse
                  │ WLAN im Heimnetz
                  ▼
ESP32: HTTP-API → begrenzte Befehlsqueue → Parameter/Recorder-Steuerung
                                               │
Reglerzyklus → konsistenter binärer Snapshot → begrenzter RAM-Puffer
                                               │
                       separater Sender → TCP → Rohdatei auf Linux Mint
                                               │
                                     nach Ende: CSV und Analyse
```

Roboter verbindet sich als WLAN-Station, PC findet ihn über bekannte IP, optional mDNS. Zugangsdaten einmalig lokal konfigurieren. Ein kleiner HTTP-Dienst übernimmt Tuning, Status, Start und Stop; eine separate TCP-Verbindung überträgt binäre Messblöcke. Keine WebSockets, Browseroberfläche oder BLE-Provisionierung erforderlich. Die Trennung hält Stop/Status unabhängig vom Datenrückstau. Beide Verbindungen werden authentifiziert und derselben Sitzung zugeordnet. Schreibende API mit gerätespezifischem Schlüssel schützen; genau ein schreibender Client, begrenzte Requestgröße und Timeouts.

Netzwerkcallbacks validieren Requests und stellen sie in eine Queue. Nur die Regelungsseite übernimmt Parameter an einer definierten Zyklusgrenze. Bestätigung erst nach tatsächlicher Übernahme, einschließlich wirksamer Werte und Konfigurationsrevision. Keine mehrfachen ungeschützten Schreibzugriffe auf PID-Strukturen.

Parameter haben Typ, Einheit, Bereich und Änderungsbedingung; NaN und ungültige Werte ablehnen. Erstversion ändert Gains bei deaktivierter Fahrt, zwischen Messungen. Profile werden auf dem PC gespeichert und nach dem Anwenden zurückgelesen. Integrator-/Filterzustände beim Wechsel bewusst behandeln. Während der Aufnahme bleibt die Konfiguration fest. Dauerhaftes Speichern auf dem Roboter ist eine optionale spätere Funktion.

Der bestehende serielle Zugang bleibt für Wartung verfügbar. Netzwerk-Tuning darf nicht die Fahrquelle umschalten. Im WLAN-Tuningbetrieb BLE möglichst deaktivieren, sofern es nicht zur Fahrt gebraucht wird: Beide teilen eine Funkeinheit ([Espressif Koexistenz](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/coexist.html)).

## Messablauf und Fehlerverhalten

1. `capabilities`: Firmware-/Schema-Version, Kanalprofile, reservierbaren Puffer und Frequenzinformation abfragen.
2. `prepare`: Dauer (20–40 s) und Profil validieren, alle Recorderpuffer reservieren. PC öffnet die Zieldatei, verbindet den Datenkanal und bestätigt Empfangsbereitschaft. Noch keine Samples erzeugen.
3. `start(request_id)`: PC löst die Messung aus. Start an nächster definierter Erfassungsgrenze; Mess-ID und echter lokaler Startzeitpunkt werden bestätigt. Wiederholung derselben Request-ID startet keine zweite Messung.
4. `RECORDING`: Regler schreibt nur einen konsistenten Binärrecord in den reservierten Puffer. Kein Socketaufruf, keine dynamische Allokation, Flashschreibvorgänge oder CSV-Formatierung in diesem Pfad. Ein separater niedriger priorisierter Sender bündelt mehrere Records, beispielsweise bis 2–4 KiB oder maximal etwa 20 ms Wartezeit, und überträgt sie unabhängig.
5. Nach Ablauf der Dauer oder Stop endet die Erfassung anhand der Geräteuhr. `DRAINING` sendet die verbleibenden Daten und einen Abschlussrecord mit Anzahl, letztem Sample, tatsächlicher Dauer, Fehlern und Prüfsumme. Drain-Timeout begrenzt die Wartezeit.
6. PC prüft Block-/Sample-Sequenzen, Länge und Abschluss-Prüfsumme, schließt die Datei und bestätigt den vollständigen Empfang. Nur dann gilt die Aufnahme als vollständig. Erst danach CSV und Analyse. Bei Fehler bleibt die vorhandene Teilaufnahme mit Kennzeichnung erhalten.

**Transport:** TCP erledigt geordnete Zustellung und Wiederholungen innerhalb einer bestehenden Verbindung. Der Sender muss Teilwrites und Rückstau behandeln, mit nicht blockierenden Sockets bzw. kurzen begrenzten Wartezeiten und korrektem Retry bei `EAGAIN`. Ein Block bleibt bis zur vollständigen Übergabe an den TCP-Stack im Besitz des Senders. Erfolgreiches `send` bedeutet noch nicht, dass der PC die Daten gespeichert hat. Blöcke enthalten Mess-ID, Sequenz, Recordanzahl, Länge und Integritätsprüfung; der PC berücksichtigt, dass TCP keine Nachrichtengrenzen erhält. Der ganze Messstrom wird erst durch passenden Footer und PC-Abschlussbestätigung als vollständig bewertet. Espressif dokumentiert [nicht blockierende Sockets in lwIP](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/api-guides/lwip.html).

**Rückstau:** Der Producer wartet niemals auf den Sender. Ist der Puffer voll, endet nur die Aufnahme mit `BUFFER_FULL`; die Fahrt wird durch den Logger nicht gestoppt oder verzögert. Schon empfangene Daten bleiben analysierbar, gelten aber als unvollständig. Keine stillen Verluste, Überschreibung oder niedrigere Rate. Fehlerstatus separat aufbewahren, auch wenn kein Platz für ein Fehlerrecord mehr vorhanden ist.

**Verbindungsverlust:** Kurze Sendepausen bei bestehendem TCP werden im Puffer überbrückt. Bei erkanntem Verbindungsabbruch Aufnahme als fehlgeschlagen beenden; Erstversion implementiert bewusst keinen Wiederaufbau derselben Messung, da bereits an TCP übergebene Daten im Gerät nicht mehr vollständig vorliegen. Nach Reconnect Status lesen und neuen Versuch starten. Ebenso PC-Absturz, volle Festplatte und fehlender Abschluss führen zu einer Teilaufnahme, niemals zu einer scheinbar vollständigen Datei. Neustart wird durch eine neue Boot-ID sichtbar.

Die RC-Steuerung bleibt vom Datenkanal unabhängig. Ein Funk-Start hat variable Verzögerung; genau dokumentiert wird der lokale Erfassungsstart, nicht der Mausklickzeitpunkt oder PC-Empfang. Die Paketlaufzeit verändert die aufgezeichneten Zeitabstände nicht. Externe Uhrensynchronisation gehört nicht zum ersten Umfang.

## Welche Daten und welche Genauigkeit?

Erstversion zeichnet jeden tatsächlichen Reglerzyklus auf: Tick-ID, 64-Bit-Gerätezeit in µs, dt, Sollgeschwindigkeit/Lenkung/RC-Modus, Radwinkel und -geschwindigkeiten, Neigung und Sollneigung, relevante Gyrowerte, Speed-/Angle-PID-Anteile, optional Yaw, BodyX, Motorziele, Begrenzungs-/Fallflags und Akkuspannung. Kanalprofil und endgültige Recordgröße gemeinsam festlegen. Möglichst vorhandene Daten übernehmen, keine zusätzlichen Encoder-/ADC-Lesezugriffe pro Logsample.

Zeitliche Konsistenz: Snapshot nach abgeschlossener Berechnung, Sensorzeitpunkte bzw. Datenalter mitführen. Die Encoder werden nacheinander gelesen; keine Gleichzeitigkeit zur IMU behaupten. Motorziele sind keine gemessenen Ströme, Servo-Sollwerte keine gemessenen Positionen. Akku-ADC ist kein Sensor für die 3,3-V-Schiene. Rohdaten vor Softwarefilterung sind für Vibrationsanalyse wertvoll; Einheiten, Kalibrierung und Filterparameter gehören zur Datei.

Das binäre Format erhält explizite Feldtypen, Byteorder, Version und Recordlänge, keinen unversionierten C-Struct-Dump. Header: Build-Identität einschließlich lokalem Änderungsstand, Boot-ID, Parameter, ODR/Filter/Kalibrierung und Kanalliste. Footer: dt-Statistik, Überlauf-/Verlustzähler und Abschlussgrund. PC erzeugt CSV und Berichte. Analyse muss echte Zeitstempel verwenden; bestehende 20-Hz-Annahmen prüfen. Für Spektren unregelmäßige Abstände berücksichtigen, gegebenenfalls kontrolliert resampeln.

µs-Zeitstempel sind keine zugesicherte µs-Messgenauigkeit. ODR, Sensorauslese, Filterverzögerung, Kalibrierung und Nyquist-Grenze bleiben relevant. Die native Regleraufzeichnung ist erheblich aussagekräftiger als der heutige 20-Hz-Trace, garantiert aber nicht die Erkennung beliebig schneller Vibrationen.

**Erweiterung nur bei Bedarf:** separater IMU-Strom mit echten frischen 1-kHz-Samples aus dem FIFO. Laut [TDK-Datenblatt](https://invensense.tdk.com/wp-content/uploads/2022/12/DS-000347-ICM-42688-P-v1.7.pdf) gibt es ein 2-KB-FIFO mit Zeitstempeloptionen. FIFO blockweise auslesen, Überläufe erkennen und Sensorzeit auf MCU-Zeit abbilden. Ein einziger Treiber verwaltet den SPI-Zugriff; neue Pufferung darf die Sensorlatenz im Regler nicht unbemerkt erhöhen. Das FIFO ersetzt keinen Versuchsspeicher. INT1/2 sind in der Netzliste nicht an U4 angeschlossen; Polling oder ein nach Layoutprüfung ergänzter Anschluss wäre erforderlich. Wiederholtes Loggen desselben Sensorwertes zählt nicht als frische 1-kHz-Messung.

## Alternativen und Abbruchkriterien

| Ansatz | Entscheidung |
|---|---|
| BLE-Brücke | Verworfen als Hauptlösung: Fahrquellenkopplung, stark gedrosselte Fragmentübertragung und zusätzlicher Protokollumbau; vorhandenes BLE spart weniger Arbeit als erwartet. |
| WLAN/TCP + interner Sendepuffer | Bevorzugt für 20–40 s: volle Aufnahme auf dem PC, RAM überbrückt kurze Sendepausen. Nachweis von Durchsatz und Regeltiming erforderlich. |
| Vollständige Aufnahme im internen RAM | Als Hauptlösung verworfen: geforderte Dauer passt bei den vorgesehenen hochauflösenden Profilen nicht. |
| UDP-Streaming | Zunächst verworfen: Verlustbehandlung und Wiederholungen müssten selbst implementiert werden. Der unabhängige begrenzte Sender verhindert, dass TCP-Rückstau den Regler blockiert. |
| WLAN + PSRAM | Entfällt: am vorhandenen Roboter laut Nutzer kein externer RAM. |
| Interner Flash während Aufnahme | Für die einfache Erstversion verworfen: Schreib-/Löschpausen und Cacheabhängigkeiten erschweren Regeltiming. Dauerhaftigkeit wird nicht benötigt. |
| SD oder separater Logger | Nur neu bewerten, wenn Streaming unter Last nicht zuverlässig funktioniert. SD bringt Schreibpausen, Pins und Verkabelung; separater Logger entlastet Speicherung, benötigt aber eine geprüfte Datenschnittstelle. UART1/2 sind bereits Touch/SBUS zugeordnet, UART0 CH340; keinen freien Port voraussetzen. |

Flash-Cacheeinschränkungen hängen von SDK/Build ab ([Espressif Speicherhinweise](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/api-guides/external-ram.html)). Keine Flashschreibvorgänge während Aufnahme einplanen. Der Sendepuffer entkoppelt den Regler von Socket-Wartezeiten, beseitigt aber weder CPU-/Speicherbuslast noch WLAN-Interrupts oder Versorgungsspitzen.

Wenn 40 Sekunden ohne Lücken oder unzulässigen Reglerjitter nicht erreichbar sind, zuerst Sender-Bündelung, Taskprioritäten, Funkqualität und Speicherbudget prüfen. Hilft dies nicht, Hardwarelogger oder bewusst vereinbartes kleineres Kanalprofil neu bewerten. Nicht still auf 20 Hz zurückfallen, Laufzeit verkürzen oder Regleralgorithmen nur zur Erfüllung einer nominellen Loggingrate ändern.

## Umsetzung und Abnahme

1. Freien internen RAM nach WLAN-/Reglerinitialisierung und tatsächliche dt-Verteilung messen; 64-KiB-Pufferziel mit Betriebsreserve prüfen. Repräsentativen TCP-Lasttest durchführen, ohne Regelung für den Test zu ersetzen.
2. WLAN-Station, Steuer-API, separaten begrenzten Sender und Linux-Python-Client aufbauen. Parameter anwenden und zurücklesen, PC-Profile, Diagnoseverbindung unabhängig von Fahrt.
3. Recorder im nativen Regeltakt, Trigger-Zustände, Binärformat, PC-Datei und eindeutigen Abschluss integrieren. Bestehende Analyse auf neue Zeitbasis anpassen.
4. Unter Akku ohne USB Baseline gegen WLAN/Streaming vergleichen: p99/Maximum der Regeltaktzeit, Sensoralter, verlorene Samples, Pufferhöchststand, freien Speicher und Versorgung. Aufnahmezeiten 20/30/40 Sekunden einschließlich Fahrmanövern prüfen. Erst danach Vibrationsdaten kausal interpretieren.
5. IMU-FIFO nur hinzufügen, falls konkrete Analysefragen durch native Reglerdaten nicht beantwortet werden; dessen Datenrate zusätzlich budgetieren.

Erfolg: PC startet ohne USB genau eine Aufnahme; RC-Zuständigkeit bleibt gleich; wiederholte 40-Sekunden-Läufe ohne Sequenzlücken, mit konsistentem Footer und PC-Abschlussbestätigung; Konfiguration im Log stimmt mit Rücklesewerten überein. Vorläufiges Entwicklungsziel: direkte Recorderarbeit unter 5 % des gemessenen Regeltaktbudgets. Maximal zulässigen zusätzlichen Jitter anhand der Baseline festlegen und messen, keine harte Echtzeitgarantie aus Core-/Task-Trennung ableiten.

Gezielt prüfen: doppelter Start, künstlich langsamer Empfänger, Sendepause kürzer und länger als Pufferreserve, WLAN-/TCP-Abbruch, voller Puffer, volle PC-Festplatte, ungültige Parameter, Parametermutation während Messung, Abbruch beim Leeren und Neustart. Jeder unvollständige Lauf muss als solcher erkennbar bleiben; der Regler läuft bei Loggerfehlern weiter. Ein langsamer Plot darf nicht den PC-Empfang bremsen: Erstversion speichert binär und analysiert nach Ende.

Für die Umsetzung folgt ein Engineering-Plan mit `/plan-eng-review`. Anforderungen sind ausreichend konkret; erster technischer Entscheidungspunkt ist der Nachweis von Pufferbudget, Streamingdurchsatz und Regeltiming auf dem vorhandenen Roboter.
