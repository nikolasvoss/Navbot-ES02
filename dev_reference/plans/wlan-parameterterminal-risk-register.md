# Risikoregister: WLAN-Parameterterminal

Stand 30.09.2026. Risiko: **high** wegen unauthentifiziertem Netzwerkzugriff auf physische Regelparameter. Der Nutzer akzeptiert dies ausdrücklich für das Entwicklungstool; vor normalem Betrieb bleibt das WLAN-Feature per Build-Schalter auszuschalten. Reale Hardwareprüfung ist offen.

| Risiko / Codebeleg | Auswirkung | Vorgabe und Nachweis |
|---|---|---|
| Dirty Baseline; aktuelle V-/Priming-Änderungen im Hauptsketch | Versehentlicher Verlust funktionierender Fahranpassungen | Snapshot, Manifest, eigener Basis-Commit im Worktree; Original unangetastet |
| HTTP-Callback mutiert Gains parallel zur Regelung | Inkonsistente Reglerzustände | Begrenzte POD-Queue, Loop besitzt Mutationen und Snapshots; Batchtest |
| Veraltetes CH5-OFF; `sbus_dt_ms` ist nur Intervall | Schreiben während aktiver/unbekannter Fahrt | Gültige Frames zeitstempeln; Freshness/Failsafe/CH5 bei Ausführung prüfen |
| `PidParameterTuning==0` ruft `PidParameter()` auf | Bestätigte Werte werden überschrieben | U-Modus sichtbar, Gainwrite in Automatik ablehnen; Batch U1 testen |
| `SpeedPid` und `Speed_Pid` haben verschiedene Skalierung | Falsche Gainwirkung | Gleiche Konfigurationsobjekte wie Serial, keine direkte Effektivgainmutation |
| `CtrlInput()` koppelt BLE-Verbindung an Fahrt | Tuning verändert Steuerquelle | Eigenständiger WLAN-Pfad; BLE im WLAN-Tuning-Build aus, Legacybuild erhalten |
| Parameterterminal ist unauthentifiziert | Jeder erreichbare WLAN-Client kann lesen und freigegebene Werte schreiben | Bewusste Entwicklungsentscheidung; Schreib-Allowlist sowie SBUS-/CH5-Sperren bleiben aktiv; WLAN-Feature beim normalen Build deaktiviert lassen |
| Timeout nach angewandtem Write | Client behauptet fälschlich Fehlschlag oder wiederholt | IDs/Boot-ID, kurze Gültigkeit, Abschlussablage, keine automatischen Writes; Zustand nachlesen |
| WLAN-Task-Stack bei Request-Parsing | Gültiger POST löste `Stack canary watchpoint triggered (wifi-tuning)` aus; Ursache war ein 4,8-KiB-Key-Array im Duplikat-Scanner | Scanner speichert Offsets/Längen statt Keykopien; Host-Stackmaß 4.944 → 480 B, Hardware-Retest des Fix-Builds steht noch aus |
| Offener SoftAP bei WLAN-Provisionierung | Geräte in Funkreichweite können WLAN-Daten provisionieren oder verändern | Security 0 bleibt als Entwicklungssetup; nur in kontrollierter Umgebung provisionieren |
| HTTP ohne TLS und Authentifizierung | WLAN-Clients können API-Aufrufe beobachten, senden und nachahmen | Bewusste Entwicklungsentscheidung; Feature bei normalen Firmwarebuilds abschalten und Routerports nicht freigeben |
| Eingangslimits wirken scheinbar physisch sicher | Ungeeignete Gains trotz gültiger Zahlen | Als technische Grenzen dokumentieren; nur CH5 OFF, keine automatischen Fahrversuche |
| Profilladen teilweise wirksam | Nicht reproduzierbare Konfiguration | Vollständige Validierung und atomarer Batch, keine Einzelcommand-Schleife |

Der aktuelle Stand wurde nach Entfernung der Bearer-Authentifizierung manuell auf Zugriffsgrenzen und Diff geprüft. Lokale Tests und Feature-on/off-Builds sind dokumentiert. Kein Flash/Upload. Laufzeitfehler lassen bestehenden RC-/Regelbetrieb unverändert; Parameteränderungen behalten SBUS-/CH5-Prüfungen. Unbelegte Hardwareabnahme bleibt offen und verhindert die Behauptung eines vollständig getesteten Fahrbetriebs.
