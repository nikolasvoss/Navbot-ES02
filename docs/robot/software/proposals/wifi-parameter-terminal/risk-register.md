# Risikoregister: WLAN-Parameterterminal

Stand 28.09.2026. Engineering-Planstatus: **clean** für definierten Phase-1-Scope. Risiko: **high** wegen Netzwerkzugriff auf physische Regelparameter. Dies ist kein bestandener Implementierungs-/Sicherheitsreview. Keine offenen Produktentscheidungen erforderlich; reale Credentials und Hardwareprüfungen werden erst bei Einrichtung relevant.

| Risiko / Codebeleg | Auswirkung | Vorgabe und Nachweis |
|---|---|---|
| Dirty Baseline; aktuelle V-/Priming-Änderungen im Hauptsketch | Versehentlicher Verlust funktionierender Fahranpassungen | Snapshot, Manifest, eigener Basis-Commit im Worktree; Original unangetastet |
| HTTP-Callback mutiert Gains parallel zur Regelung | Inkonsistente Reglerzustände | Begrenzte POD-Queue, Loop besitzt Mutationen und Snapshots; Batchtest |
| Veraltetes CH5-OFF; `sbus_dt_ms` ist nur Intervall | Schreiben während aktiver/unbekannter Fahrt | Gültige Frames zeitstempeln; Freshness/Failsafe/CH5 bei Ausführung prüfen |
| `PidParameterTuning==0` ruft `PidParameter()` auf | Bestätigte Werte werden überschrieben | U-Modus sichtbar, Gainwrite in Automatik ablehnen; Batch U1 testen |
| `SpeedPid` und `Speed_Pid` haben verschiedene Skalierung | Falsche Gainwirkung | Gleiche Konfigurationsobjekte wie Serial, keine direkte Effektivgainmutation |
| `CtrlInput()` koppelt BLE-Verbindung an Fahrt | Tuning verändert Steuerquelle | Eigenständiger WLAN-Pfad; BLE im WLAN-Tuning-Build aus, Legacybuild erhalten |
| Eingabe ungültig/zu groß oder unauthentifiziert | Heapmangel oder ungewollte Parameter | Allowlist, Finite-/Bereichsprüfung, 2-KiB-Limit, Auth für alle Endpunkte, Queuelimits |
| Timeout nach angewandtem Write | Client behauptet fälschlich Fehlschlag oder wiederholt | IDs/Boot-ID, kurze Gültigkeit, Abschlussablage, keine automatischen Writes; Zustand nachlesen |
| WLAN-Task/Server blockiert oder Lastspitzen | Reglerjitter/Watchdog/Versorgungseinbruch | Keine Netzwerkarbeit im Regler, blockierendes Task-Warten statt Busyloop, Last-/Hardwarevergleich |
| Token/SSID-Passwort in Git oder Ausgaben | Fremdzugriff im Netz | Ignorierte lokale Secrets, Dummy-Beispiele, keine Credentials in Logs/CLI/URL |
| HTTP ohne TLS | Keine Vertraulichkeit gegenüber Netzangreifern | Bewusste Heimnetzgrenze, Bearer-Token, keine externe Freigabe; keine TLS-Sicherheit behaupten |
| Eingangslimits wirken scheinbar physisch sicher | Ungeeignete Gains trotz gültiger Zahlen | Als technische Grenzen dokumentieren; nur CH5 OFF, keine automatischen Fahrversuche |
| Profilladen teilweise wirksam | Nicht reproduzierbare Konfiguration | Vollständige Validierung und atomarer Batch, keine Einzelcommand-Schleife |

Erforderliche Folgestufen: `/security-review` (high), `/cli-qa` full, `/review`, `/document-release`. Kein Ship/Flash in diesem Auftrag. Laufzeitfehler lassen bestehenden RC-/Regelbetrieb unverändert; Tuning kann abgewiesen/deaktiviert werden. Unbelegte Hardwareabnahme bleibt sichtbar offen und verhindert die Behauptung eines vollständig getesteten Fahrbetriebs.
