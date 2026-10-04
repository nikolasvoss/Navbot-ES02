# Testplan: WLAN-Parameterterminal

Stand 28.09.2026; gehört zum [Implementierungsauftrag](implementation-brief.md). Planung, noch keine Testergebnisse. QA: Firmware/intern plus neue CLI, keine UI. Keine Recorder-/Streamingtests in Phase 1.

## Automatisierte Tests

| Bereich | Entscheidende Fälle | Erwartung |
|---|---|---|
| Eingaben | PP5, PD0.12, SP0.045, V0.2, Abfrage; Randwerte; NaN/Inf, Resttext, unbekannter Name, überlange Zeile | Exakte Interpretation oder eindeutige Ablehnung, kein Clamp |
| Registry | Alle 18 Parameter, Konfigurationsobjekte statt effektiver /100-Werte, V außerhalb 0…0,4 | Semantik wie Serial; keine Reglerformeländerung |
| Mutation | CH5 an/aus, Fallflag bei CH5 an, kein Frame seit Boot, 249/251 ms altes Frame, Lost/Failsafe, nicht unterstützter Modus | Nur gültig/frisch/CH5 OFF erlaubt; Check bei Anwendung, nicht nur Empfang |
| Autorace | Bei Enqueue OFF, vor Ausführung ON; abgelaufener Request | Keine Änderung |
| Automatik | Gainwrite bei U0, U1 plus Gains als Batch, U0 plus Gains, unabhängiges V | AUTO_MODE bzw. atomar definierter Erfolg |
| Batch | Erstes Feld gültig, späteres ungültig; unbekanntes Feld; doppelte JSON-Keys | Kein einziger Wert geändert |
| Zustände | Snapshot während Serialänderung/Batch; Neustart-/Boot-ID; doppelter Request gleich/anders | Kohärenter Snapshot, kein doppelter Effekt, klare Fehler |
| Begrenzung | Queue voll, langsamer HTTP-Client, Teilrequest, Oversize, Acktimeout | Ressourcen begrenzt; Regelpfad wartet nicht auf Netzwerk |
| Auth | Fehlender/falscher Token für GET/POST, korrektes Token | Kein unauthentifizierter Zugriff; keine Tokens im Output |
| Client | Mockserver: Erfolg, 401/409/503/504, Timeout nach möglicher Anwendung, defektes JSON, falsche Boot-ID | Passende Exitcodes, keine automatische Mutation/Wiederholung |
| Profile | Save/Load, Force, fehlender Pfad, ungültige Version, Secrets/Unbekanntes, halb ungültiges Profil | Atomare Datei/Batch, keine Teiländerung oder Secretkopie |

Firmware-Registry/Transaktionslogik möglichst Arduino-unabhängig testen, mit realem Parser/Validator und injizierten Zuständen/Uhr. Nicht nur eine Python-Nachbildung der Firmware testen. Host-C++-Harness genügt, kein großes neues Testframework erforderlich. Netzwerkgrenzen zusätzlich per HTTP-Mock für Client und später am Gerät prüfen.

## Build-/Regressionsnachweis

1. Snapshot-Baseline, Feature aus, Feature an mit Dummy-Secrets bauen; gleiche installierte Core-/Bibliotheksversionen und USB-Optionen. Kein PSRAM.
2. Bestehende Python-Analyseskripttests ausführen, soweit verfügbar. Keine Messlog-Schemas/Reglerdefaults verändern.
3. Diff prüfen: aktuelle V-Rückführung, Priming, Geschwindigkeitszeitbasis und CH3-Skalierung unverändert. Serialbefehle erhalten; kein motorischer Fernsteuerendpunkt hinzugefügt.
4. `/cli-qa` full: Hilfe, Einzelbefehl, REPL, EOF/Ctrl-C, JSON-Ausgabe, Profile und Fehlercodes mit zeitlich begrenztem Mockserver. `/security-review` und `/review` protokollieren.

## Hardwareabnahme, nur nach gesondert autorisiertem Upload

- Vor Flash Port/Board nach USB-Notiz identifizieren, vorhandenen Imagepfad für Rückkehr festhalten. Flash erst nach separatem Auftrag.
- Ohne WLAN/bei falschen Zugangsdaten: Boot, Serial und SBUS funktionieren; keine Reconnect-Warteschleife im Reglerpfad.
- Mit WLAN und CH5 OFF: U lesen, U1 setzen, PP/PD/SP/V ändern, zurücklesen; äquivalente Serialabfrage bestätigt dieselben Werte. Profile nur bewusst mit gültigen Werten laden.
- CH5 ON: Netzwerkschreiben abgelehnt; Lesen möglich. Frische-/Failsafe-Prüfung zuerst mit abgestütztem Roboter, keine riskanten Fahrversuche zur Netzwerk-QA.
- Serial-/WLAN-Wechsel, WLAN-Reconnect: keine spontane Gain-/Modusänderung. Netzwerkverbindung übernimmt nicht die Fahrsteuerung.
- Regeltakt ohne/mit WLAN sowie unter begrenzter Requestlast vergleichen: Median, p99, Maximum, Reset-/Watchdogereignisse und Speicherreserve. Zulässige Abweichung anhand Baseline vor Fahrfreigabe bewerten, keine isolierte Durchschnittsrate als Beweis verwenden.
- Neustart: Builddefaults wiederhergestellt, PC-Profil bleibt Datei, keine automatische Wiederanwendung.

Ohne Board: automatische Ergebnisse liefern und diese Punkte explizit als **nicht ausgeführt** markieren. Nicht den gesamten Auftrag wegen fehlender Hardware stoppen.
