# HOTRC HT-10A – Fernsteuerung

Die Fernsteuerung ist über den Empfänger per **SBUS** mit dem NavBot verbunden. Die Firmware wertet die Kanäle wie folgt aus (SBUS-Kanäle werden hier ab 1 gezählt):

| Kanal | Funktion |
| --- | --- |
| CH1 | Körperrollen / seitliche Neigung |
| CH2 | Beinlänge; im Modus „Pitching Adjust“ stattdessen Nickwinkel |
| CH3 | Fahrgeschwindigkeit vor/zurück |
| CH4 | Drehung / Lenken |
| CH5 | PID-Regelung: aus, ein ohne Touchscreen, ein mit Touchscreen (3 Stufen) |
| CH6 | Haltungsmodus oder Markiermodus (2 Stufen) |
| CH7 | Rollregelung manuell oder automatisch (2 Stufen) |
| CH8 | Fahr-/Haltungsmodus: Standard, Pitching Adjust oder Ball-Balance (3 Stufen) |
| CH9 | X-Zielposition des Balls im Ball-Balance-Modus |
| CH10 | Y-Zielposition des Balls im Ball-Balance-Modus |

**Hinweis zur HT-10A:** Die Tabelle beschreibt die vom Roboter erwarteten SBUS-Kanäle, nicht die fest verdrahtete Position bestimmter HT-10A-Schalter. Welche Knüppel, Schalter oder Drehregler CH1–CH10 senden, hängt von der Senderkonfiguration ab. Bei Änderungen am Sender die Kanalzuordnung im Empfänger-/Servomonitor prüfen.

Die Beschreibung entspricht der aktuellen SBUS-Auswertung in `src/ES-02/OllieFOCdrive/OllieFOCdrive.ino` und den Modus-Konstanten in `OllieFOCdrive.h`.
