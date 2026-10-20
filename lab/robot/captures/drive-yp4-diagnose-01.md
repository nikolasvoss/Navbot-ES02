# DRIVE-Vergleich

| Log | gültig/defekt | aktiv | Startneigung | CH3-Zeilen | Stopps | Gegenlauf max. | Winkel-I max. | Motorziel max. | Board-ADC min. |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| `drive-yp4-diagnose-01.log` | 872/0 | 811 | -0.041° | 372 | 2 | 8.733 | 20.0 | 88.0 | 4.833 V |

## Auswertbarkeit

- `drive-yp4-diagnose-01.log`: Stoppereignis vorhanden; für einen Vergleich Impulsstärke und -dauer beachten.

## Einzelne Stopps

| Log | Richtung | CH3 max. | Impulsdauer | Tempo vor Loslassen | Gegenlauf max. |
|---|---|---:|---:|---:|---:|
| `drive-yp4-diagnose-01.log` | rueckwaerts | 30.0 | 1.76 s | -33.328 | 8.733 |
| `drive-yp4-diagnose-01.log` | rueckwaerts | 30.0 | 0.56 s | -43.446 | 0.051 |

Gegenlauf max. ist der größte Betrag der entgegengesetzten mittleren Raddrehzahl innerhalb 1 s nach CH3→neutral; Einheit wie im Firmware-Trace. Unterschiedliche Impulse sind kein direkter A/B-Vergleich. Die Kennzahl beschreibt den Trace, nicht die Ursache oder das sichtbare Fahrverhalten.
