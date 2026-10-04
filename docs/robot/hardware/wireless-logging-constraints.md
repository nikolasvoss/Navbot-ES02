# Hardwarebefunde für kabelloses Logging

Stand 28.09.2026; Dokumentenvergleich und anschließende Hardwarekorrektur durch den Nutzer, keine eigene Messung.

## PSRAM und GPIO35–37

Beobachtung: [PCB-Netzliste](electronics/netlist.md), U4, nennt ESP32-S3-WROOM-1-N8R8 und verbindet zugleich IO35/36 mit LED2/3 sowie IO37 mit EN_B. Die Firmware verwendet diese GPIOs ebenfalls. Laut [Espressif-Datenblatt, Abschnitt 3, Fußnote b](https://documentation.espressif.com/esp32-s3-wroom-1_wroom-1u_datasheet_en.pdf) stehen IO35–37 bei Octal-PSRAM-Modulen nicht für andere Zwecke zur Verfügung.

**Korrektur durch den Nutzer am 28.09.2026:** Bestückt ist ESP32-S3-WROOM-1, es gibt keinen externen RAM, und die Pinbelegung ist korrekt. Dies ist die maßgebliche Angabe zum aufgebauten Roboter. Die N8R8-Bezeichnung der exportierten Netzliste bleibt als abweichende Designangabe erhalten; daraus keinen realen PSRAM-Pinkonflikt ableiten.

Konsequenz: Mit internem SRAM planen, PSRAM deaktiviert lassen und keine Pinänderung vorsehen. Die tatsächlich freie interne Pufferkapazität unter Firmware-/WLAN-Last muss noch gemessen werden. Die Flashgröße ist aus der Basisbezeichnung allein nicht bestimmt.

## IMU-Interrupts

Beobachtung: Die exportierte U5-Belegung enthält INT1/INT2, U4 aber keine entsprechenden Netze; R28 zieht INT2 mit 10 kΩ nach GND. Der vorhandene IMU-Treiber liest Datenregister über SPI und nutzt keinen FIFO-/Interruptpfad.

Konsequenz: Kein vorhandener ESP-Interruptanschluss annehmen. Für FIFO-Erfassung entweder begrenztes Polling entwickeln oder nach Prüfung von Layout und realer Platine einen geeigneten Anschluss planen.

Das [Konzept](../software/proposals/wifi-logging/concept.md) beschreibt Speicherbudget und Entscheidungspunkte.
