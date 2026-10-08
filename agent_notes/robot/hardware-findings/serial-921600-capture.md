# CH340 capture at 921600 baud produced corrupt rows

Follow-up setting: the user selected 576000 baud for current use after this capture and asked that the physical UART rate be measured in the future. No 576000-baud hardware measurement has been performed; treat it as provisional. A future firmware improvement could let captures select the channels they need to avoid transmitting unused data points. That software change is not part of this hardware observation.

## Observation

On 2026-10-07, the Navbot-ES02 was flashed with the diagnostic image and captured through `/dev/ttyUSB0`, identified as CH340 USB VID:PID `1a86:7523`. Serial MCP used 921600 baud, 8N1, no flow control, and DTR/RTS low. The diagnostic image keeps motor drivers disabled and skips balance and servo setup.

The raw byte capture is available locally at `/tmp/navbot-serial-rate-921600.bin` (900,000 bytes). It spans 95.54 seconds of device `time_us` timestamps, exceeding the requested 60 seconds.

## Evidence

- Valid CSV: 7,824 rows, each with 17 numeric fields; valid receive rate 81.89 rows/s.
- Device sequence span: 9,555 producer records over the timestamp span, or 100.01 records/s inferred from device IDs.
- Malformed numeric lines: 1,714; missing sequence IDs between valid rows: 1,731 across 125 detected gaps.
- Successive valid-row timestamp intervals: minimum/median/p95 10,000 µs; maximum 7,450,000 µs.
- Firmware snapshots reported zero queue drops and zero UART write failures.
- An earlier UTF-8 read attempt failed with an invalid byte sequence. The hex-mode capture contained 900,000 ASCII-range bytes but numerous missing separators and merged lines.

## Practical consequence and limits

Reception at 921600 baud is not reliable on the tested CH340/host/MCP path and is not accepted for data analysis. The zero firmware-side error counters and missing sequence IDs place the observed loss after producer enqueue and UART write accounting, but do not isolate whether the CH340 bridge, physical connection, host driver, or MCP capture decoder is responsible. No lower-baud comparison was run, and no baud fallback was added.
