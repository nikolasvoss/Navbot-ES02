# Record balance telemetry over Wi-Fi

The optional `balance_v1` recorder captures one 128-byte sample at each eligible balance-control tick. The Linux client saves the framed stream as `.nblog`, and the decoder exports CSV and a JSON quality report. The recorder supports the two-wheel master balance path and 20–40 second runs. It does not command the robot.

## Firmware build

The default build keeps recording disabled. In `src/ES-02/OllieFOCdrive`, copy `wifi_tuning_build.example.h` to the ignored local file `wifi_tuning_build.h`. Keep both `WIFI_TUNING_ENABLE` and `WIFI_RECORDING_ENABLE` set to `1`. Do not overwrite an existing local file; merge the two flags into it. Compile with the project's installed Arduino libraries and ESP32-S3 board profile. The recording build uses internal RAM and does not enable PSRAM.

From the repository root, compile to a separate output directory:

```bash
arduino-cli compile --libraries ~/Arduino/libraries --fqbn 'esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default,PartitionScheme=no_fs' --output-dir /tmp/navbot-wifi-recording src/ES-02/OllieFOCdrive
```

To install the resulting image, follow [the USB and serial instructions](../agent_notes/usb-serial.md) and use `python3 scripts/upload_firmware.py /tmp/navbot-wifi-recording` after checking the connected board and port.

The recording service reuses the unauthenticated Wi-Fi tuning service on HTTP port 80 and streams on the advertised TCP port, currently 8766. The station must already be provisioned. HTTP and TCP are plaintext and are intended for a trusted development WLAN.

The recording service starts after the robot joins the provisioned WLAN. It does not require a terminal token or other bearer credential. The recording data stream still uses its short-lived per-session ticket to associate a client with the active capture.

## Linux client

Create `~/.config/navbot/wifi-tuning.json` with the robot's IPv4 address. The tuning client creates this file on first use and sets mode `0600`:

```json
{"host":"192.168.1.20"}
```

```bash
mkdir -p ~/.config/navbot
chmod 700 ~/.config/navbot
chmod 600 ~/.config/navbot/wifi-tuning.json
python3 scripts/wifi_record.py --status
python3 scripts/wifi_record.py --seconds 30 --output trial.nblog
python3 scripts/decode_wifi_trace.py trial.nblog --csv trial.csv --report trial-summary.json
```

The output path must be in an existing directory. The recorder refuses to replace an existing output, partial file, or error report unless `--force` is passed. A validated full-duration run exits 0. Input/configuration errors exit 2, transport failures exit 3, device refusals exit 4, and interrupted or user-stopped runs exit 5. A valid user-stopped recording is saved but remains classified as shorter than the requested duration.

Failed transfers retain `trial.nblog.partial` and write `trial.nblog.error.json`. The decoder rejects corrupt or truncated files by default. Pass `--allow-partial` to export only the valid prefix; the command still exits nonzero and the report remains marked partial.

## Evidence limits

Host tests and firmware builds do not establish robot timing or Wi-Fi throughput. No hardware acceptance is implied. The metadata reports a recorder build ID but does not include a source hash. Verify configuration, heap headroom, ring occupancy, control timing and representative movement on hardware before relying on recordings for experiments.
