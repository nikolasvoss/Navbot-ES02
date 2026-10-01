# Record balance telemetry over Wi-Fi

The optional `balance_v1` recorder captures 128-byte samples at up to 100 Hz. Sample timestamps preserve the actual intervals, and `control_dt` records the controller interval at each captured sample. Firmware batches five samples per sender wake, with a 60 ms maximum wait before flushing a partial batch. The Linux client saves the framed stream as `.nblog`, and the decoder exports CSV and a JSON quality report. The recorder supports the two-wheel master balance path and 20–40 second runs. It does not command the robot.

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

Live runs on 30.09.2026 showed that sample frequency and sender wake frequency affect the control-loop timing. With CH5 off, the 100 Hz capture before batching had a `control_dt` median of 2.354 ms. Batching notifications without extending the sender wait produced a 2.397 ms median and ring high-water mark 2, so it did not form five-record batches. The verified batching build waits up to 60 ms and reached ring high-water mark 5. Its CH5-off run contained 1,846 records with `control_dt` median 2.032 ms and p95 2.813 ms. That is close to the earlier no-recording CH5-off K58 median of 2.104 ms.

A 20-second CH5=1 recording on the batching build contained 1,768 active records, with median `control_dt` 2.128 ms, mean 2.316 ms, and p95 2.948 ms. It had ring high-water 5 and no CRC, sequence, numeric, or overflow faults. The CH5-off recording on the same build had median `control_dt` 2.032 ms and p95 2.813 ms. This comparison changes the controller state along with CH5, so it does not isolate logging overhead.

A same-mode CH5=1 comparison later used K58 at 20 Hz with the recorder idle, then Wi-Fi capture at about 89 Hz. Both runs used boot ID `eac66ebaef436e41`. The idle baseline had median `control_dt` 2.122 ms, p95 2.762 ms, and maximum 3.223 ms. The Wi-Fi recording had median 2.117 ms, p95 2.892 ms, and maximum 4.081 ms. The median was 5 µs lower during Wi-Fi recording; p95 was 130 µs higher. The trace windows were about 4.3 minutes apart, and their sample rates differ. Treat these results as an estimate, not proof that logging has no effect. No movement test was run, and the user had reported a change in perceived behavior. The metadata reports a recorder build ID but does not include a source hash. See [the hardware observations](../agent_notes/wireless-logger-hardware.md#wlan-aufnahmen-am-aufgebauten-roboter-am-30092026) before using recordings for experiments. Raw files and the comparison report are under `/tmp/navbot-verify/20260930-ch5-comparison/`.
