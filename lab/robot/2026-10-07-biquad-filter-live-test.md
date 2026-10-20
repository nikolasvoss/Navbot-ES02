# Biquad filter live diagnostic capture

Date: 2026-10-07

## Observation

The ESP32-S3 sensor-diagnostic image ran with the CH340 USB-UART connection at 576000 baud and the diagnostic frame interval set to 10 ms. The robot was confirmed secured by the user. The diagnostic source keeps both motor-driver enable pins low and exits setup before initializing BLE, FOC, servos, or balance control.

A 60000-byte capture contained 250 complete 25-column numeric rows. Every row reported `imu_ok=1`. The 249 timestamp intervals had a 10.000 ms median, a 9.966 ms minimum, and a 10.040 ms maximum. No interval exceeded 15 ms.

For this stationary capture, the filtered gyro-X standard deviation was 0.641 of the unfiltered value. The filtered acceleration-Z standard deviation was 0.733 of the unfiltered value. Raw battery readings ranged from 7.250077 to 7.307390 V; filtered readings ranged from 7.262542 to 7.293375 V.

## Evidence

The CH340 appeared as `/dev/ttyUSB0` with USB VID 1A86 and PID 7523. The serial MCP opened that port at 576000 baud, explicitly set RTS and DTR low, and read the CSV stream. A final 60000-byte read was parsed into complete rows; timing came from column 25, IMU readiness from column 24, and signal statistics from the documented raw and filtered channels.

The diagnostic build was flashed from `/tmp/navbot-filter-live/output`. The flash tool reported verified hashes for the bootloader, partition table, and application. No normal control image was flashed.

## Practical consequence and limits

The measured timestamps confirm that the diagnostic stream sustained the configured 100 Hz interval during this stationary capture, with no observed gap above 15 ms. The lower filtered signal deviations show stationary smoothing. This is not a controlled 20 Hz frequency-response measurement. The normal-mode IMU cadence, behavior under motor load, and the physical cutoff under controlled vibration remain unmeasured.
