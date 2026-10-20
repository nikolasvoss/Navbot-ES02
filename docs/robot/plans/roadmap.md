# Roadmap

## Verify sensor data and filters

The ESP32 captures IMU data with monotonic timestamps. In the safe diagnostic mode, the motor drivers stay disabled; the firmware sends IMU, battery, and SBUS values as CSV lines over the existing CH340 UART connection. SerialPlot provides the live display and can save CSV snapshots. Its ASCII reader reads unprefixed, comma-separated numeric values; the prefix filter stays disabled. The saved CSV data can then be evaluated with an FFT tool; according to its feature list, SerialPlot itself offers no FFT plots. The diagnostic mode computes no controller outputs. Filtering and resonances are assessed from the captured time series; the control cycle and behavior under motor load must be measured separately.

Filtering runs digitally on the ESP32 and partly in the ICM42688. Gyro and accelerometer filters, as well as complementary filtering or sensor fusion, are evaluated from the recordings. The specific filter values and acceptance criteria will be set later. For high data rates, transmission is buffered. Binary frames will be used if measurements show that CSV over the available serial port does not reliably reach the target rate. A DAC is normally not needed for this.

## Next steps

1. Define limits for target rate, timing deviation, data loss, and permissible delay of the control cycle. Before measurements in control operation, determine a safe test procedure. The current diagnostic mode keeps the motor drivers disabled and computes no controller outputs.
2. Using longer CSV recordings, check the actual rate, time intervals, and data gaps against these limits. Measure the influence on the control cycle only with the defined test procedure.
3. Decide whether controller outputs must be recorded. This requires a method that keeps the motor drivers disabled or explicitly allows a safe test in control operation.
4. Compare unfiltered and filtered IMU values, as well as sensor fusion, using repeatable recordings and offline FFT. Then set filter values and acceptance criteria.
5. Check native USB CDC on the existing ESP32-S3 board. So far only the CH340 USB UART has been observed at the USB-C connector. The data format stays separate from the transport.
6. Introduce buffering or binary frames only when measurements show that CSV does not reliably reach the defined data rate.
7. Later, check a tool that shows the FFT of a selected measurement channel from CSV recordings. A live FFT would need its own serial client, since SerialPlot does not provide FFT plots.
8. Clean up logging commands. Clearly document the selection of measured values, output formats, and behavior in diagnostic and normal mode.

## Clean up controller start values and diagnostic switches

- `DIAGNOSTIC_LIVE_TUNING_DEFAULTS` bundles behavior for normal driving: gain selection and live tuning, CH3 scaling, and the time base of the wheel speed. During the later cleanup, make these responsibilities explicit. Keep the existing driving behavior until comparison measurements support changes. This switch is independent of `SENSOR_DIAGNOSTIC_MODE`.

## IMU diagnostic measurements

- Measure after decoupling IMU processing from the 100 Hz CSV output: `IMUtime_dt` distribution in normal and sensor diagnostic mode; timestamp intervals, sequence gaps, and logger drop/write errors in the diagnostic CSV; and filter response at stationary and controlled known frequencies. For frequency measurements, use a capture rate matched to the target bandwidth and anti-alias filtering. The current diagnostic remains motor-free and does not establish behavior under motor load.

These measurements matter because different update rates and filter settings can change smoothing, latency, and fusion values. The 100 Hz CSV output can alias raw signals above 50 Hz, so fast vibrations appear wrong or remain invisible. Without this comparison, diagnostic recordings establish neither normal IMU processing nor behavior under motor load.

### Software IMU cutoff frequency (50 Hz)

Measure the actual IMU update rate in normal control operation and compare the current software cutoff frequency of 50 Hz with suitable alternatives. Record raw and filtered gyro and acceleration data at controlled frequencies, and evaluate attenuation and delay of the attitude fusion. Set the cutoff frequency based on the measured motion and noise spectrum, not on the nominal sensor sampling rate alone.

## Driving stability at high speed

At high speed the robot still tips over. Record the failure pattern in a safe, reproducible driving test, covering roll angle, target and actual wheel speed, effective speed setpoint, and the balance controller output. Check whether the existing tilt reduction takes effect in the observed case and engages early enough. Change thresholds and controller tuning only after this evaluation.
