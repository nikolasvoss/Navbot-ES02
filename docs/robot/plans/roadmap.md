# Roadmap

## Verify sensor data and filters

The ESP32 captures IMU data with monotonic timestamps. In the safe diagnostic mode, the motor drivers stay disabled; the firmware sends IMU, battery, and SBUS values as CSV lines over the existing CH340 UART connection. SerialPlot provides the live display and can save CSV snapshots. Its ASCII reader reads unprefixed, comma-separated numeric values; the prefix filter stays disabled. The saved CSV data can then be evaluated with an FFT tool; according to its feature list, SerialPlot itself offers no FFT plots. The diagnostic mode computes no controller outputs. Filtering and resonances are assessed from the captured time series; the control cycle and behavior under motor load must be measured separately.

Filtering runs digitally on the ESP32 and partly in the ICM42688. Gyro and accelerometer filters, as well as complementary filtering or sensor fusion, are evaluated from the recordings. The specific filter values and acceptance criteria will be set later. For high data rates, transmission is buffered. Binary frames will be used if measurements show that CSV over the available serial port does not reliably reach the target rate. A DAC is normally not needed for this.

## Next steps

1. Define limits for target rate, timing deviation, data loss, and permissible delay of the control cycle. Before measurements in control operation, determine a safe test procedure. The current diagnostic mode keeps the motor drivers disabled and computes no controller outputs.
2. Using longer CSV recordings, check the actual rate, time intervals, and data gaps against these limits. Measure the influence on the control cycle only with the defined test procedure.
3. Decide whether controller outputs must be recorded. This requires a method that keeps the motor drivers disabled or explicitly allows a safe test in control operation.
4. Compare unfiltered and filtered IMU values, as well as sensor fusion, using repeatable recordings and offline FFT. Then set filter values and acceptance criteria.
6. Introduce buffering or binary frames only when measurements show that CSV does not reliably reach the defined data rate.
7. Later, check a tool that shows the FFT of a selected measurement channel from CSV recordings. A live FFT would need its own serial client, since SerialPlot does not provide FFT plots.
8. Clean up logging commands. Clearly document the selection of measured values, output formats, and behavior in diagnostic and normal mode.

## Clean up controller start values and diagnostic switches


## IMU diagnostic measurements

- Measure after decoupling IMU processing from the 100 Hz CSV output: `IMUtime_dt` distribution in normal and sensor diagnostic mode; timestamp intervals, sequence gaps, and logger drop/write errors in the diagnostic CSV; and filter response at stationary and controlled known frequencies. For frequency measurements, use a capture rate matched to the target bandwidth and anti-alias filtering. The current diagnostic remains motor-free and does not establish behavior under motor load.

These measurements matter because different update rates and filter settings can change smoothing, latency, and fusion values. The 100 Hz CSV output can alias raw signals above 50 Hz, so fast vibrations appear wrong or remain invisible. Without this comparison, diagnostic recordings establish neither normal IMU processing nor behavior under motor load.

### Software IMU cutoff frequency (50 Hz)

Measure the actual IMU update rate in normal control operation and compare the current software cutoff frequency of 50 Hz with suitable alternatives. Record raw and filtered gyro and acceleration data at controlled frequencies, and evaluate attenuation and delay of the attitude fusion. Set the cutoff frequency based on the measured motion and noise spectrum, not on the nominal sensor sampling rate alone.

## Speed up the main loop pass

One `loop()` pass takes about 1.4 ms, so the control tick runs at about 710 Hz instead of 1 kHz and the IMU is read at about 680 Hz while it outputs 1 kHz ([measurements](../../../lab/integration/2026-10-25-odometry-signs-and-rate.md#2026-10-26-update)). About 30% of IMU samples are overwritten before they are read, and the IMU biquad filters, designed for `kSampleRateHz` = 1000, run with cutoffs about 30% lower than configured. The balance controller and the Mahony filter use measured `dt`, so the rate does not break their math, but the balance gains were tuned on this slow loop.

The ros2_pc link and SLAM are not affected: `WheelOdometry` and `Imu` go out on a 20 ms schedule at 50 Hz, wheel angles are cumulative, and both messages carry `t_us`. Only the extra lag in the filtered IMU values reaches `/imu/data`.

Measured so far: each AS5600 read is a 120 µs I²C burst at 400 kHz, so both encoders take about 0.24 ms of bus time per pass. The SCL rise time is 190 ns, fine for 400 kHz but too slow for 1 MHz (limit 120 ns) without stronger pull-ups. The control tick itself takes about 0.36 ms. The remaining ~0.75 ms per pass (I²C driver overhead, the IMU SPI read, FOC math, S.BUS, the command router) is not yet measured.

1. Add a profiling build behind a compile-time switch, off by default: time `move()`, each `loopFOC()`, `imu::service()`, `readSbus()`, the command router and the tick with `micros()`, and report mean and maximum once per second through the logging task.
2. Cheaper reads without behavior change, chosen from the profile. Candidate: read the IMU with one SPI transaction per access (`beginTransaction` and one burst transfer instead of one `transfer()` call per byte, a higher SPI clock, and a user-bank select only when the bank changes).
3. Make the rates consistent. Choose one IMU rate that the loop always meets and design the filters for it (keep 1 kHz if passes stay below about 0.8 ms, otherwise set `kSampleRateHz` to 500), run the control tick on each accepted IMU sample with a fallback tick for the stale-IMU checks, and design the battery filter for the real tick rate. Add a host test that keeps the IMU ODR code consistent with `kSampleRateHz`.

Step 3 changes the filter cutoffs and the control rate, so the balance gains may need retuning: do it with the robot supported and follow the [balance controller](../software/balance-controller.md) commissioning steps. See also the control-timing row in the [robust control plan](robust-control-system.md).

## Driving stability at high speed

At high speed the robot still tips over. Record the failure pattern in a safe, reproducible driving test, covering roll angle, target and actual wheel speed, effective speed setpoint, and the balance controller output. Check whether the existing tilt reduction takes effect in the observed case and engages early enough. Change thresholds and controller tuning only after this evaluation.

## Clean up the RC value conversion chain

A remote value passes through several hidden conversions before a controller uses it: transmitter µs to raw SBUS, a per-channel `mapf()` range and sign in `applyRcChannels()`, a negation for turn, and for some channels a biquad filter. The resulting units (for example ±30 "speed" compared with wheel rad/s) are not physical, and the signs are spread across several functions. The [RC channel map](../software/rc-channel-map.md#from-transmitter-to-firmware-value) documents the current chain.

Replace it with one conversion per channel from raw SBUS to a physical unit (m/s, rad/s, degrees, or a switch state), with all scaling and signs in one place and covered by host tests with the measured values 192, 992, and 1792. Reject or clip raw values outside the valid range. The [robust control plan](robust-control-system.md) already defines the new mapping (CH3 speed in m/s, CH4 yaw rate in rad/s, CH9 trim in degrees) and removes the BLE path; do this cleanup as part of that implementation.

## Fix the sensor diagnostic mode

The diagnostic build (`SENSOR_DIAGNOSTIC_MODE 1`) no longer streams a CSV. It is a link build: its live signals are listed on the [sensor diagnostic mode](../software/sensor-diagnostic-mode.md#which-signals-are-live) page, and its robot check is in the [integration roadmap](../../integration/plans/roadmap.md#link-logs-signals-and-parameters-on-the-robot). The receiver's ten raw SBUS channel values are not in the signal table, so the mode cannot report them yet.

- If the raw SBUS channels are needed for diagnosis, add them as `SIGNAL` entries and capture them with navlink.

## Switch the LiDAR off after a fall

Route the LiDAR's `M−` pin (LiDAR enable) to the robot PCB, most likely, and drive it so that the LiDAR switches off when the robot falls.
