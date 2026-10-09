# IMU accepted-read timing in diagnostic mode

Date: 2026-10-09

## Observation

With the robot secured, the diagnostic-only firmware ran at 115200 baud with both motor enables low and servo initialization skipped. The user captured 12,940 bytes over ten seconds. Nine `IMU_TIMING` records report 9,066 accepted reads over 9,000,175 µs, an aggregate host-accepted rate of 1,007.3 Hz. Individual one-second windows report 1,002 to 1,008 Hz.

The cumulative long-gap counter rises once. The reported maximum accepted-read interval is 1,149 µs before that event and 7,802 µs afterward. The module counts intervals above 1,500 µs as long gaps.

The driver maps the 1,000 Hz setting to ICM-42688-P ODR code 6 for accelerometer and gyroscope. TDK documents 1 kHz for code `0110` and notes internal-clock ODR uncertainty can reach ±1% in PLL mode or ±8% in RCOSC mode. See the [ICM-42688-P datasheet](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000347-icm-42688-p-v1.6.pdf), clocking section 4.10 and ODR register descriptions.

## Evidence

The user-provided serial output was captured from `/dev/ttyUSB0` at 115200 baud with PySerial, with RTS and DTR low before opening the port. The nine timing lines were pasted into the task chat. The user reported the binary capture size as 12,940 bytes at `/tmp/imu-timing-capture.bin`.

## Practical consequence and limits

The observed diagnostic accepted-read rate is close to nominal 1 kHz, though 0.7% above it on average. One 7.802 ms interval shows that accepted reads were not uniformly spaced; the cumulative gap counter confirms one interval exceeded the 1.5 ms threshold. The status register is a read-clear data-ready flag, not a conversion counter, so this capture cannot prove that every sensor conversion was observed. Normal motor-control-loop timing remains unmeasured.
