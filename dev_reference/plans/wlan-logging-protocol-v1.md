# WLAN recording protocol v1

The recorder stores a byte-for-byte copy of frames after the stream-ticket handshake. TCP is only a byte stream. A receiver reads the fixed 32-byte header first, validates every field and size, then reads exactly the advertised payload. It never searches for `NBL1` after corruption.

## Record

Every DATA payload consists of 1 to 32 consecutive 128-byte records. Each record is encoded explicitly in little-endian order. Its 24-byte prefix is `timestamp_us:u64`, `sequence:u32`, `flags:u32`, `imu_age_us:u32`, and `control_dt_us:u32`. It is followed by 26 IEEE-754 binary32 values in this order: `roll_ok`, `BodyPitching_f`, gyro x/y/z, movement speed, body turn, motor velocity 1/2, Speed PID error/P/I/D/output, Angle PID error/P/I/D/output, Yaw PID output, BodyX, motor target 1/2, wheel speed feedback output, voltage, and wheel speed feedback gain.

The flag assignments are bits 0-1 gain mode, bit 2 tumble, bit 3 regulator computed this tick, bit 4 valid/fresh RC, bit 5 IMU read timestamp available, bit 6 numeric fault, bits 7-8 roll mode, bits 9-10 attitude mode, and bits 11-12 posture/mark mode. Other bits are zero. Inactive PID values use the canonical quiet NaN `0x7fc00000`.

## Frame

The 32-byte header is `NBL1`, version u16=1, type u16 (1 META, 2 DATA, 3 END), recording ID u64, frame sequence u32, payload length u32, record count u32, and CRC32 u32. All integers are little-endian. CRC is CRC-32/ISO-HDLC over header bytes 0-27 followed by payload; the standard vector is `123456789` → `0xcbf43926`.

META is UTF-8 JSON, at most 8192 bytes, record count zero. DATA is 1-32 records with exact `count*128` length. END is UTF-8 JSON at most 4096 bytes, record count zero. META, DATA and END must arrive in that order, with consecutive frame sequence numbers and one recording ID. The CRC reported by END covers raw DATA payload bytes in order.

META identifies schema, boot and recording IDs, device start time, requested duration, build ID, enabled feature flags, supported/control modes, configured gains and effective-gain notes, IMU ODR and filter cutoff, calibration state and roll/pitch/gyro biases, record field order and units, buffer capacity, and mode-bit meanings. The current build ID is `wifi-recording-v1`; `source_hash_available` is `false`, so the source revision is not represented. Credentials and tickets are never included.

## Ownership and lifecycle

The control loop owns sample snapshots and HTTP session transitions. It copies one complete encoded record into a fixed 512-slot SPSC ring. The consumer owns a 4096-byte staging area and the TCP data socket. It releases slots only after copying their bytes to staging. No network calls or JSON serialization run in the balance loop. The unauthenticated HTTP service owns `/api/v1/recording/*`; its queue routes bounded requests to the loop. A separate data listener owns port 8766, checks one one-use ticket to associate the stream with the active recording, and publishes readiness through the command/event queue. A sender failure sets an atomic terminal state that stops production at the next capture check.

The on-device state order is IDLE → PREPARED → RECORDING → DRAINING → AWAIT_ACK → COMPLETE, with FAILED/UNCONFIRMED terminals. Duration is measured against the device monotonic clock. HTTP transitions run in the loop, and sender failures stop acquisition at the next capture check; neither path alters motor commands. PC output starts at META in `OUTPUT.nblog.partial`; it is flushed and fsynced before publication and ACK. Any incomplete or invalid transfer remains a `.partial` file.
