
# Navbot-ES02 - Open Source Desktop Dual-Wheel Legged Robot

![robot](docs/image/photo1.JPG)

**Navbot-ES02** is a compact, open-source desktop robot that combines self-balancing wheels with articulated legs. It’s an experimental platform for exploring locomotion, balance, and user interaction — ideal for makers, educators, and robotics enthusiasts.

Full tutorial: [Frank Fu’s Build Guide](https://frankfu.blog/embodied-ai-robot/diy-desktop-dual-wheel-legged-roboy/)

---

##  Key Features

-  **Self-balancing** using MPU6050 and PID control
-  **Legged motion** using 270° digital servos
-  **Touchscreen interface** for motion control to achieve  ball-balancing
-  **Radiolink T12D RC System** for real-time operation switching via PWM/SBUS.
-  **Modular Arduino-based firmware** for easy customization

---

## Hardware Overview

| Component      | Description                                   |
|----------------|-----------------------------------------------|
| **MCU**        | ESP32-S3 DevKit                               |
| **Motors**     |  2208 90KV 3mm Shaft Gimbal Brushless Motors            |
| **IMU**        |  ICM42688 a 3-axis gyroscope and a 3-axis accelerometer            |
| **Servos**     | Pro-Tronik PTK 7452 MG-D 2.8kg/0.11s 12.3g Micro Digital Servo     |
| **HMI**        | 2.8" resistive/capacitive touchscreen         |
| **Battery**    | 2Pcs 18650 Battery Charger Pack              |
| **Controller**    | Radiolink T12D 12CH 2.4G RC Transmitter R12F Receiver                     |

---



## 📦 Software Structure

```
Navbot-ES02/
├── OllieFOCdrive/     # Motor & balance control
├── TouchUI/           # Touchscreen-based user interface
├── WiFiControl/       # Optional WebSocket/HTML control panel
├── partitions.csv     # Flash partition layout
└── lib/               # Required Arduino libraries
```

---

## Getting Started

### 1. Install Requirements

- Arduino IDE 2.x
- ESP32 board package (`esp32@3.0.7`)
- Libraries (via Library Manager or GitHub):
  - [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)
  - [SimpleFOcDrivers](https://github.com/simplefoc/Arduino-FOC-drivers)
  - [Preferences](https://github.com/vshymanskyy/Preferences)
    
### 2. Flash the Firmware

```bash
git clone https://github.com/fuwei007/Navbot-ES02.git
```

- Open `OllieFOCdrive.ino` with Arduino IDE
- Select board: `ESP32S3 Dev Module`
- Upload code and monitor via Serial for debugging

For an already compiled Arduino build, use [`scripts/upload_firmware.py`](scripts/upload_firmware.py). The exact compile and upload commands are in [`agent_notes/usb-serial.md`](agent_notes/usb-serial.md).

### Safe sensor diagnostics

Set `SENSOR_DIAGNOSTIC_MODE` to `1` and rebuild to use the sensor-only diagnostic firmware; the current source has it set to `0` for normal operation. In diagnostic mode the wheel driver enable pins are held low, motor and servo initialization is skipped, and the robot reports sensor data over the CH340 USB serial connection. Support the robot mechanically because the legs are not driven. BLE control is also inactive in this mode.

Use **ESP32S3 Dev Module** with **USB CDC On Boot: Disabled** when uploading. The diagnostic firmware sends 20 numeric CSV samples per second on the CH340 USB port at **115200 baud**. In SerialPlot, select the current `/dev/ttyUSB*` port, set **DTR off** and **RTS off** in the Port tab, and choose **ASCII**, **comma delimiter**, **14 channels** in Data Format. Opening the port may cause a brief reset; keep it open while watching for spontaneous reboots. Startup text can cause a few harmless parsing warnings before the numeric samples begin.

If SerialPlot shows `boot:...DOWNLOAD(USB/UART0)` and `waiting for download`, the ESP32 entered its flash loader instead of starting the firmware. With SerialPlot connected, leave DTR off and pulse RTS on then off, or press the board's RESET/EN button. The next boot line should say `SPI_FAST_FLASH_BOOT`; numeric CSV samples then follow. If the board says nothing at all, also check that RTS is off, because asserted RTS can hold it in reset. Do not run the Python monitor simultaneously with SerialPlot: both need exclusive access to the port.

Channel order (also use these as names in SerialPlot's Plot tab):

| # | Name | Unit / meaning |
| --- | --- | --- |
| 1 | `uptime_s` | seconds since boot; drops on reset |
| 2–4 | `gyro_x`, `gyro_y`, `gyro_z` | rad/s |
| 5–7 | `acc_x`, `acc_y`, `acc_z` | g |
| 8–10 | `roll`, `pitch`, `yaw` | degrees; uncalibrated values may drift |
| 11 | `battery_raw_v` | V, direct ADC conversion without filter delay |
| 12 | `rc_age_ms` | age of last SBUS frame; `-1` before any frame |
| 13 | `rc_failsafe` | `0` valid, `1` lost, `3` failsafe, `-1` before any frame |
| 14 | `imu_ok` | `1` initialized, `0` failed |

For plain text inspection, close SerialPlot first (the serial port can be opened by only one program) and use the included monitor. It requires `pyserial` and also keeps RTS deasserted:

```bash
python3 scripts/sensor_monitor.py
```

The `DIAG,boot` line includes `reset_reason` (`1` means power-on or external reset, `9` means brownout). The CSV voltage channel uses the direct ADC value so it does not have the filtered voltage's startup delay.

The diagnostic mode is selected at compile time. Leave it enabled while checking sensors, receiver and supply voltage. Changing it to `0` restores the original motor-control startup path and requires a rebuild and upload.

### Capture a balance shutdown

With `SENSOR_DIAGNOSTIC_MODE` set to `0`, upload the rebuilt sketch and support the robot so it cannot fall. Close every other serial monitor, then run:

```bash
python3 scripts/capture_balance_trace.py --seconds 20 --output balance-trace.log
```

The script waits for startup, sends `K55`, and records a 50 Hz `TRACE` stream. Switch RC channel 5 from off to on **once** during the capture. The columns are `time_ms,ch5_mode,ch6_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,pitch_deg,servo1_cmd_deg,servo2_cmd_deg,servo3_cmd_deg,servo4_cmd_deg,servo1_range_deg,servo2_range_deg,servo3_range_deg,servo4_range_deg,motor1_target,motor2_target`. Each `range` is the highest minus lowest integer servo command in the preceding 20 ms. The minimum voltage uses all battery ADC samples since the previous row; it is still a board ADC estimate, not a measurement of the ESP32's 3.3 V rail. The servo values are commanded angles, not measured shaft positions. The saved log also includes any reboot or brownout text that reaches the serial port. The script sends `K0` at the end to stop tracing. If the robot loses power, retain the log even if the script reports a port error.

After uploading firmware built from the current source, add `--trace-mode 56` and choose a new output file to capture the compact `CTRL` stream instead. It records `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,gyro_z_rad_s,body_turn_command,angle_error_deg,angle_output,yaw_error,yaw_output,motor1_target,motor2_target,max_servo_range_deg`. The angle and yaw outputs are the two terms combined into the wheel targets; an inactive balance mode reports zero for their error/output columns. `K56` suppresses the repeated low-voltage warning text in its own stream to reduce serial interleaving. `body_turn_command` and `yaw_error` use the firmware's numerical scale; do not assume they are physical rad/s without checking the RC mapping. Existing firmware images that only implement `K55` must be reflashed for this mode.

For controller decomposition, `--trace-mode 57` records `BAL` rows: `time_ms,ch5_mode,voltage_min_raw_v,roll_deg,angle_error_deg,angle_out_p,angle_out_i,angle_out_d,body_x,motor1_target,motor2_target,max_servo_range_deg`. Its shorter lines reduce, but do not eliminate, 2 Mbaud serial corruption. Give each run a unique output filename.

For forward/reverse motion and stopping behavior, `--trace-mode 58` records `DRIVE` rows: `time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,ch3_target,motor1_velocity_f,motor2_velocity_f,tick_dt_s,speed_error,speed_out_p,speed_out_i,speed_out_d,speed_output,body_x,body_pitching_f,roll_ok,angle_output,angle_out_p,angle_out_i,angle_out_d,wheel_speed_feedback,motor1_target,motor2_target,top_ball_x,touch_x,body_pitching,max_servo_range_deg`. Use this only after confirming the trace output with CH5 off; each run should have a new filename. The logger samples at 20 Hz and does not change controller gains. The live-tuning image uses 115200 baud, so capture it with `--baud-rate 115200`. Logs from before the wheel-speed-feedback build have the older header without that column.

The current `DIAGNOSTIC_LIVE_TUNING_DEFAULTS=1` build preloads the user's selected angle P/I/D=5/200/0.12 with integral limit 0.1 (±20 motor-target units), speed P/I/D=0.045/0/0, and yaw P/I/D=4/0/0 **at startup**. The experimental wheel-speed-to-wheel gain `V` starts at 0.2 and contributes at most ±8 motor-target units; `V0` disables this new term while retaining all other tuned values. It does not add drive torque when a reverse command is still below its target speed, because reverse acceleration is already strong; it does act on overspeed and on braking after CH3 returns to neutral. `V` reads back the gain, and `V0.1` selects a smaller effect. Live tuning remains enabled, but no USB connection is needed to run the robot after flashing. The wheel speed estimate uses the measured control tick. The SBUS CH3 command is multiplied by 2.0; the `ch3_target` DRIVE column shows the scaled target. These gains also apply when the physical CH5 switch passes briefly through mode 1. `SP0.04` changes speed P live; `SP` reads it back. `PL0.05` changes the angle integral limit live; `PL` reads it back. `PP`, `PI`, `PD`, and `YP` read the other relevant gains. `U0` restores this build's tuned startup defaults at the next active control step; `U1` keeps live tuning. Live changes are not stored in flash and reset to these startup values after reboot. The CH3 scaling is fixed in this build and cannot be tuned through serial commands.

To log commands and telemetry through one serial connection, run `python3 scripts/capture_balance_trace.py --baud-rate 115200 --trace-mode 56 --interactive --seconds 40 --output control-live-yaw10.log`. The script logs every trace row and each sent command (`# CMD ...`), while showing only every tenth trace row in the terminal. With CH5=0, type `PP` and `YP` to verify the startup gains before entering CH5=2. The interactive input accepts only gain, `U0`/`U1`, and trace-selection commands; it cannot send calibration or motor-drive commands.

For a repeatable agent-facing run summary, pass one or more saved logs to `scripts/summarize_balance_trace.py`. It reports valid and malformed rows, capture span, CH5 mode counts/transitions, and a small set of min/max/mean values without making causal claims:

```bash
python3 scripts/summarize_balance_trace.py balance-trace.log
python3 scripts/summarize_balance_trace.py run-1.log run-2.log --output balance-summary.md
```

For repeated K58 drive/stop trials, one command captures a new log at 115200 baud and writes a short `*.md` report after capture. It refuses to overwrite an existing log or report:

```bash
python3 scripts/run_drive_trial.py --interactive --output drive-test-01.log --seconds 40
```

The report checks trace quality, CH5 activation, CH3 movement, start tilt, and the largest opposite wheel-speed reading in the first second after a CH3 release. If there is no usable stop event, the command returns status 2 and keeps both files. It does not switch the robot. With `--interactive`, type a gain command such as `SP0.04`, then `SP` to read it back, before enabling CH5. The capture logs accepted commands as `# CMD ...` and includes firmware replies. Use a new filename for each gain setting and state the observed movement with the report path. Opening a new serial session may restart the board, so set and read back live gains again in each run.

Compare saved trials without opening Serial or recompiling firmware:

```bash
python3 scripts/analyze_drive_trace.py drive-test-01.log drive-test-02.log --require-stop --output drive-comparison.md
```

`--format json` provides the same measurements for other scripts. A trial with neutral CH3 or a strongly tilted start is flagged as unsuitable for a stop comparison; it is not interpreted as a controller improvement or regression.

### 3. Connect the model remote control
ES02 has 3 lines, namely GND, 5V, and sbus.

<img src="docs/image/ES02 sbus wire.png" height="350"/>

The remote control receiver may have multiple interfaces. It is necessary to confirm the sbus
output port by yourself. The following picture shows the wiring ports of the RadioLink receiver.

<img src="docs/image/RadioLink connector.png" height="350"/>

The channels of the joystick are generally defaulted to ch1-4. Besides, six auxiliary channels
are needed to switch on some functions. Each remote control is different, and they should be set
according to personal habits. The corresponding functions of ch6-10 can be referred to as follows:

- CH5 : 3-section switch, 0:stop, 1:start, 2:start and touch tablet enable. 
- CH6 : 2-section switch, 0:posture mode, 1:mark mode. 
- CH7 : 2-section switch, manual or auto roll, 0:manual , 1:auto . 
- CH8 : 3-section switch, 0:default, 1:pitching adjust, 2:ball poise. 
- CH9 : The x-coordinate of the small ball . 
- CH10:The y-coordinate of the small ball.


---

## Tourial Videos

- DIY ES02 Desktop dual-wheel legged bot Preview:
  
  [![Video 2](https://img.youtube.com/vi/u7Jmyq_AXwc/0.jpg)](https://www.youtube.com/watch?v=u7Jmyq_AXwc)


- Open Source ESP32 DIY Robot ES02: Learn Coding & Robotics | Educational Dual-Wheel Legged Bot:
  [![Video 1](https://img.youtube.com/vi/hujr_VRSyrw/0.jpg)](https://www.youtube.com/watch?v=hujr_VRSyrw)

---

## Discord link
| Link: [https://discord.gg/syywQ2CKN3](https://discord.gg/syywQ2CKN3)        |
| :------------: |
| <img src="docs/image/discord link.png" height="200"/> |

---

## Acknowledgments

- [SimpleFOC Community](https://simplefoc.com/)
- [Frank Fu’s Robotics Blog](https://frankfu.blog)
- [LVGL GUI Library](https://lvgl.io/)
- Arduino & ESP32 Open Source Communities

---

> Feel free to fork, contribute, or build your own version!  
> PRs and feedback are always welcome.

## WLAN-Parameterterminal (Phase 1)

Der optionale WLAN-Tuning-Build und der Python-Client sind in [dev_reference/wifi-parameterterminal.md](dev_reference/wifi-parameterterminal.md) beschrieben. WLAN ist standardmäßig ausgeschaltet; das Terminal kann nur die freigegebenen Reglerparameter lesen und bei frischem SBUS sowie CH5 OFF schreiben. Es gibt keinen Upload- oder Flash-Schritt in diesem Ablauf.
