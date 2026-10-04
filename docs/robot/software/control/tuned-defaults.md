# Tuned control defaults, 2026-10-01

The user reported hard vibration and stopped motors while the electronics remained on. A serial session found automatic tuning mode `U0`. Restoring the earlier live gain set with `U1` stopped the vibration according to the user. This comparison changed several gains and does not isolate one coefficient or prove a motor-protection event.

The selected defaults are angle `PP5 PI200 PD0.11 PL0.1`, speed `SP0.045 SI0.005 SD0 SL50`, yaw `YP4 YI0 YD0 YL0`, and direct wheel-speed feedback `V0`. Startup keeps live tuning enabled with `U1`. Later serial adjustments remain volatile and reboot reloads the compiled defaults.

The startup-default build completed successfully with `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`. Upload through the reidentified CH340 at `/dev/ttyUSB1` identified ESP32-S3 revision v0.2 and passed flash verification. A post-upload serial startup check queried all 14 values without writing gains and matched every value. This established startup persistence; it did not include a new post-flash balancing trial.

At the user's subsequent request, both CH5 balance/speed presets, the global controller initializers, and the common yaw preset also use this gain set. Roll and touch-controller gains retain their existing mode-specific values. The motor target limit and control timing retain their existing implementations.

The retained local startup captures are [control-startup.log](../../../../agent_notes/robot/evidence/2026-10-01/control-startup.log) and [control-defaults-flash-test.log](../../../../agent_notes/robot/evidence/2026-10-01/control-defaults-flash-test.log). Other historical control-recovery traces are not present in this workspace.

## Flash test of the aligned presets

The subsequent build of commit `cf81c50` was uploaded on 2026-10-01 through CH340 `1a86:7523` at `/dev/ttyUSB0`. The sketch binary SHA-256 was `a3e9aa43e600caee298694ad1267aa5033daf90f4177e295c6cacadbee6c78d7`. The build's generated source contained the aligned presets and initializers, and its recorded board configuration used USB CDC disabled.

The uploader identified ESP32-S3 revision v0.2 and completed flash hash verification with exit status zero. A serial check used 115200 baud, DTR low, and RTS low. All 14 startup settings matched without gain writes. After an explicit RTS reset with DTR kept low, a second startup check matched all 14 again. Both checks observed `Motor ready.`. The retained local raw log is [control-defaults-flash-test.log](../../../../agent_notes/robot/evidence/2026-10-01/control-defaults-flash-test.log). This test establishes successful flashing and restart persistence, not active balancing or driving performance.
