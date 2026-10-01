# Tuned control defaults, 2026-10-01

The user reported hard vibration and stopped motors while the electronics remained on. A serial session found automatic tuning mode `U0`. Restoring the earlier live gain set with `U1` stopped the vibration according to the user. This comparison changed several gains and does not isolate one coefficient or prove a motor-protection event.

The selected defaults are angle `PP5 PI200 PD0.11 PL0.1`, speed `SP0.045 SI0.005 SD0 SL50`, yaw `YP4 YI0 YD0 YL0`, and direct wheel-speed feedback `V0`. Startup keeps live tuning enabled with `U1`. Later serial adjustments remain volatile and reboot reloads the compiled defaults.

The startup-default build completed successfully with `esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=default`. Upload through the reidentified CH340 at `/dev/ttyUSB1` identified ESP32-S3 revision v0.2 and passed flash verification. A post-upload serial startup check queried all 14 values without writing gains and matched every value. This established startup persistence; it did not include a new post-flash balancing trial.

At the user's subsequent request, both CH5 balance/speed presets, the global controller initializers, and the common yaw preset also use this gain set. Roll and touch-controller gains retain their existing mode-specific values. The motor target limit and control timing retain their existing implementations.

The detailed local captures are retained under `agent_notes/control-recovery-2026-10-01*` and `agent_notes/control-startup-2026-10-01.log`. These capture artifacts are not part of this tuning commit.
