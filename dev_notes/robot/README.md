# Robot work notes

- Confirm the sender's actual SBUS channel assignments in its monitor. The [RC channel map](../../docs/robot/software/control/rc-channel-map.md) documents the firmware mapping, not the HOTRC switch setup.
- Build with `python3 scripts/build_firmware.py`. For uploads, follow `AGENTS.md` and use the configured `navbot_flash` tool with the active build directory.
- Check the [hardware overview](../../docs/robot/hardware/system-overview.md) before changing wiring. It distinguishes the PCB design from observations on the assembled robot.
- Read the [sensor diagnostic guide](../../docs/robot/software/diagnostics/sensor-diagnostic-mode.md) before collecting sensor-only serial data.
