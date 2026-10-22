# Robot documentation

The robot is the ESP32-S3 controller board and everything attached to it: wheel motors, servos, IMU, battery, and RC receiver. Placement rules for new pages are in [AGENTS.md](../../AGENTS.md#where-files-go).

## Hardware

- [System overview](hardware/system-overview.md): start here for any hardware question.
- [Electronics reference](hardware/electronics-reference.md): power paths, GPIOs, connectors, and firmware interfaces.
- [Netlist](hardware/netlist.md): every component pad and net, for targeted search.
- [Mounting holes](hardware/touchscreenstand.md): robot mounting-hole pattern, also used by the lidar stand.

## Software

- [Control flow](software/control-flow.md): how the balance and drive control works.
- [RC channel map](software/rc-channel-map.md): SBUS channel assignment in the firmware.
- [Tuned control defaults](software/tuned-defaults.md): startup gains and how they were chosen.
- [Trace capture and live tuning](software/trace-capture.md): serial trace modes, capture scripts, and live gain commands.
- [Motor commissioning](software/motor-commissioning.md): bounded per-wheel current and voltage identification, manual resistance method, KV fit, and the pending commissioning record.
- [Sensor diagnostic mode](software/sensor-diagnostic-mode.md): motor-free sensor streaming.
- [USB, serial, and flashing](software/usb-serial-flashing.md): verified connection facts and flashing.
- [Firmware build design](software/firmware-build-design.md): the build command and its options.

## Plans

- [Roadmap](plans/roadmap.md)
- [Robust standing and driving control](plans/robust-control-system.md): proposed control chain, payload trim, vibration checks, and staged tuning.
