# ros2_pc-Hardware

This directory documents the ros2_pc, its carrier board, power supply, connectors, and attached hardware.

The measured current of CM5 together with a running LiDAR is recorded below. The LDS08RR reference below includes module label and PCB photo evidence; its electrical behavior and connection remain unverified. Do not infer pin assignments, power limits, or peripheral connections from the roadmap. Add each fact with its board revision and evidence when the hardware is selected or inspected.

## Measured current with LiDAR

User-reported measurement, 2026-10-15:

| Configuration | Average current | Observed peaks |
| --- | --- | --- |
| CM5 together with LiDAR, with LiDAR running continuously | Approximately 800 mA | Up to 1.2 A |

These values describe the combined load in the measured operating condition. Supply voltage, measurement point, instrument, measurement duration, and CM5 workload were not reported. Startup current was not specified. The observed peaks do not establish a maximum current requirement or verify the Navbot supply's available capacity.

Source: [user measurement record](../../../lab/ros2_pc/2026-10-15-power-observations.md).

The [ros2_pc carrier-board pinout](ros2-pc-carrier-board.md) is a reference. It does not confirm that the project has selected or verified that carrier board.

The [LDS08RR LiDAR hardware reference](lds08rr-lidar.md) records the photographed module rating, connector markings, upstream protocol variants, and pending electrical and mechanical checks.

The planned HMMD radar and Roborock LiDAR work is listed in the [ros2_pc software roadmap](../plans/roadmap.md). Record their electrical and mechanical details here only after the exact modules are identified.
