# CM5 and LiDAR current measurement, 2026-10-15

Status: concluded

Observation: The user measured approximately 800 mA average current for CM5 together with LiDAR, with observed peaks up to 1.2 A. The LiDAR ran continuously during the measurement.

Evidence: User report in chat on 2026-10-15:

> Dokumentiere. Ich habe die Stromaufnahme des CM5 mit LIDAR gemessen. Im Schnitt braucht er ungefähr 800 Milliampere, mit Peaks bis 1,2 Ampere. Das LIDAR läuft dabei die ganze Zeit.

No independent measurement was performed by the agent. Supply voltage, measurement point, instrument, measurement duration, and CM5 workload were not reported. Startup current was not specified.

Practical consequence: Use these values as an observed combined operating load when assessing the power supply. They do not establish the worst-case current or confirm sufficient reserve on the Navbot 5 V rail. Canonical record: [ros2_pc hardware](../../docs/ros2_pc/hardware/system-overview.md#measured-current-with-lidar).
