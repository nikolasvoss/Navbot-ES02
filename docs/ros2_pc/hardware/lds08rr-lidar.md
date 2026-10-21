# LDS08RR LiDAR

The LDS08RR is the ros2_pc's 2D LiDAR. This page covers its wiring, electrical interface, data format and mechanics. Software setup is in [ros2_pc software](../software/README.md#lds08rr-lidar).

## Status

| Topic | State | Date |
| --- | --- | --- |
| Model and connector markings | Confirmed by user photographs | 2026-10-13 |
| Protocol | Confirmed: Camsense variant, 115200 baud, 8N1, non-inverted | 2026-10-15 |
| Live reception | Verified: browser scans and ROS `/scan` at about 5 Hz on `/dev/ttyAMA0` | 2026-10-15 |
| Supply wiring, voltage margins | Not verified | — |
| Motor control (`M−`) | Not connected, no circuit chosen | — |
| Angle zero, direction, distance accuracy | Not calibrated | — |

See also the [roadmap item](../ROS_ROADMAP.md#2-roborock-lidar-in-ros-2-integrieren).

## Wiring to the ros2_pc

The LiDAR is on UART0 of the carrier's J8 header. The user confirmed the connection on 2026-10-15. Software verified reception on `/dev/ttyAMA0`, not the supply wiring or voltage margins. Complete the [open electrical checks](#open-checks) before changing wiring.

| LiDAR signal | J8 pin | ros2_pc signal | Condition |
| --- | ---: | --- | --- |
| `VIN` | 4 | 5 V | Verify carrier supply path and current budget |
| `GND` | 6 | Ground | Verify ground continuity |
| `TX` | 10 | GPIO15 / UART0 RX | Verify signal voltage before connecting |
| `RX` | 8, reserved | GPIO14 / UART0 TX | Leave disconnected unless a capture proves commands are required |
| `M−` | 11, reserved | GPIO17 | Only through a verified external sink; never connect directly to the 5 V idle signal |
| `NC` | — | — | Leave unconnected |

GPIO17 is only reserved. No overlay, GPIO output or motor-control code activates it. Select and verify the sink circuit and drive requirements first. The manufacturer's [GPIO guidance](https://pip-assets.raspberrypi.com/categories/685-whitepapers-app-notes/documents/RP-006553-WP/A-history-of-GPIO-usage-on-Raspberry-Pi-devices-and-current-best-practices) specifies a maximum GPIO input of 3.3 V.

Host state on 2026-10-15: the ros2_pc reported module revision 1.0 and Ubuntu 24.04.5. GPIO14/15 were `TXD0`/`RXD0`, and `/dev/ttyAMA0` mapped to RP1 `serial@30000`. No serial console or other reader owned the port. No boot change or reboot was needed. UART2 on GPIO4/5 stays assigned to the HMMD radar; this setup did not enable or change it.

## Module label

The [label photograph](images/lds08rr-label.jpg) shows:

| Item | Printed marking |
| --- | --- |
| Model | `LDS08RR` |
| Rated voltage and power | `5VDC/1.5W` |
| Laser classification | `Class 1 Laser Product` |
| Compliance statement | `IEC 60825-1:2014` and `EN 60825-1:2014+A11:2021` |
| Separate identification label | `HHCD2SAA1204104` |

1.5 W at 5 V gives 0.30 A. This is a label rating, not a measured startup, stall or peak current. Laser classification and compliance are unverified label claims.

![LDS08RR model, voltage, power, and laser label](images/lds08rr-label.jpg)

## Connector

In the [PCB photograph](images/lds08rr-pcb.jpg), the six-contact connector is at the top, with upright silkscreen and the contacts below its body. Positions run left to right in that view. They are not manufacturer pin numbers.

| Photo position | Silkscreen | What is known |
| ---: | --- | --- |
| 1, leftmost | `NC` | Internal connection unverified; not an available GPIO |
| 2 | `M−` | Active low; draws 1 mA when pulled to GND; 5 V when idle (user, 2026-10-14) |
| 3 | `RX` | Probably the module's receive input; direction and voltage not measured |
| 4 | `TX` | 3 V peak (user, 2026-10-14); an earlier DMM reading gave 1.8 V. Low level not measured |
| 5 | `GND` | LiDAR ground |
| 6, rightmost | `VIN` | Supply input; recorded maximum 500 mA, measured 230 mA with motor on |

A mating plug viewed from the cable side reverses this order. Connector series, pitch and pin-1 convention are unknown. `M−` alone does not identify a PWM input or a motor-power connection.

Other PCB markings are `PWM`, `M−`, `M−CT`, `VCC`, `VIN`, `PGND` and `TX`. A separate two-contact connector with red and black wires sits near the motor. These markings do not establish a circuit diagram. The photo also shows four mounting ears and a circular PCB.

![LDS08RR PCB and six-contact connector markings](images/lds08rr-pcb.jpg)

### Electrical notes

- The 5 V supply rating does not establish the UART logic levels.
- The 3 V TX peak and the 1.8 V DMM reading are separate measurements. The instrument and conditions of the peak reading were not given, and nobody measured independently. The capture confirms timing and protocol, not voltage levels.
- The Camsense driver has no host motor output, so reception says nothing about the motor circuit. `M−` needs an external transistor sink (open collector or open drain) that keeps its 5 V idle level away from the GPIO.
- The 1 mA is the `M−` pin current, not the module or motor current.

## Data format

The module uses the LDS08RR **Camsense** variant at **115200 baud, 8N1, non-inverted**. The user's `acq0003.csv` logic capture contains three complete checksum-valid packets. A direct UART0 capture on 2026-10-15 contains 450 valid packets in 3.01 seconds and no failed checksum candidates.

| Field | Encoding |
| --- | --- |
| Packet | 60 bytes, starts with `55 AA 07 0C` |
| Speed | Little-endian uint16, divided by 3840 for Hz |
| Start and end angles | Little-endian uint16, `(raw − 0xA000) / 64` degrees |
| Samples | 12 × 4 bytes: signed int16 distance in mm, uint8 quality, reserved byte |
| Missing return | `0x8000` distance, observed in the capture |
| Checksum | Trailing little-endian 15-bit checksum, validated by the driver |

The first logic-capture packet covers 175.33°–186.25° with 193–288 mm returns. The driver reports about 5 Hz (300 RPM). These units follow the confirmed decoder; distance accuracy and physical angles are not calibrated.

The integration uses `3IROBOTIX-LDS08RR-CAMSENSE` from `lds2d[viz]==0.7.0`, matching [the upstream decoder](https://github.com/kaiaai/LDS/blob/main/src/LDS_LDS08RR_CAMSENSE.h). The [LDS library](https://github.com/kaiaai/LDS) identifies this module family as the Revo LDS used in Roborock and the 3irobotix CRL-200S platform. Its Camsense support started in [LDS issue 17](https://github.com/kaiaai/LDS/issues/17); capture and live decoding have since confirmed it for this module. No other protocol is selected or auto-detected. The earlier second protocol variant and its selection flag were removed after confirmation.

### ROS output

The software publishes `sensor_msgs/msg/LaserScan` on `/scan` and diagnostics on `/lidar/status`. Each of 360 one-degree bins holds the nearest valid return in metres, or positive infinity when missing. The 0.01–30 m limits are provisional software bounds. Angle zero and rotation direction follow the driver and are not calibrated. Scan start is estimated as host receive completion minus one reported rotation period.

In the 2026-10-15 check, one launcher fed both the browser and ROS. Over five seconds, 26 `/scan` messages with 360 bins arrived, `/lidar/status` stayed fresh, and there were no receive or publish errors. The browser rendered the live polar plot.

## Mechanical dimensions

The [dimension record](mechanics/lds08rr-dimensions.json) and [mounting-hole drawing](mechanics/lds08rr-lochbild.svg) transcribe user sketches of 2026-10-13 and later answers: [mounting](images/lds08rr-mounting-sketch.jpg), [side view](images/lds08rr-side-sketch.jpg) and [combined views](images/lds08rr-dimension-sketch.jpg). **Nothing has been measured independently.**

- Four Ø 3.1 mm holes at 54.0 mm horizontal and 44.0 mm vertical centre spacing. The sketch gives a 69.4 mm diagonal; a rectangle implies 69.66 mm. Rectangularity is not measured.
- The rotor is centred on the hole pattern (user confirmation).
- Measured from the bottom of the lower housing, the plastic plate spans 17.8–28.0 mm, so it is 10.2 mm thick. Overall height to the motor top is 38.0 mm.
- The mounting ears are 4.3 mm thick and flush with the plate top.
- Widths: lower housing 59.0 mm, plate Ø 59.6 mm, motor 24.5 mm. The motor's outer edge is 50.0 mm from the rotor centre. Overall length is 80.9 mm.

Coordinates originate at the hole-pattern centre, viewed from the PCB side with the motor on the left. Tolerances, hole depth, fasteners, optical-axis offset and scan-plane height are not specified.

### Simplified 3D model

[lds08rr-simple.step](mechanics/lds08rr-simple.step) is an envelope for bracket and clearance checks. Regenerate it by running [lds08rr_simple.py](mechanics/lds08rr_simple.py) from `mechanics/`. The origin is the rotor centre; z = 0 is the housing bottom, with +z towards the PCB and motor.

| Part | Model geometry |
| --- | --- |
| Housing | Ø 59.0 mm, z 0–17.8 mm |
| Plate | Ø 59.6 mm plus the area under the motor, z 17.8–28.0 mm |
| Ears | z 23.7–28.0 mm; width assumed 7 mm |
| Motor | Ø 24.5 mm, axis at x = −37.75 mm, z 28.0–38.0 mm |

The model omits the connector, PCB components and the true ear outline. Its concentric housing gives 80.5 mm overall length against the sketched 80.9 mm. Matching both the sketched length and the 59.0 mm housing width would shift the housing centre about 1.4 mm from the rotor. Measure this before relying on the clearance on the side opposite the motor.

## Open checks

- Connector identification, continuity, UART levels and required input signals.
- Operating, startup and stall current, and the motor-control circuit.
- Range, accuracy, optical clearance, measured dimensions and hole spacing, scan-plane alignment.
- Angle zero, rotation direction and distance calibration.

## Evidence

The photographs document the label, connector markings and visible layout. The user's logic capture and the direct UART capture establish the protocol. The [agent observations](../../../agent_notes/ros2_pc/sensors/lds08rr/observations.md) link the raw UART record. Upstream decoder definitions were checked on 2026-10-15. No manufacturer pinout or mechanical drawing has been verified.
