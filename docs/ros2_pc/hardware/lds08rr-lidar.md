# LDS08RR LiDAR

## Scope and status

LDS08RR is the selected ros2_pc LiDAR. User photographs from 13 October 2026 confirm its label and PCB markings. Live Camsense reception, browser scans, and ROS `/scan` were verified on 15 October 2026. Electrical acceptance and orientation calibration remain open. See the [software steps](../software/lds08rr-lidar.md) and [roadmap](../plans/roadmap.md#2-integrate-roborock-lidar-in-ros-2).

## Module label

The [label photograph](../assets/lds08rr-label.jpg) shows:

| Item | Printed marking |
| --- | --- |
| Model | `LDS08RR` |
| Rated voltage and power | `5VDC/1.5W` |
| Laser classification | `Class 1 Laser Product` |
| Compliance statement | `IEC 60825-1:2014` and `EN 60825-1:2014+A11:2021` |
| Separate identification label | `HHCD2SAA1204104` |

Dividing the labelled power by voltage gives 0.30 A, not a measured startup, stall, or peak limit. Laser classification and compliance are unverified label claims.

![LDS08RR model, voltage, power, and laser label](../assets/lds08rr-label.jpg)

## Connector and PCB markings

In the [PCB photograph](../assets/lds08rr-pcb.jpg), the six-contact connector is at the top, with upright silkscreen and contacts below its body. Positions below run left to right in that view; they are not manufacturer pin numbers.

| Photo position | Silkscreen | Status |
| ---: | --- | --- |
| 1, leftmost | `NC` | Internal connection unverified; not an available GPIO |
| 2 | `M−` | User reports active low, 1 mA when grounded, and 5 V idle on 14 October 2026 |
| 3 | `RX` | Likely module receive signal; direction and voltage unmeasured |
| 4 | `TX` | User reports 3 V peak on 14 October 2026; earlier DMM reading 1.8 V. 115200 baud, 8N1 confirmed; low voltage level unmeasured |
| 5 | `GND` | LiDAR ground |
| 6, rightmost | `VIN` | Supply input; recorded maximum 500 mA, measured 230 mA with motor on |

A mating plug viewed from the cable side reverses this order. Connector series, pitch, and pin-1 convention remain unknown. `M−` alone does not identify a PWM input or motor-power connection.

Other PCB markings are `PWM`, `M−`, `M−CT`, `VCC`, `VIN`, `PGND`, and `TX`. A separate two-contact connector has red and black wires near the motor; these do not establish a circuit diagram. The photo shows four mounting ears and a circular PCB. [Mechanical dimensions](#mechanical-dimensions) come from user sketches; screw sizes and scan-plane height remain unknown.

![LDS08RR PCB and six-contact connector markings](../assets/lds08rr-pcb.jpg)

## Mechanical dimensions

The [dimension record](../../../cad/ros2_pc/lds08rr-dimensions.json) and [mounting-hole drawing](../assets/lds08rr-hole-pattern.svg) transcribe user sketches received on 13 October 2026 and later follow-up answers: [mounting](../assets/lds08rr-mounting-sketch.jpg), [side view](../assets/lds08rr-side-sketch.jpg), and [combined views](../assets/lds08rr-dimension-sketch.jpg). Dimensions are not independently measured.

- Four Ø 3.1 mm holes have 54.0 mm horizontal and 44.0 mm vertical centre spacing. The sketch gives a 69.4 mm diagonal; a rectangle implies 69.66 mm. Rectangularity is unmeasured.
- The rotor is centred on the hole pattern, per user confirmation.
- From the lower housing bottom, the plastic plate spans 17.8–28.0 mm, giving 10.2 mm thickness. Overall height to the motor top is 38.0 mm.
- Mounting ears are 4.3 mm thick and flush with the plate top.
- Widths are 59.0 mm for the lower housing, Ø 59.6 mm for the plate, and 24.5 mm for the motor. The motor's outer edge is 50.0 mm from the rotor centre; overall length is 80.9 mm.

Coordinates originate at the hole-pattern centre, viewed from the PCB side with the motor left. Tolerances, hole depth, fasteners, optical-axis offset, and scan-plane height are unspecified.

### Simplified 3D model

[lds08rr-simple.step](../../../cad/ros2_pc/lds08rr-simple.step) provides an envelope for bracket and clearance checks. Regenerate it by running [lds08rr_simple.py](../../../cad/ros2_pc/lds08rr_simple.py) from `mechanics/`.

The origin is the rotor centre; z = 0 is the housing bottom, with +z toward the PCB and motor.

| Part | Model geometry |
| --- | --- |
| Housing | Ø 59.0 mm, z 0–17.8 mm |
| Plate | Ø 59.6 mm plus area under motor, z 17.8–28.0 mm |
| Ears | z 23.7–28.0 mm; assumed width 7 mm |
| Motor | Ø 24.5 mm, axis x = −37.75 mm, z 28.0–38.0 mm |

The model omits the connector, PCB components, and true ear outline. Its concentric housing gives 80.5 mm overall length versus the sketched 80.9 mm. Matching the sketched length and 59.0 mm housing width would offset the housing centre about 1.4 mm from the rotor.

## Confirmed data format

The module uses the LDS08RR Camsense variant at **115200 baud, 8N1, non-inverted**. The user's `acq0003.csv` logic capture contains three complete checksum-valid packets. A direct UART0 capture on 15 October 2026 contains 450 valid packets in 3.01 seconds, with no failed checksum candidates.

| Field | Encoding |
| --- | --- |
| Packet | 60 bytes, starts with `55 AA 07 0C` |
| Speed | Little-endian uint16, divided by 3840 for Hz |
| Start and end angles | Little-endian uint16, `(raw − 0xA000) / 64` degrees |
| Samples | 12 × 4 bytes: signed int16 distance in mm, uint8 quality, reserved byte |
| Missing return | `0x8000` distance observed in the capture |
| Checksum | Trailing little-endian 15-bit checksum, validated by the driver |

The first logic-capture packet covers 175.33°–186.25°, with 193–288 mm returns. The driver reports about 5 Hz / 300 RPM. These units follow the confirmed decoder; distance accuracy and physical angles remain uncalibrated.

The integration uses `3IROBOTIX-LDS08RR-CAMSENSE` from `lds2d[viz]==0.7.0`, matching [the upstream decoder](https://github.com/kaiaai/LDS/blob/main/src/LDS_LDS08RR_CAMSENSE.h). The [LDS library](https://github.com/kaiaai/LDS) identifies this module family as Revo LDS used in Roborock and the 3irobotix CRL-200S platform. Its experimental Camsense support originated in [LDS issue 17](https://github.com/kaiaai/LDS/issues/17); this module is now independently confirmed by capture and live decoding. No other protocol is selected or auto-detected.

## Pending electrical and mechanical verification

Before wiring, verify connector mapping, ground continuity, UART voltages, and a supply path that prevents backfeeding. The 5 V supply rating does not establish UART logic levels.

The Camsense driver has no host motor output; reception does not establish the module's motor circuit. `M−` needs an external transistor sink, such as open collector or open drain, to isolate its reported 5 V idle level from GPIO. Logic thresholds, waveform, motor voltage, and total motor current remain unverified.

The reported 3 V TX peak and earlier 1.8 V DMM reading are separate measurements. Peak-measurement equipment and conditions were unspecified; no independent measurement was made. TX low voltage level remains unmeasured; digital timing and protocol are confirmed by the capture. Voltage measurement establishes the electrical interface; a UART capture establishes framing.

Remaining checks:

- Connector identification, continuity, UART levels, and required input signals.
- Operating, startup, and stall current, and the motor-control circuit.
- Range, accuracy, optical clearance, measured dimensions and hole spacing, and scan-plane alignment.

The software publishes `sensor_msgs/msg/LaserScan` on `/scan` and diagnostics on `/lidar/status`. Each of 360 one-degree bins holds the nearest valid return in metres, or positive infinity when missing. The 0.01–30 m limits are provisional software bounds. Angle zero and rotation direction follow the driver and remain uncalibrated. Scan start is estimated as host receive completion minus one reported rotation period.

## ros2_pc connection

LiDAR data is received through UART0 on the Compute Module 5 J8 header. The user confirmed connection on 15 October 2026; software verified reception on `/dev/ttyAMA0`, not the physical supply wiring or voltage margins. Confirm the electrical checks above before changing wiring.

| LiDAR signal | J8 physical pin | CM5 signal | Connection condition |
| --- | ---: | --- | --- |
| `VIN` | 4 | 5 V | Verify carrier supply path and current budget |
| `GND` | 6 | Ground | Verify ground continuity |
| `TX` | 10 | GPIO15 / UART0 RX | Verify signal voltage before connecting |
| `RX` | 8, reserved | GPIO14 / UART0 TX | Leave disconnected unless a capture proves commands are required |
| `M−` | 11, reserved | GPIO17 | Through a verified external sink only; never connect directly to the 5 V idle signal |
| `NC` | None | None | Leave unconnected |

GPIO17 is reserved only; no overlay, GPIO output, or motor-control code activates it. Select and verify the sink circuit and drive requirements first. Raspberry Pi's [GPIO guidance](https://pip-assets.raspberrypi.com/categories/685-whitepapers-app-notes/documents/RP-006553-WP/A-history-of-GPIO-usage-on-Raspberry-Pi-devices-and-current-best-practices) specifies a maximum 3.3 V GPIO input.

On 15 October 2026, the host identified as `Raspberry Pi Compute Module 5 Rev 1.0` on Ubuntu 24.04.5. GPIO14/15 were `TXD0`/`RXD0`; `/dev/ttyAMA0` mapped to RP1 `serial@30000`. No serial console or competing reader owned that port. No boot change or reboot was needed.

The live launcher uses one reader for the browser and ROS. Five seconds of observation received 26 `/scan` messages with 360 bins and fresh `/lidar/status`, without receive or ROS publish errors. The browser rendered the live polar plot. Angle zero and direction remain native and uncalibrated.

UART2 on GPIO4/5 and `dtoverlay=uart2-pi5` remain assigned to the HMMD radar; this LiDAR setup did not enable or change them. GPIO17 remains inactive.

The unused protocol integration and selection flag were removed after capture and live-scan confirmation. See the completed [roadmap item](../plans/roadmap.md#basis-and-open-questions).

## Evidence

The original photographs document the label, connector markings, and visible layout. The user TX capture and direct UART capture establish the protocol; [agent observations](../../../lab/ros2_pc/2026-10-13-lds08rr-observations.md) link the raw UART record. Upstream decoder definitions were checked on 15 October 2026. No manufacturer pinout or mechanical drawing was verified.
