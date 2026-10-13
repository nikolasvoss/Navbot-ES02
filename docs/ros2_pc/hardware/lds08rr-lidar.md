# LDS08RR LiDAR

## Scope and status

The LDS08RR is the module named for the planned ros2_pc LiDAR integration. User-supplied photographs received on 13 October 2026 confirm the model marking and the PCB markings below. They do not confirm an installed connection, electrical measurements, or a working ROS 2 driver. Integration remains planned in the [ros2_pc roadmap](../ROS_ROADMAP.md#2-roborock-lidar-in-ros-2-integrieren).

## Module label

The [label photograph](images/lds08rr-label.jpg) shows these markings on this module:

| Item | Printed marking |
| --- | --- |
| Model | `LDS08RR` |
| Rated voltage and power | `5VDC/1.5W` |
| Laser classification | `Class 1 Laser Product` |
| Compliance statement | `IEC 60825-1:2014` and `EN 60825-1:2014+A11:2021` |
| Separate identification label | `HHCD2SAA1204104` |

The rated power divided by the rated voltage gives 0.30 A. This is a calculation from the label, not a measurement or a startup, stall, or peak-current limit. The laser classification and compliance statement are label claims, not independent certification checks.

![LDS08RR model, voltage, power, and laser label](images/lds08rr-label.jpg)

## Connector and PCB markings

The [PCB photograph](images/lds08rr-pcb.jpg) shows a six-contact connector at the top of the image. In that exact view, with the silkscreen text upright and the contacts below the connector body, the printed labels run left to right as follows. The numbers below describe photo positions only; they are not manufacturer pin numbers.

| Photo position | Silkscreen | Verification status |
| ---: | --- | --- |
| 1, leftmost | `NC` | Marking only; internal connection unverified |
| 2 | `M−` | Function and electrical requirements unverified |
| 3 | `RX` | Likely module receive signal by naming; direction and voltage unmeasured |
| 4 | `TX` | Likely module transmit signal by naming; direction and voltage unmeasured |
| 5 | `GND` | Ground marking; continuity unmeasured |
| 6, rightmost | `VIN` | Supply-input marking; connection to the labelled 5 V supply unverified |

A mating plug viewed from its cable side reverses the apparent order. Connector series, pitch, and pin-1 convention are not established by the photograph. `NC` is not an available GPIO assignment. `M−` does not by itself establish a PWM input or a motor-power connection.

The board also has `PWM`, `M−`, `M−CT`, `VCC`, `VIN`, `PGND`, and `TX` markings. A separate two-contact connector carries red and black wires near the motor. Neither wire colour nor these markings establish a complete circuit or wiring diagram. The photo shows four outer mounting ears and a circular PCB. Hole spacing and side-view dimensions come from a user sketch, recorded in the [mechanical dimensions section](#mechanical-dimensions), and are not independently measured. Screw sizes and scan-plane height remain unknown.

![LDS08RR PCB and six-contact connector markings](images/lds08rr-pcb.jpg)

## Mechanical dimensions

The [dimension record](mechanics/lds08rr-dimensions.json) and [mounting-hole drawing](mechanics/lds08rr-lochbild.svg) transcribe user-supplied dimensioned sketches received on 13 October 2026: the [mounting sketch](images/lds08rr-mounting-sketch.jpg), the [side-view sketch](images/lds08rr-side-sketch.jpg), and a later [combined side and top-view sketch](images/lds08rr-dimension-sketch.jpg) with follow-up answers from the user. The values are not independently measured.

- Four Ø 3.1 mm mounting holes form a rectangle with 54.0 mm horizontal and 44.0 mm vertical centre spacing, as read from the mounting sketch. The sketch gives a 69.4 mm diagonal; a 54 × 44 mm rectangle implies 69.66 mm. Rectangularity is not measured.
- The rotor centre coincides with the centre of the hole pattern, as confirmed by the user.
- Heights from the bottom of the lower round housing: 17.8 mm to the underside of the plastic plate and 28.0 mm to its top, which carries the PCB. The plate is therefore 10.2 mm thick. The overall height to the top of the motor is 38.0 mm.
- The mounting ears are 4.3 mm thick, with their top flush with the plate top.
- Widths: lower housing 59.0 mm, plate Ø 59.6 mm (top view), and motor 24.5 mm. The outer edge of the motor is 50.0 mm from the rotor centre, and the overall length is 80.9 mm.

Coordinates use the hole-pattern centre as origin, viewed from the PCB side with the motor on the left. Tolerances, hole depth, fastener type, optical-axis offset, and scan-plane height remain unspecified.

### Simplified 3D model

[lds08rr-simple.step](mechanics/lds08rr-simple.step) is a simplified envelope of the module for measurements, such as bracket design and clearance checks. The build123d source is [lds08rr_simple.py](mechanics/lds08rr_simple.py). To regenerate the STEP file, run the script from the `mechanics/` folder.

The model origin is the rotor centre. z = 0 is the bottom of the lower housing, and +z points toward the PCB and motor. It models the housing (Ø 59.0 mm, z 0 to 17.8 mm), the plate (Ø 59.6 mm plus the area under the motor, z 17.8 to 28.0 mm), the ears (z 23.7 to 28.0 mm), and the motor (Ø 24.5 mm, axis at x = −37.75 mm, z 28.0 to 38.0 mm). The connector, PCB components, and true ear outline are not modelled; the 7 mm ear width is assumed. The housing is modelled concentric with the rotor. As a result, the model is 80.5 mm long instead of the sketched 80.9 mm. The sketched 59.0 mm housing width, combined with the 80.9 mm overall length, would put the housing centre about 1.4 mm off the rotor axis.

## Upstream protocol variants

The [kaiaai LDS library](https://github.com/kaiaai/LDS) identifies LDS08RR as Revo LDS used in Roborock and the 3irobotix CRL-200S platform. Its documentation distinguishes Delta framing from a Camsense-family revision. That identification does not prove which revision is shown in this project's photographs.

The [lds2d model table](https://github.com/kaiaai/lds2d#supported-models) gives the following software-reference configurations. Both LDS08RR entries are marked as ports not yet confirmed on physical hardware by that project.

| Upstream variant | Arduino LDS identifier | lds2d identifier | UART baud rate | Framing | Upstream motor classification |
| --- | --- | --- | ---: | --- | --- |
| Delta | `LDS_LDS08RR` | `3IROBOTIX-LDS08RR` | 115200 | `0xAA` | Host PWM |
| Camsense family | `LDS_LDS08RR_CAMSENSE` | `LDS08RR-CAMSENSE` | 115200 | `0x55 0xAA` | Onboard |

Sources for the identifiers and framing are the [LDS README](https://github.com/kaiaai/LDS#experimental-needs-testing). Baud rates and motor classifications come from the [lds2d README](https://github.com/kaiaai/lds2d#supported-models). These are driver assumptions, not verified electrical specifications for this module.

The Camsense support derives from a user's capture in [LDS issue 17](https://github.com/kaiaai/LDS/issues/17) and remains experimental in the LDS library. That capture belongs to another device. A capture from this module is still needed to establish its protocol. LDS01RR, LDS02RR, and LDROBOT LD08 specifications are not interchangeable with LDS08RR specifications.

## Pending electrical and mechanical verification

The ros2_pc connection is not assigned. A UART or USB-to-UART adapter choice depends on confirmed connector mapping and signal voltages. The labelled 5 V supply rating does not establish 5 V UART logic. A connection requires verified ground continuity and a supply arrangement that avoids backfeeding another source.

If this module requires external motor control, it needs a suitable motor power stage; a GPIO cannot supply motor current. Motor voltage, current, control polarity, PWM frequency, and the role of `M−` remain unknown. The upstream onboard-motor classification for one variant does not resolve those questions from a photo.

The remaining hardware record needs:

- Connector identification, continuity checks, UART logic levels, and any required input signals.
- Operating, startup, and stall current, plus the actual motor-control circuit.
- A raw serial capture identifying framing, scan rate, distance units, and invalid measurements.
- Range, accuracy, optical clearance, independently measured dimensions and mounting-hole spacing, and scan-plane alignment for this module.

The future ROS 2 integration can publish `sensor_msgs/msg/LaserScan` as described in the roadmap. No driver, GPIO allocation, motor-control implementation, or sensor transform has been selected by this hardware reference.

## Evidence

The two original user photographs above are the local evidence for the label, connector markings, and visible layout. Upstream driver documentation was checked on 13 October 2026. No LDS08RR manufacturer pinout or mechanical drawing was verified during this documentation task.
