# LDS08RR photo evidence, 2026-10-13

## Connector M− measurement, 2026-10-14

Observation: The LiDAR connector's `M−` pin is active low and draws 1 mA when pulled to GND.

Evidence: User report on 2026-10-14: "the lidar m- pin on the connector is low active and draws 1mA when pulled to GND". No independent measurement was performed by the agent.

Practical consequence: Activate by pulling the connector's `M−` pin low; the controlling sink must handle the reported 1 mA. This is connector-pin current, not total module or motor current. Open-circuit voltage, logic thresholds, and PWM behaviour are still unverified. This updates the electrical evidence beyond the photographs described below.

## Photo evidence

Observation: The user-provided module label reads LDS08RR, 5VDC/1.5W, Class 1 Laser Product, IEC 60825-1:2014, and EN 60825-1:2014+A11:2021. A separate label reads HHCD2SAA1204104. The PCB has a six-contact connector whose upright photographed left-to-right labels are NC, M−, RX, TX, GND, VIN. A separate two-contact connector has red and black wires. Four outer mounting ears are visible.

Evidence: Original user photographs 59725.jpg and 59723.jpg supplied in this chat on 2026-10-13, preserved as [label](../../../../docs/ros2_pc/hardware/images/lds08rr-label.jpg) and [PCB](../../../../docs/ros2_pc/hardware/images/lds08rr-pcb.jpg). Canonical reference: [LDS08RR hardware](../../../../docs/ros2_pc/hardware/lds08rr-lidar.md).

Practical consequence: The model and nominal supply rating can be documented as label evidence. The photo position order is not manufacturer pin numbering. No continuity, UART voltage, current, motor-control function, protocol revision, or installation has been verified. Do not derive wiring from M− or wire colours. The two protocol variants documented upstream cannot be distinguished from these photographs alone.

# LDS08RR mounting sketch, 2026-10-13

Observation: A user-supplied mounting sketch gives four Ø 3.1 mm holes at 54.0 mm horizontal and 44.0 mm vertical centre spacing. A side-view sketch gives 38.0 mm overall height, 59.0 mm lower-housing width, and 24.5 mm motor width. Its 17.6 mm and 28.0 mm height references have unconfirmed upper reference surfaces.

Evidence: Transcribed in [dimension record](../../../../docs/ros2_pc/hardware/mechanics/lds08rr-dimensions.json), with [drawing](../../../../docs/ros2_pc/hardware/mechanics/lds08rr-lochbild.svg), [mounting sketch](../../../../docs/ros2_pc/hardware/images/lds08rr-mounting-sketch.jpg), and [side sketch](../../../../docs/ros2_pc/hardware/images/lds08rr-side-sketch.jpg). The record marks every value as transcribed from a sketch, not measured.

Practical consequence: Use these values only as unverified planning inputs. Measure the module before fixing a bracket. Do not infer plate thickness from the 17.6 mm or 28.0 mm references. The optical-axis and motor-centre offsets from the hole pattern are not established.

# LDS08RR heights and simplified model, 2026-10-13

Observation: A second user sketch (side and top view) and the user's chat answers establish the following. The left height reads 17.8 mm, not the 17.6 mm transcribed earlier, and runs from the housing bottom to the underside of the plastic plate. The 28.0 mm runs to the plate top, which carries the PCB, so the plate is 10.2 mm thick. The mounting ears are 4.3 mm thick and flush with the plate top. The rotor centre coincides with the hole-pattern centre. The sketch also gives a Ø 59.6 mm plate, a 50.0 mm distance from the motor's outer edge to the rotor centre, an 80.9 mm overall length, and a 69.4 mm hole diagonal.

Evidence: [combined sketch](../../../../docs/ros2_pc/hardware/images/lds08rr-dimension-sketch.jpg), the user's answers in chat on 2026-10-13, the updated [dimension record](../../../../docs/ros2_pc/hardware/mechanics/lds08rr-dimensions.json), and the [simplified STEP model](../../../../docs/ros2_pc/hardware/mechanics/lds08rr-simple.step) built from [lds08rr_simple.py](../../../../docs/ros2_pc/hardware/mechanics/lds08rr_simple.py).

Practical consequence: The earlier warning against subtracting 17.6 from 28.0 no longer applies, because the reference surfaces are now confirmed. The values are still sketch values, not measurements. The 59.0 mm housing width and the 80.9 mm overall length disagree by about 1.4 mm with a housing concentric to the rotor. Measure this before relying on the clearance on the housing side opposite the motor.
