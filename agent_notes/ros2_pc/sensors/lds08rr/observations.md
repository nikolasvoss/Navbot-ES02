# LDS08RR photo evidence, 2026-10-13

## TX peak clarification, 2026-10-14

Observation: The user reports LiDAR TX at 3 V peak. The earlier 1.8 V DMM reading is retained as a separate measurement.

Evidence: User statement: "lidar tx is 3V peak. document." Instrument and measurement conditions were not specified. No independent agent measurement was performed.

Practical consequence: Record 3 V as the reported TX peak in the canonical hardware reference and browser notes. This supersedes the earlier absence of a reported peak voltage. Low level, timing, protocol, and installed UART operation remain unverified; this documentation update does not establish live hardware acceptance.

## User voltage and installation clarification, 2026-10-14

Observation: The user reports `M−` at 5 V idle and a 1.8 V DMM reading on LiDAR TX. The LiDAR is not connected to the ros2_pc. At the time of this earlier report, TX's peak voltage and protocol were unconfirmed; the later 3 V peak report is recorded above.

Evidence: User answers: "m- is 5V idle and lidar tx is 1.8V (measured with dmm, not sure if it averages from 3v3 or so)" and "not connected atm. implement both, but add todo to remove later again". These are user measurements and installation statements, not independent agent measurements.

Practical consequence: Use an external transistor sink for `M−`; its 5 V idle signal must not reach a GPIO directly. That DMM reading alone did not establish a 1.8 V logic interface. Keep both implemented LDS08RR decoders temporarily. Once this module's capture identifies the protocol and valid scans confirm the selected driver, remove the unused project variant, its selector, tests, and documentation. The roadmap records this removal TODO. Motor output remains inert.

## ros2_pc UART2 inspection, 2026-10-14

Observation: The software host identifies as `Raspberry Pi Compute Module 5 Rev 1.0` running Ubuntu 24.04.5. `/dev/ttyAMA0` and `/dev/ttyAMA10` exist; `/dev/ttyAMA2` does not. `/boot/firmware/config.txt` has no UART2 overlay. `pinctrl get 4-5` reports GPIO4 and GPIO5 with no alternate function, while GPIO14 and GPIO15 carry UART0. The installed overlay README documents `dtoverlay=uart2-pi5` for UART2 on GPIO4 and GPIO5.

Evidence: Direct inspection of `/proc/device-tree/model`, `/etc/os-release`, `/dev`, `/boot/firmware/config.txt`, `pinctrl get`, and `/boot/firmware/overlays/README` on the ros2_pc on 2026-10-14. No boot configuration was changed and no sensor wiring was performed.

Practical consequence: Reserve J8 pin 7 / GPIO4 for UART2 TX and pin 29 / GPIO5 for UART2 RX. After electrical checks, add the documented overlay and reboot, then inspect the resulting device mapping. Do not assume `/dev/ttyAMA2` until the active image confirms it. Reserve GPIO17 on J8 pin 11 for a future external `M−` sink only; do not wire it directly to the module.

## LiDAR software status, 2026-10-14

Observation: The `lds08rr_lidar` package uses the released `lds2d[viz]==0.7.0` drivers with explicit Delta or Camsense selection. One reader feeds the upstream plot and ROS 2 output. The web status page starts without a UART or the HMMD radar. Delta opens with an inert motor-output adapter.

Evidence: The implementation and hardware-free protocol fixtures in the `feat/lds08rr-lidar` worktree. Upstream driver fixtures exercise both protocol parsers; they do not identify the installed module's protocol.

Practical consequence: Use demo mode for browser checks. Keep protocol selection provisional until a raw capture from this module identifies it. The ROS angle zero, direction, range limits, and scan-start timestamp remain software assumptions or estimates until hardware measurements establish them.

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
