# Touchscreen stand: hole pattern from user statement, 2026-10-13

## Observation

The user gives the mounting holes of `touchscreenstand.stl` as a rectangular pattern of 96.4 mm × 72.9 mm, a hole diameter of 2.2 mm, and a countersink.

## Evidence

- The user's message of 2026-10-13 is the only source of the dimensions. Original wording (German): „Die Maße der Mounting Holes von TouchscreenStand.stl sind die folgenden: rechteckig: 96,4mm x 72,9mm, lochdurchmesser 2,2mm mit senkung. dokumentiere das an passender stelle“
- [`hardware/3d print/touchscreenstand.stl`](../../../hardware/3d%20print/touchscreenstand.stl) is in the repository. It was not measured for this note.
- "96.4 mm × 72.9 mm" is read as centre-to-centre spacing.

## Practical consequence

The dimensions are a user statement, not checked STL geometry. Countersink geometry, tolerances, axis direction and screw size remain open. Canonical reference: [touchscreen stand](../../../docs/robot/hardware/mechanics/touchscreenstand.md).
