# Touchscreen stand: hole pattern from user report, 2026-10-13

Status: concluded

## Observation

The user gives a rectangular hole pattern of 96.4 mm × 72.9 mm for the mounting holes of `touchscreenstand.stl`, a hole diameter of 2.2 mm, and a countersink.

## Evidence

- The user message of 2026-10-13 is the only source of the dimensions. Wording (translated from German): "The dimensions of the Mounting Holes of TouchscreenStand.stl are the following: rectangular: 96.4mm x 72.9mm, hole diameter 2.2mm with countersink. document this at a suitable place"
- The file [`cad/robot/touchscreenstand.stl`](../../cad/robot/touchscreenstand.stl) is in the repository. It was not measured for this note.
- The figure "96.4 mm × 72.9 mm" is read as center-to-center spacing.

## Practical consequence

The dimensions are a user statement, not verified STL geometry. Countersink geometry, tolerances, axis direction, and screw size remain open. Canonical reference: [Touchscreen stand](../../docs/robot/hardware/touchscreenstand.md).
