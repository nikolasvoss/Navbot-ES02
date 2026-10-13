"""Rectangular frame stand for the LiDAR / CM5 mount.

Hole pattern and hole diameter come from docs/robot/hardware/mechanics/touchscreenstand.md.
Countersink: 6.1 mm at the top face, 90 degree included angle (typical), opening upward (+Z).
Run: python3 lidar_cm5_stand.py  -> writes lidar_cm5_stand.stl and .step next to this file.
"""
from math import radians, tan
from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    Locations,
    Mode,
    Cone,
    Cylinder,
    Pos,
    export_step,
    export_stl,
)

HOLE_SPACING_X = 96.4  # mm, centre to centre
HOLE_SPACING_Y = 72.9  # mm, centre to centre
HOLE_DIAMETER = 2.2  # mm
COUNTERSINK_DIAMETER = 6.1  # mm, at the top face
COUNTERSINK_ANGLE = 90.0  # degrees, included angle
ROD_WIDTH = 8.0  # mm
ROD_HEIGHT = 5.0  # mm

COUNTERSINK_DEPTH = (COUNTERSINK_DIAMETER - HOLE_DIAMETER) / 2 / tan(radians(COUNTERSINK_ANGLE / 2))

outer_x = HOLE_SPACING_X + ROD_WIDTH
outer_y = HOLE_SPACING_Y + ROD_WIDTH

with BuildPart() as stand:
    Box(outer_x, outer_y, ROD_HEIGHT)
    Box(
        outer_x - 2 * ROD_WIDTH,
        outer_y - 2 * ROD_WIDTH,
        ROD_HEIGHT,
        mode=Mode.SUBTRACT,
    )
    with Locations(
        (HOLE_SPACING_X / 2, HOLE_SPACING_Y / 2),
        (-HOLE_SPACING_X / 2, HOLE_SPACING_Y / 2),
        (HOLE_SPACING_X / 2, -HOLE_SPACING_Y / 2),
        (-HOLE_SPACING_X / 2, -HOLE_SPACING_Y / 2),
    ):
        Cylinder(HOLE_DIAMETER / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
        with Locations((0, 0, (ROD_HEIGHT - COUNTERSINK_DEPTH) / 2)):
            Cone(
                HOLE_DIAMETER / 2,
                COUNTERSINK_DIAMETER / 2,
                COUNTERSINK_DEPTH,
                mode=Mode.SUBTRACT,
            )

if __name__ == "__main__":
    here = Path(__file__).parent
    export_stl(stand.part, str(here / "lidar_cm5_stand.stl"))
    export_step(stand.part, str(here / "lidar_cm5_stand.step"))
    bb = stand.part.bounding_box()
    print(f"size {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, volume {stand.part.volume:.1f} mm3, valid={stand.part.is_valid}")
