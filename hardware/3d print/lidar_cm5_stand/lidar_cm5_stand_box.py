"""Stand for the robot (bottom frame), a CM5 IO board (board frame) and the LDS08RR lidar (top).

Derived from lidar_cm5_stand.py. Total height is 26 mm outer to outer. The stand prints as
three parts without support material: one base and two identical top pieces.

Base, printed bottom-down:
- Bottom frame: robot mounting holes from docs/robot/hardware/mechanics/touchscreenstand.md
  (96.4 x 72.9 mm), each with a 90 degree countersink, 6.1 mm wide, on its upper face, in a
  round boss that thickens the wall around the countersink.
- Board frame: an 8 x 5 mm frame on the print bed that runs around the 90 x 160 mm CM5 IO board.
  Its top is flush with the bottom frame, so the robot screw countersinks stay open.
  The board lies across the stand: its 160 mm side is parallel to the short (Y) side.
- Board mounts: the four corner holes of the Waveshare CM5 IO board drawing (121.5 x 69 mm),
  each on a pad under the board with an M2.5 heat-set insert, joined to the board frame by an
  8 x 5 mm rod. The pads stand BOARD_PAD_LIFT above the frames, so the board rests only on them. Board coordinates (u along 160 mm, v along 90 mm, origin at the board corner)
  map to stand x = v - 45, y = 80 - u.
- Four pillars in the X sides of the board frame, ending at the split plane with an M3
  heat-set insert each.

Top piece, printed flat, print 2x: one short bar screwed onto two pillars, carrying two of the
LDS08RR holes from docs/ros2_pc/hardware/mechanics/lds08rr-dimensions.json (54 x 44 mm) on
round pads joined by 8 x 5 mm rods. The left and right pieces are the same part rotated 180 deg.

Print details: 0.5 mm chamfer on all bed edges against elephant's foot, 1.5 mm fillets on the
inner corners, mouse ears at the base corners against warping (cut off after printing), and
HOLE_COMPENSATION added to the screw holes because small vertical holes print undersize.

Run: python3 lidar_cm5_stand_box.py  -> writes lidar_cm5_stand_base.* and lidar_cm5_stand_top.*
(STL and STEP, in print orientation) and lidar_cm5_stand_assembly.step.
"""
from math import radians, tan
from pathlib import Path

from build123d import (
    Box,
    BuildPart,
    Compound,
    Cone,
    Cylinder,
    GeomType,
    Locations,
    Mode,
    Part,
    Pos,
    Rot,
    add,
    export_step,
    export_stl,
)

HOLE_SPACING_X = 96.4  # mm, robot holes, centre to centre
HOLE_SPACING_Y = 72.9  # mm
HOLE_DIAMETER = 2.2  # mm
COUNTERSINK_DIAMETER = 6.1  # mm, at the upper face of the bottom frame
COUNTERSINK_ANGLE = 90.0  # degrees, included angle
BOARD_X = 90.0  # mm, CM5 IO board, across the stand
BOARD_Y = 160.0  # mm, parallel to the short side of the stand
BOARD_CLEARANCE = 0.5  # mm per side between board and board frame
BOARD_HOLES_UV = [(11.0, 8.0), (11.0, 77.0), (132.5, 8.0), (132.5, 77.0)]  # mm, from board corner
BOARD_INSERT_DIAMETER = 3.6  # mm, hole for an M2.5 heat-set insert
BOARD_PAD_DIAMETER = 8.0  # mm
BOARD_PAD_LIFT = 1.0  # mm, pad top above the bottom frame top
ROBOT_BOSS_DIAMETER = 12.0  # mm, round boss around each robot hole
LIDAR_SPACING_X = 54.0  # mm, LDS08RR holes, centre to centre
LIDAR_SPACING_Y = 44.0  # mm
LIDAR_HOLE_DIAMETER = 3.1  # mm
SPLIT_INSERT_DIAMETER = 4.0  # mm, hole for an M3 heat-set insert in each pillar top
SPLIT_INSERT_DEPTH = 6.0  # mm
SPLIT_SCREW_DIAMETER = 3.4  # mm, M3 clearance hole in the top piece
HOLE_COMPENSATION = 0.2  # mm, added to screw holes; insert holes use the maker's printed size
ROD_WIDTH = 8.0  # mm
ROD_HEIGHT = 5.0  # mm
TOTAL_HEIGHT = 26.0  # mm, outer to outer
PILLAR_Y = 22.0  # mm, pillar centre line, in line with the lidar rods
BED_CHAMFER = 0.5  # mm
INNER_FILLET = 1.5  # mm
MOUSE_EAR_DIAMETER = 15.0  # mm
MOUSE_EAR_HEIGHT = 0.4  # mm, about two layers

robot_hole = HOLE_DIAMETER + HOLE_COMPENSATION
lidar_hole = LIDAR_HOLE_DIAMETER + HOLE_COMPENSATION
split_screw = SPLIT_SCREW_DIAMETER + HOLE_COMPENSATION
COUNTERSINK_DEPTH = (COUNTERSINK_DIAMETER - robot_hole) / 2 / tan(radians(COUNTERSINK_ANGLE / 2))
outer_x = HOLE_SPACING_X + ROD_WIDTH
outer_y = HOLE_SPACING_Y + ROD_WIDTH
bottom_z = -TOTAL_HEIGHT / 2
bottom_mid_z = bottom_z + ROD_HEIGHT / 2
bottom_top_z = bottom_z + ROD_HEIGHT
pad_height = ROD_HEIGHT + BOARD_PAD_LIFT
board_plane_z = bottom_z + pad_height  # underside of the IO board
split_z = TOTAL_HEIGHT / 2 - ROD_HEIGHT
top_mid_z = split_z + ROD_HEIGHT / 2
board_inner_x = BOARD_X + 2 * BOARD_CLEARANCE
board_inner_y = BOARD_Y + 2 * BOARD_CLEARANCE
frame_outer_x = board_inner_x / 2 + ROD_WIDTH
frame_outer_y = board_inner_y / 2 + ROD_WIDTH
pillar_x = board_inner_x / 2 + ROD_WIDTH / 2  # centre line of the board frame X sides
ROBOT_HOLES = [(sx * HOLE_SPACING_X / 2, sy * HOLE_SPACING_Y / 2) for sx in (1, -1) for sy in (1, -1)]
BOARD_HOLES = [(v - BOARD_X / 2, BOARD_Y / 2 - u) for u, v in BOARD_HOLES_UV]
PILLARS = [(sx * pillar_x, sy * PILLAR_Y) for sx in (1, -1) for sy in (1, -1)]
TOP_LIDAR_HOLES = [(LIDAR_SPACING_X / 2, sy * LIDAR_SPACING_Y / 2) for sy in (1, -1)]
TOP_SCREWS = [(pillar_x, sy * PILLAR_Y) for sy in (1, -1)]

# rod from each lidar hole outward to the centre line of the top bar
rod_length = pillar_x - LIDAR_SPACING_X / 2


def concave_line_edges(part: Part, margin: float = 0.05) -> list:
    """Straight edges where two planar faces meet at an inside (reflex) corner."""
    faces_of = {}
    for face in part.faces():
        for edge in face.edges():
            faces_of.setdefault(edge, []).append(face)
    result = []
    for edge, faces in faces_of.items():
        if edge.geom_type != GeomType.LINE or len(faces) != 2:
            continue
        if any(f.geom_type != GeomType.PLANE for f in faces):
            continue
        p = edge.center()
        n1, n2 = (f.normal_at(p) for f in faces)
        if abs(n1.dot(n2)) > 0.99:
            continue
        if part.is_inside(p + (n1 - n2) * margin) and part.is_inside(p + (n2 - n1) * margin):
            result.append(edge)
    return result


def finish(body: Part, bed_z: float, flat_z: float | None = None) -> Part:
    """Round the inner corners, then chamfer the edges that lie on the bed face.

    Horizontal inner corners at flat_z stay sharp, so a part resting on that plane lies flat.
    """
    fillet_edges = [
        e for e in concave_line_edges(body)
        if flat_z is None or not all(abs(v.Z - flat_z) < 1e-6 for v in e.vertices())
    ]
    body = body.fillet(INNER_FILLET, fillet_edges)
    bed_edges = [e for e in body.edges() if all(abs(v.Z - bed_z) < 1e-6 for v in e.vertices())]
    return body.chamfer(BED_CHAMFER, None, bed_edges)


# ---- base ---------------------------------------------------------------------------------
with BuildPart() as base_body:
    with Locations((0, 0, bottom_mid_z)):
        Box(outer_x, outer_y, ROD_HEIGHT)
        Box(outer_x - 2 * ROD_WIDTH, outer_y - 2 * ROD_WIDTH, ROD_HEIGHT, mode=Mode.SUBTRACT)
    with Locations((0, 0, bottom_mid_z)):
        Box(2 * frame_outer_x, 2 * frame_outer_y, ROD_HEIGHT)
        Box(board_inner_x, board_inner_y, ROD_HEIGHT, mode=Mode.SUBTRACT)

    # board mounts: pad under each hole, rod to the nearest X side of the board frame
    for x, y in BOARD_HOLES:
        sign = 1 if x > 0 else -1
        with Locations((x, y, bottom_z + pad_height / 2)):
            Cylinder(BOARD_PAD_DIAMETER / 2, pad_height)
        board_rod_length = frame_outer_x - abs(x)
        with Locations((x + sign * board_rod_length / 2, y, bottom_mid_z)):
            Box(board_rod_length, ROD_WIDTH, ROD_HEIGHT)

    with Locations(*[(x, y, bottom_mid_z) for x, y in ROBOT_HOLES]):
        Cylinder(ROBOT_BOSS_DIAMETER / 2, ROD_HEIGHT)

    pillar_height = split_z - bottom_z
    with Locations(*[(x, y, bottom_z + pillar_height / 2) for x, y in PILLARS]):
        Box(ROD_WIDTH, ROD_WIDTH, pillar_height)

with BuildPart() as base:
    add(finish(base_body.part, bottom_z, flat_z=bottom_top_z))
    ear_z = bottom_z + MOUSE_EAR_HEIGHT / 2
    with Locations(*[(sx * frame_outer_x, sy * frame_outer_y, ear_z) for sx in (1, -1) for sy in (1, -1)]):
        Cylinder(MOUSE_EAR_DIAMETER / 2, MOUSE_EAR_HEIGHT)
    with Locations(*[(x, y, bottom_z + pad_height / 2) for x, y in BOARD_HOLES]):
        Cylinder(BOARD_INSERT_DIAMETER / 2, pad_height, mode=Mode.SUBTRACT)
    with Locations(*[(x, y, split_z - SPLIT_INSERT_DEPTH / 2) for x, y in PILLARS]):
        Cylinder(SPLIT_INSERT_DIAMETER / 2, SPLIT_INSERT_DEPTH, mode=Mode.SUBTRACT)
    with Locations(*ROBOT_HOLES):
        with Locations((0, 0, bottom_mid_z)):
            Cylinder(robot_hole / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
        with Locations((0, 0, bottom_top_z - COUNTERSINK_DEPTH / 2)):
            Cone(robot_hole / 2, COUNTERSINK_DIAMETER / 2, COUNTERSINK_DEPTH, mode=Mode.SUBTRACT)

# ---- top piece, the +X one in assembly position --------------------------------------------
with BuildPart() as top_body:
    with Locations((pillar_x, 0, top_mid_z)):
        Box(ROD_WIDTH, 2 * PILLAR_Y + ROD_WIDTH, ROD_HEIGHT)
    for x, y in TOP_LIDAR_HOLES:
        with Locations((x, y, top_mid_z)):
            Cylinder(ROD_WIDTH / 2, ROD_HEIGHT)
        with Locations((x + rod_length / 2, y, top_mid_z)):
            Box(rod_length, ROD_WIDTH, ROD_HEIGHT)

with BuildPart() as top:
    add(finish(top_body.part, split_z))
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_LIDAR_HOLES]):
        Cylinder(lidar_hole / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_SCREWS]):
        Cylinder(split_screw / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)

# export_step rejects the Part that BuildPart returns after add(); its single solid exports fine
(base_solid,) = base.part.solids()
(top_solid,) = top.part.solids()
top_left = Rot(0, 0, 180) * top_solid
assembly = Compound([base_solid, top_solid, top_left])

if __name__ == "__main__":
    here = Path(__file__).parent
    base_print = Pos(0, 0, -bottom_z) * base_solid
    top_print = Pos(-pillar_x, 0, -split_z) * top_solid
    for name, part in (("lidar_cm5_stand_base", base_print), ("lidar_cm5_stand_top", top_print)):
        export_stl(part, str(here / f"{name}.stl"))
        export_step(part, str(here / f"{name}.step"))
        bb = part.bounding_box()
        print(
            f"{name}: {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, "
            f"volume {part.volume:.1f} mm3, solids={len(part.solids())}, valid={part.is_valid}"
        )
    export_step(assembly, str(here / "lidar_cm5_stand_assembly.step"))
