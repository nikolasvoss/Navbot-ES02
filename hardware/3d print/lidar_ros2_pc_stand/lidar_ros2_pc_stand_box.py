"""Stand for the robot (bottom frame), a ros2_pc IO board (board frame) and the LDS08RR lidar (top).

Derived from lidar_ros2_pc_stand.py. Total height is 26 mm outer to outer. The stand prints as
three parts without support material: one base and two identical top pieces.

Base, printed bottom-down:
- Bottom frame: robot mounting holes from docs/robot/hardware/mechanics/touchscreenstand.md
  (96.4 x 72.9 mm), each with a 90 degree countersink, 6.1 mm wide, on its upper face, in a
  round boss that thickens the wall around the countersink.
- Board frame: an 8 x 5 mm frame on the print bed that runs around the 90 x 160 mm ros2_pc IO board.
  Its top is flush with the bottom frame, so the robot screw countersinks stay open.
  The board lies across the stand: its 160 mm side is parallel to the short (Y) side.
- Board mounts: the four corner holes of the Waveshare ros2_pc IO board drawing (121.5 x 69 mm),
  each on a pad under the board with a 1 mm pilot hole (drill out for the screw), joined to the board frame by an
  8 x 5 mm rod. The pads stand BOARD_PAD_LIFT above the frames, so the board rests only on them. Board coordinates (u along 160 mm, v along 90 mm, origin at the board corner)
  map to stand x = v - 45, y = 80 - u.
- Four pillars beside the board, PILLAR_BOARD_GAP from its edge and clear of the ports, ending at the split plane with a small pilot
  hole each; drill it out for the chosen screw or insert.

Top piece, print 2x, printed upside down (lidar face on the bed): an 8 x 5 mm bar across the IO
board (along X) at y = +-PILLAR_Y, screwed onto the two pillars on one long side, and an 8 mm wide
arc around a round keep-out for the lidar housing and electronics. The arc carries two of the
LDS08RR holes from docs/ros2_pc/hardware/mechanics/lds08rr-dimensions.json, 54 mm apart along X
(the 44 mm pair spacing runs along Y), as 1 mm pilot holes. The keep-out is assumed centred on the
hole pattern (the JSON does not establish that). The +Y and -Y pieces are the same part,
rotated 180 deg. All vertical edges are rounded. Over each pillar the bar ends in a round pad,
SPLIT_PAD_DIAMETER wide, with a pilot hole to be drilled out like the pillar holes and a 90 deg
countersink for an M3 countersunk screw (DIN 7991) in the lidar face.

Print details: 0.5 mm chamfer on all bed edges against elephant's foot, rounded vertical edges
(larger at the four outer base corners, where warping starts), 1.5 mm fillets on the inner
corners, and HOLE_COMPENSATION added to the screw holes because small vertical holes print
undersize. No mouse ears: with rounded corners PLA and PETG hold without them; for ABS/ASA add
brim ears in the slicer.

Run: python3 lidar_ros2_pc_stand_box.py  -> writes lidar_ros2_pc_stand_base.* and lidar_ros2_pc_stand_top.*
(STL and STEP, in print orientation) and lidar_ros2_pc_stand_assembly.step.
"""
from math import hypot, radians, sqrt, tan
from pathlib import Path

from build123d import (
    Axis,
    Box,
    BuildPart,
    BuildSketch,
    Compound,
    Cone,
    Cylinder,
    GeomType,
    Locations,
    Mode,
    Part,
    Plane,
    Polygon,
    Pos,
    Rot,
    add,
    export_step,
    extrude,
    export_stl,
)

HOLE_SPACING_X = 96.4  # mm, robot holes, centre to centre
HOLE_SPACING_Y = 72.9  # mm
HOLE_DIAMETER = 2.2  # mm
COUNTERSINK_DIAMETER = 6.1  # mm, at the upper face of the bottom frame
COUNTERSINK_ANGLE = 90.0  # degrees, included angle
BOARD_X = 90.0  # mm, ros2_pc IO board, across the stand
BOARD_Y = 160.0  # mm, parallel to the short side of the stand
BOARD_CLEARANCE = 0.5  # mm per side between board and board frame, along Y
BOARD_HOLES_UV = [(11.0, 8.0), (11.0, 77.0), (132.5, 8.0), (132.5, 77.0)]  # mm, from board corner
BOARD_SCREW_HOLE = 1.0  # mm, pilot hole, drill out for the board screws
BOARD_PAD_DIAMETER = 8.0  # mm
BOARD_PAD_LIFT = 1.0  # mm, pad top above the bottom frame top
ROBOT_BOSS_DIAMETER = 12.0  # mm, round boss around each robot hole
LIDAR_SPACING_X = 54.0  # mm, LDS08RR holes, centre to centre
LIDAR_SPACING_Y = 44.0  # mm
LIDAR_SCREW_HOLE = 1.0  # mm, pilot hole, drill out for the lidar screws
PILLAR_BOARD_GAP = 2.0  # mm, board edge to pillar
PILLAR_DEPTH = 10.0  # mm, along Y; reaches onto the robot hole boss so the pillar corner rests on it
SPLIT_PILOT_DIAMETER = 2.2  # mm, pilot hole in pillar tops and top piece, drill out as needed
SPLIT_PILOT_DEPTH = 8.0  # mm, into each pillar
SPLIT_COUNTERSINK_DIAMETER = 6.0  # mm, M3 countersunk head (DIN 7991), in the top piece's lidar face
SPLIT_PAD_DIAMETER = 12.0  # mm, round pad of the top piece around each pillar screw
HOLE_COMPENSATION = 0.2  # mm, added to screw holes; insert holes use the maker's printed size
ROD_WIDTH = 8.0  # mm
ROD_HEIGHT = 5.0  # mm
TOTAL_HEIGHT = 26.0  # mm, outer to outer
PILLAR_Y = 27.0  # mm, pillar and top bar centre line, clear of the USB jack
BED_CHAMFER = 0.5  # mm
INNER_FILLET = 1.5  # mm
OUTER_CORNER_FILLET = 4.0  # mm, the four outer corners of the base
LIDAR_KEEPOUT_DIAMETER = 61.0  # mm, 59 mm lidar housing plus 1 mm per side, kept free in the top

robot_hole = HOLE_DIAMETER + HOLE_COMPENSATION
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
board_inner_x = BOARD_X + 2 * PILLAR_BOARD_GAP  # the pillars stand flush in the X sides
board_inner_y = BOARD_Y + 2 * BOARD_CLEARANCE
frame_outer_x = board_inner_x / 2 + ROD_WIDTH
frame_outer_y = board_inner_y / 2 + ROD_WIDTH
pillar_x = board_inner_x / 2 + ROD_WIDTH / 2  # centre line of the board frame X sides
ROBOT_HOLES = [(sx * HOLE_SPACING_X / 2, sy * HOLE_SPACING_Y / 2) for sx in (1, -1) for sy in (1, -1)]
BOARD_HOLES = [(v - BOARD_X / 2, BOARD_Y / 2 - u) for u, v in BOARD_HOLES_UV]
PILLARS = [(sx * pillar_x, sy * PILLAR_Y) for sx in (1, -1) for sy in (1, -1)]
TOP_LIDAR_HOLES = [(sx * LIDAR_SPACING_X / 2, LIDAR_SPACING_Y / 2) for sx in (1, -1)]
TOP_SCREWS = [(sx * pillar_x, PILLAR_Y) for sx in (1, -1)]

# arc between the lidar holes, concentric with the keep-out; its centre line runs through the
# holes, and it ends in a half-round cap around each hole
arc_mid_r = hypot(LIDAR_SPACING_X / 2, LIDAR_SPACING_Y / 2)
arc_inner_r = arc_mid_r - ROD_WIDTH / 2
arc_outer_r = arc_mid_r + ROD_WIDTH / 2
assert arc_inner_r >= LIDAR_KEEPOUT_DIAMETER / 2, "arc would cut into the lidar keep-out"
bar_inner_x = sqrt(arc_mid_r**2 - PILLAR_Y**2)  # the bar from the pillar ends on the arc centre line


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


def round_vertical_edges(body: Part, flat_z: float) -> Part:
    """Round the four outer corners wide, then every other vertical edge and the inner corners."""
    corners = [
        e for e in body.edges().filter_by(Axis.Z)
        if abs(abs(e.center().X) - frame_outer_x) < 1e-6 and abs(abs(e.center().Y) - frame_outer_y) < 1e-6
    ]
    body = body.fillet(OUTER_CORNER_FILLET, corners)
    faces_of = {}
    for face in body.faces():
        for edge in face.edges():
            faces_of.setdefault(edge, []).append(face)

    def is_corner(edge) -> bool:
        """Edge between two distinct faces that meet at an angle (not a seam, not tangent)."""
        faces = faces_of.get(edge, [])
        if len(faces) != 2 or faces[0] == faces[1]:
            return False
        p = edge.center()
        return abs(faces[0].normal_at(p).dot(faces[1].normal_at(p))) < 0.999

    vertical = [
        e for e in body.edges().filter_by(Axis.Z) if e.geom_type == GeomType.LINE and is_corner(e)
    ]
    inner = [e for e in concave_line_edges(body) if e not in vertical]
    inner = [e for e in inner if not all(abs(v.Z - flat_z) < 1e-6 for v in e.vertices())]
    edges = vertical + inner
    try:
        return body.fillet(INNER_FILLET, edges)
    except ValueError:
        # some corners (e.g. a pillar face running flush into the frame face below) cannot be
        # rounded; keep every edge that rounds together with the ones already accepted
        accepted = []
        for edge in edges:
            try:
                body.fillet(INNER_FILLET, accepted + [edge])
                accepted.append(edge)
            except ValueError:
                c = edge.center()
                print(f"  sharp edge kept at ({c.X:.1f}, {c.Y:.1f}, {c.Z:.1f})")
        return body.fillet(INNER_FILLET, accepted)


def chamfer_bed(body: Part, bed_z: float) -> Part:
    """Chamfer the edges that lie on the bed face."""
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
        Box(ROD_WIDTH, PILLAR_DEPTH, pillar_height)

with BuildPart() as base:
    add(chamfer_bed(round_vertical_edges(base_body.part, flat_z=bottom_top_z), bottom_z))
    with Locations(*[(x, y, bottom_z + pad_height / 2) for x, y in BOARD_HOLES]):
        Cylinder(BOARD_SCREW_HOLE / 2, pad_height, mode=Mode.SUBTRACT)
    with Locations(*[(x, y, split_z - SPLIT_PILOT_DEPTH / 2) for x, y in PILLARS]):
        Cylinder(SPLIT_PILOT_DIAMETER / 2, SPLIT_PILOT_DEPTH, mode=Mode.SUBTRACT)
    with Locations(*ROBOT_HOLES):
        with Locations((0, 0, bottom_mid_z)):
            Cylinder(robot_hole / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
        with Locations((0, 0, bottom_top_z - COUNTERSINK_DEPTH / 2)):
            Cone(robot_hole / 2, COUNTERSINK_DIAMETER / 2, COUNTERSINK_DEPTH, mode=Mode.SUBTRACT)

# ---- top piece, the +Y one in assembly position --------------------------------------------
with BuildPart() as arc:
    with Locations((0, 0, top_mid_z)):
        Cylinder(arc_outer_r, ROD_HEIGHT)
        Cylinder(arc_inner_r, ROD_HEIGHT, mode=Mode.SUBTRACT)
    # keep the part between the two hole directions, on the +Y side
    hx, hy = (2 * arc_outer_r * c / arc_mid_r for c in TOP_LIDAR_HOLES[0])
    with BuildSketch(Plane.XY.offset(split_z)):
        Polygon((0, 0), (hx, hy), (hx, 2 * arc_outer_r), (-hx, 2 * arc_outer_r), (-hx, hy), align=None)
    extrude(amount=ROD_HEIGHT, mode=Mode.INTERSECT)

with BuildPart() as top_body:
    add(arc.part)
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_LIDAR_HOLES]):
        Cylinder(ROD_WIDTH / 2, ROD_HEIGHT)
    bar_length = pillar_x + ROD_WIDTH / 2 - bar_inner_x
    with Locations(*[(sx * (bar_inner_x + bar_length / 2), PILLAR_Y, top_mid_z) for sx in (1, -1)]):
        Box(bar_length, ROD_WIDTH, ROD_HEIGHT)
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_SCREWS]):
        Cylinder(SPLIT_PAD_DIAMETER / 2, ROD_HEIGHT)
    with Locations((0, 0, top_mid_z)):
        Cylinder(arc_inner_r, ROD_HEIGHT, mode=Mode.SUBTRACT)

with BuildPart() as top:
    rounded = top_body.part.fillet(INNER_FILLET, top_body.part.edges().filter_by(Axis.Z))
    add(chamfer_bed(rounded, split_z + ROD_HEIGHT))
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_LIDAR_HOLES]):
        Cylinder(LIDAR_SCREW_HOLE / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
    split_countersink_depth = (
        (SPLIT_COUNTERSINK_DIAMETER - SPLIT_PILOT_DIAMETER) / 2 / tan(radians(COUNTERSINK_ANGLE / 2))
    )
    with Locations(*[(x, y, top_mid_z) for x, y in TOP_SCREWS]):
        Cylinder(SPLIT_PILOT_DIAMETER / 2, ROD_HEIGHT, mode=Mode.SUBTRACT)
    with Locations(*[(x, y, split_z + ROD_HEIGHT - split_countersink_depth / 2) for x, y in TOP_SCREWS]):
        Cone(SPLIT_PILOT_DIAMETER / 2, SPLIT_COUNTERSINK_DIAMETER / 2, split_countersink_depth, mode=Mode.SUBTRACT)

# export_step rejects the Part that BuildPart returns after add(); its single solid exports fine
(base_solid,) = base.part.solids()
(top_solid,) = top.part.solids()
top_left = Rot(0, 0, 180) * top_solid
assembly = Compound([base_solid, top_solid, top_left])

if __name__ == "__main__":
    here = Path(__file__).parent
    base_print = Pos(0, 0, -bottom_z) * base_solid
    flipped = Rot(180, 0, 0) * top_solid  # lidar face down
    fb = flipped.bounding_box()
    top_print = Pos(-fb.center().X, -fb.center().Y, -fb.min.Z) * flipped
    for name, part in (("lidar_ros2_pc_stand_base", base_print), ("lidar_ros2_pc_stand_top", top_print)):
        export_stl(part, str(here / f"{name}.stl"))
        export_step(part, str(here / f"{name}.step"))
        bb = part.bounding_box()
        print(
            f"{name}: {bb.size.X:.2f} x {bb.size.Y:.2f} x {bb.size.Z:.2f} mm, "
            f"volume {part.volume:.1f} mm3, solids={len(part.solids())}, valid={part.is_valid}"
        )
    export_step(assembly, str(here / "lidar_ros2_pc_stand_assembly.step"))
