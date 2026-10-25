"""Simplified LDS08RR envelope model for measurements; writes lds08rr-simple.step.

Dimensions come from the user sketch of 2026-10-13 (lds08rr-dimensions.json), not
from independent measurement. Origin: rotor centre = mounting-hole pattern centre,
z = 0 at the bottom of the round housing, +z toward the PCB and motor side.
Viewed from +z, the motor lies on -X.
"""

from build123d import (
    Align, Circle, Cylinder, Pos, Rectangle, Sketch, SlotCenterPoint, export_step, extrude,
)

H_HOUSING = 17.8  # housing bottom to plate underside
Z_PLATE_TOP = 28.0  # housing bottom to plate top (PCB side)
Z_MOTOR_TOP = 38.0  # overall height
D_HOUSING = 59.0
D_PLATE = 59.6
D_MOTOR = 24.5
MOTOR_X = -(50.0 - D_MOTOR / 2)  # motor's outer edge is 50 mm from the rotor centre
EAR_T = 4.3  # ear thickness, top flush with plate top
EAR_W = 7.0  # assumed ear width
HOLE_D = 3.1
HOLES = [(sx * 27.0, sy * 22.0) for sx in (-1, 1) for sy in (-1, 1)]

BOTTOM = (Align.CENTER, Align.CENTER, Align.MIN)


def build():
    housing = Cylinder(D_HOUSING / 2, H_HOUSING, align=BOTTOM)

    plate_sk = (
        Circle(D_PLATE / 2)
        + Pos(MOTOR_X, 0) * Circle(D_MOTOR / 2)
        + Pos(MOTOR_X / 2, 0) * Rectangle(abs(MOTOR_X), D_MOTOR)
    )
    plate = Pos(0, 0, H_HOUSING) * extrude(plate_sk, Z_PLATE_TOP - H_HOUSING)

    ears_sk = Sketch()
    for x, y in HOLES:
        ears_sk += Pos(x, y) * Circle(EAR_W / 2)
        ears_sk += SlotCenterPoint((x * 0.6, y * 0.6), (x, y), EAR_W)
    ears = Pos(0, 0, Z_PLATE_TOP - EAR_T) * extrude(ears_sk, EAR_T)

    motor = Pos(MOTOR_X, 0, Z_PLATE_TOP) * Cylinder(
        D_MOTOR / 2, Z_MOTOR_TOP - Z_PLATE_TOP, align=BOTTOM
    )

    lidar = housing + plate + ears + motor
    for x, y in HOLES:
        lidar -= Pos(x, y, Z_PLATE_TOP - EAR_T) * Cylinder(HOLE_D / 2, EAR_T, align=BOTTOM)
    return lidar


result = build()

if __name__ == "__main__":
    export_step(result, "lds08rr-simple.step")  # run from this folder
