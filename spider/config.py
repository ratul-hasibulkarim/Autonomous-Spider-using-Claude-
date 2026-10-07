"""Single source of truth for every dimension that more than one program uses.

CAD programs work in millimetres; URDF / MJCF / the controller work in metres.
Everything here is in millimetres unless the name ends in ``_M``.

Body frame (``base_link``): x forward, y left, z up.  Origin is the centre of
the chassis, on the top face of the bottom plate.  Each coxa (hip-yaw) joint
sits on that plane at a hip position on the chassis perimeter.
"""
from __future__ import annotations

import math

# --------------------------------------------------------------------------
# Leg layout
# --------------------------------------------------------------------------
LEG_NAMES = ("L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4")

HIP_X = (360.0, 120.0, -120.0, -360.0)  # front -> rear
HIP_Y = 250.0
# Nominal outward direction of each leg (deg from +x), left side; right is mirrored.
LEG_YAW_LEFT = (40.0, 78.0, 102.0, 140.0)


def hip_pose(leg: str) -> tuple[float, float, float]:
    """(x, y, yaw_deg) of a leg's coxa joint in base_link, mm / deg."""
    side, idx = leg[0], int(leg[1]) - 1
    s = 1.0 if side == "L" else -1.0
    return HIP_X[idx], s * HIP_Y, s * LEG_YAW_LEFT[idx]


# --------------------------------------------------------------------------
# Leg kinematics (identical for all 8 legs - one leg SKU)
# --------------------------------------------------------------------------
COXA_X = 65.0     # coxa axis -> femur axis, along the leg
COXA_Z = -75.0    # femur axis sits below the hip plane
FEMUR_L = 320.0   # femur axis -> knee axis
TIBIA_L = 400.0   # knee axis -> foot-sphere centre
FOOT_R = 30.0     # foot pad radius

COXA_M, COXA_Z_M = COXA_X / 1000, COXA_Z / 1000
FEMUR_L_M, TIBIA_L_M, FOOT_R_M = FEMUR_L / 1000, TIBIA_L / 1000, FOOT_R / 1000

# Joint limits (rad).  Femur/knee: positive rotation about +y drives the
# distal link downward.  Knee motor angle = femur + knee (1:1 belt).
COXA_LIMIT = (-math.radians(42), math.radians(42))
FEMUR_LIMIT = (-math.radians(55), math.radians(65))   # -60 grazes the chassis (tools/interference.py --sweep)
KNEE_LIMIT = (math.radians(10), math.radians(125))

# --------------------------------------------------------------------------
# Actuators
# --------------------------------------------------------------------------
QDD_D, QDD_T = 96.0, 48.0          # leg actuator envelope (dia, thickness)
QDD_MASS = 0.96                    # kg, incl. integrated encoder
QDD_GEAR = 9.0
QDD_TAU_PEAK = 110.0               # N m at output
QDD_TAU_CONT = 42.0
QDD_SPEED = 14.0                   # rad/s no-load at 48 V
QDD_ROTOR_INERTIA = 3.1e-4         # kg m^2 at motor side
QDD_BOLT_CIRCLE = 78.0
QDD_OUT_BOLT_CIRCLE = 40.0

GIMBAL_D, GIMBAL_T, GIMBAL_MASS = 62.0, 26.0, 0.21
GIMBAL_TAU_PEAK = 2.5

# --------------------------------------------------------------------------
# Chassis
# --------------------------------------------------------------------------
PLATE_T = 2.5
PLATE_GAP = QDD_T + 2.0            # actuator sits between the plates
TOP_PLATE_Z = PLATE_GAP            # underside of top plate
CHASSIS_LEN = 880.0
CHASSIS_WID = 620.0
CHASSIS_CORNER_R = 110.0

# Femur box inner width (belt lives inside)
FEMUR_INNER_W = 30.0
FEMUR_PLATE_T = 3.0

# Belt drive
PULLEY_TEETH = 30
PULLEY_PD = PULLEY_TEETH * 5.0 / math.pi  # HTD-5M pitch diameter
BELT_W = 15.0

# Gimbal / sensor head
MAST_X, MAST_H = 300.0, 110.0
PAN_Z = TOP_PLATE_Z + PLATE_T + MAST_H
TILT_DZ = 62.0

# Standing pose (used for the STEP assembly and as controller nominal)
STAND_RADIUS = 460.0   # foot distance from coxa axis (mm)
STAND_HEIGHT = 300.0   # foot centre below femur axis plane z (mm)


def mm(v):
    return v / 1000.0
