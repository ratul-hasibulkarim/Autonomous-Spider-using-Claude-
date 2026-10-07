"""Positions shared between part programs and the assembly (mm, base_link)."""
from __future__ import annotations

import math

from spider import config as C

# Hex standoffs tying top and bottom plates together.
STANDOFFS = [
    (240.0, 280.0), (0.0, 285.0), (-240.0, 280.0),
    (240.0, -280.0), (0.0, -285.0), (-240.0, -280.0),
    (400.0, 130.0), (400.0, -130.0), (-400.0, 130.0), (-400.0, -130.0),
]

# Leg driver cages (3 boards each) between the plates, one per leg, by leg index.
CAGE_X = (330.0, 115.0, -115.0, -330.0)
CAGE_Y = 150.0

HIPS = [C.hip_pose(leg) for leg in C.LEG_NAMES]

# Battery between the plates (footprint 295 x 189 for 13S5P).
BATTERY_XY = (-10.0, 0.0)
BATTERY_CLAMP_X = (-170.0, 150.0)   # bolt line of each end clamp

# Bulkheads (front / rear) - vertical C-channels between the plates.
BULKHEAD_X = 418.0
BULKHEAD_W = 200.0
BULKHEAD_SCREW_Y = (-60.0, 60.0)

# Top deck electronics (x, y) of each board centre.
ENCLOSURE_XY = (-30.0, 0.0)
ENCLOSURE_SIZE = (380.0, 260.0, 95.0)
JETSON_XY = (-145.0, 0.0)
PDB_XY = (95.0, 55.0)
DCDC_XY = [(105.0, -50.0), (40.0, -80.0)]
CAN_XY = (-52.0, 0.0)          # mounted rotated 90 deg
ETH_XY = (-30.0, -75.0)
GIMBAL_CAGE_XY = (-10.0, 85.0)  # 2-slot cage: pan + tilt drivers
IMU_XY = (0.0, 0.0)

MAST_XY = (C.MAST_X, 0.0)
CAMERA_X = 400.0
HANDLE_XY = [(-20.0, 185.0), (-20.0, -185.0)]
HANDLE_SPAN = 150.0
ESTOP_XY = (-330.0, 0.0)
BEACON_XY = (-330.0, 110.0)
ANTENNA_XY = [(-300.0, -110.0), (-360.0, -110.0)]

TOP_Z = C.TOP_PLATE_Z + C.PLATE_T   # top surface of top plate

# board -> (size x, size y, hole spacing x, hole spacing y)
BOARDS = {
    "jetson": (130, 115, 118, 103), "pdb": (120, 90, 110, 80), "can": (50, 80, 42, 72),
    "eth": (70, 55, 62, 47), "dcdc": (61, 58, 50, 48),
}


def bolt_circle(r: float, n: int, start_deg: float = 0.0):
    return [(r * math.cos(math.radians(start_deg + i * 360 / n)),
             r * math.sin(math.radians(start_deg + i * 360 / n))) for i in range(n)]


def rect_pattern(cx, cy, w, h):
    return [(cx + sx * w / 2, cy + sy * h / 2) for sx in (-1, 1) for sy in (-1, 1)]


def enclosure_flange_holes():
    (cx, cy), (w, h, _) = ENCLOSURE_XY, ENCLOSURE_SIZE
    f = 7.0  # flange hole inset from wall
    pts = [(cx + dx, cy + sy * (h / 2 + f)) for dx in (-w / 4, w / 4) for sy in (-1, 1)]
    pts += [(cx + sx * (w / 2 + f), cy) for sx in (-1, 1)]
    return pts


def board_holes():
    """(x, y) of every M3 PCB standoff on the top deck."""
    pts = []
    for key, xy in [("jetson", JETSON_XY), ("pdb", PDB_XY), ("can", CAN_XY), ("eth", ETH_XY),
                    ("dcdc", DCDC_XY[0]), ("dcdc", DCDC_XY[1])]:
        _, _, hx, hy = BOARDS[key]
        pts += rect_pattern(xy[0], xy[1], hx, hy)
    return pts


def hip_bolt_holes():
    """Actuator back-face bolts through the top plate: (x, y) per hip."""
    out = []
    for hx, hy, yaw in HIPS:
        for x, y in bolt_circle(C.QDD_BOLT_CIRCLE / 2, 6, 30 + yaw):
            out.append((hx + x, hy + y))
    return out


def bulkhead_holes():
    return [(sx * BULKHEAD_X, y) for sx in (-1, 1) for y in BULKHEAD_SCREW_Y]


def handle_holes():
    return [(x + sx * HANDLE_SPAN / 2, y) for x, y in HANDLE_XY for sx in (-1, 1)]


def mast_holes():
    return rect_pattern(MAST_XY[0], MAST_XY[1], 46, 46)


def battery_clamp_holes():
    return [(x, y) for x in BATTERY_CLAMP_X for y in (-25.0, 25.0)]


def imu_holes():
    return [(IMU_XY[0] + sx * 21.0, IMU_XY[1]) for sx in (-1, 1)]


def camera_holes():
    return [(CAMERA_X - 10.0, sy * 20.0) for sy in (-1, 1)]


CAGE_HOLE_DX = 30.0


def cage_positions():
    """(x, y) of each leg's driver cage, keyed by leg name."""
    out = {}
    for leg in C.LEG_NAMES:
        s = 1.0 if leg[0] == "L" else -1.0
        out[leg] = (CAGE_X[int(leg[1]) - 1], s * CAGE_Y)
    return out


def cage_holes(xy):
    return [(xy[0] + sx * CAGE_HOLE_DX, xy[1]) for sx in (-1, 1)]
