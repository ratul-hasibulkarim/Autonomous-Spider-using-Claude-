"""Collision primitives per link (metres, link frame).  Shared by URDF and MJCF.

Each entry: (kind, params, pos, rpy) with kind in box/cylinder/sphere/capsule.
capsule/cylinder params = (radius, half_length) along the geom z axis (oriented by rpy).
"""
from __future__ import annotations

import math

from spider import config as C
from spider import layout as LY


def collision_shapes(link: str):
    if link == "base_link":
        hl, hw = C.CHASSIS_LEN / 2000, C.CHASSIS_WID / 2000
        zc = (C.PLATE_GAP / 2) / 1000
        hz = (C.PLATE_GAP / 2 + C.PLATE_T) / 1000
        ex, ey, ez = (v / 1000 for v in LY.ENCLOSURE_SIZE)
        return [
            ("box", (hl - 0.06, hw, hz), (0, 0, zc), (0, 0, 0)),
            ("box", (hl, hw - 0.11, hz), (0, 0, zc), (0, 0, 0)),
            ("box", (ex / 2, ey / 2, ez / 2), (LY.ENCLOSURE_XY[0] / 1000, 0, LY.TOP_Z / 1000 + ez / 2), (0, 0, 0)),
            ("cylinder", (0.022, C.MAST_H / 2000), (LY.MAST_XY[0] / 1000, 0, LY.TOP_Z / 1000 + C.MAST_H / 2000), (0, 0, 0)),
        ]
    if link.endswith("_coxa"):
        return [("box", (0.052, 0.081, 0.05), (C.COXA_M, 0.0065, C.COXA_Z_M), (0, 0, 0)),
                ("cylinder", (0.03, 0.011), (0, 0, -0.009), (0, 0, 0))]
    if link.endswith("_femur"):
        return [("capsule", (0.038, C.FEMUR_L_M / 2), (C.FEMUR_L_M / 2, 0, 0), (0, math.pi / 2, 0))]
    if link.endswith("_tibia"):
        L = 0.30
        return [("capsule", (0.02, L / 2), (0.065 + L / 2, 0, 0), (0, math.pi / 2, 0)),
                ("sphere", (C.FOOT_R_M,), (C.TIBIA_L_M, 0, 0), (0, 0, 0))]
    if link == "sensor_pan":
        return [("box", (0.022, 0.006, 0.05), (0, -0.0555, C.TILT_DZ / 1000), (0, 0, 0))]
    if link == "lidar_link":
        return [("cylinder", (0.043, 0.036), (0, 0, -0.006), (0, 0, 0))]
    return []
