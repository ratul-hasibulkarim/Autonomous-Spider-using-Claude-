"""4x4 homogeneous transforms (numpy) and conversions to CadQuery locations."""
from __future__ import annotations

import math

import numpy as np


def trans(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    m = np.eye(4)
    m[:3, 3] = (x, y, z)
    return m


def _rot(axis: int, deg: float) -> np.ndarray:
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    m = np.eye(4)
    i, j = [(1, 2), (2, 0), (0, 1)][axis]
    m[i, i], m[i, j], m[j, i], m[j, j] = c, -s, s, c
    return m


def rotx(deg):
    return _rot(0, deg)


def roty(deg):
    return _rot(1, deg)


def rotz(deg):
    return _rot(2, deg)


def chain(*ms) -> np.ndarray:
    out = np.eye(4)
    for m in ms:
        out = out @ m
    return out


def rpy_of(m: np.ndarray) -> tuple[float, float, float]:
    """URDF fixed-axis roll/pitch/yaw (rad) of a rotation matrix."""
    r = m[:3, :3]
    pitch = math.atan2(-r[2, 0], math.hypot(r[0, 0], r[1, 0]))
    if abs(math.cos(pitch)) < 1e-9:
        return math.atan2(-r[1, 2], r[1, 1]), pitch, 0.0
    return math.atan2(r[2, 1], r[2, 2]), pitch, math.atan2(r[1, 0], r[0, 0])


def quat_of(m: np.ndarray) -> tuple[float, float, float, float]:
    """(w, x, y, z) quaternion of a rotation matrix."""
    r = m[:3, :3]
    t = np.trace(r)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = (0.25 * s, (r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s)
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k]) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = 0.25 * s
        v[j] = (r[j, i] + r[i, j]) / s
        v[k] = (r[k, i] + r[i, k]) / s
        q = ((r[k, j] - r[j, k]) / s, *v)
    w, x, y, z = q
    if w < 0:
        w, x, y, z = -w, -x, -y, -z
    return w, x, y, z


def axis_angle(axis, ang_rad) -> np.ndarray:
    """Rotation about a unit axis through the origin (rad)."""
    x, y, z = np.asarray(axis, float) / np.linalg.norm(axis)
    c, s, C = math.cos(ang_rad), math.sin(ang_rad), 1 - math.cos(ang_rad)
    m = np.eye(4)
    m[:3, :3] = [
        [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
        [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
        [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
    ]
    return m


def to_location(m: np.ndarray):
    """numpy 4x4 (mm) -> cadquery.Location."""
    import cadquery as cq
    from OCP.gp import gp_Trsf

    t = gp_Trsf()
    r = m[:3, :3]
    p = m[:3, 3]
    t.SetValues(r[0, 0], r[0, 1], r[0, 2], p[0],
                r[1, 0], r[1, 1], r[1, 2], p[1],
                r[2, 0], r[2, 1], r[2, 2], p[2])
    return cq.Location(t)
