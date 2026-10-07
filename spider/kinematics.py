"""Leg kinematics (metres, radians).  Shared by the exporters and the controller.

Hip frame: origin on the coxa axis at the hip plane, x along the leg's
neutral direction, z up.  Femur and knee rotate about +y, so a positive angle
drives the distal link downward:

    x = L1 + L2 cos q2 + L3 cos(q2+q3)           (in the coxa-rotated frame)
    z = Zc - L2 sin q2 - L3 sin(q2+q3)

The knee is belt-driven from a hip-mounted motor, so the knee *motor* angle
is q2 + q3 (the tibia's absolute pitch in the coxa frame).
"""
from __future__ import annotations

import numpy as np

from spider import config as C

L1, ZC, L2, L3 = C.COXA_M, C.COXA_Z_M, C.FEMUR_L_M, C.TIBIA_L_M


def leg_fk(q):
    """q (...,3) -> foot centre in hip frame (...,3)."""
    q = np.asarray(q, float)
    q1, q2, q3 = q[..., 0], q[..., 1], q[..., 2]
    rho = L1 + L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    z = ZC - L2 * np.sin(q2) - L3 * np.sin(q2 + q3)
    return np.stack([rho * np.cos(q1), rho * np.sin(q1), z], axis=-1)


def leg_ik(p, clamp=True):
    """Foot centre in hip frame (...,3) -> joint angles (...,3).  Knee-up branch.

    Unreachable targets are pulled onto the workspace boundary (clamp=True)."""
    p = np.asarray(p, float)
    px, py, pz = p[..., 0], p[..., 1], p[..., 2]
    q1 = np.arctan2(py, px)
    rho = np.hypot(px, py) - L1
    h = ZC - pz
    d2 = rho * rho + h * h
    c3 = (d2 - L2 * L2 - L3 * L3) / (2 * L2 * L3)
    if clamp:
        c3 = np.clip(c3, np.cos(C.KNEE_LIMIT[1]), np.cos(C.KNEE_LIMIT[0]))
    q3 = np.arccos(c3)
    q2 = np.arctan2(h, rho) - np.arctan2(L3 * np.sin(q3), L2 + L3 * np.cos(q3))
    return np.stack([q1, q2, q3], axis=-1)


def leg_jacobian(q):
    """d(foot)/d(q) in hip frame, (...,3,3) - used for force estimation."""
    q = np.asarray(q, float)
    q1, q2, q3 = q[..., 0], q[..., 1], q[..., 2]
    rho = L1 + L2 * np.cos(q2) + L3 * np.cos(q2 + q3)
    drho2 = -L2 * np.sin(q2) - L3 * np.sin(q2 + q3)
    drho3 = -L3 * np.sin(q2 + q3)
    dz2 = -L2 * np.cos(q2) - L3 * np.cos(q2 + q3)
    dz3 = -L3 * np.cos(q2 + q3)
    c1, s1 = np.cos(q1), np.sin(q1)
    J = np.zeros(q.shape[:-1] + (3, 3))
    J[..., 0, 0], J[..., 1, 0] = -rho * s1, rho * c1
    J[..., 0, 1], J[..., 1, 1], J[..., 2, 1] = drho2 * c1, drho2 * s1, dz2
    J[..., 0, 2], J[..., 1, 2], J[..., 2, 2] = drho3 * c1, drho3 * s1, dz3
    return J
