"""Rigid-body properties of each link, from the exact B-rep of every part.

Each part's mass, centre of mass and inertia tensor (about its own CoM) come
from OpenCascade; they are rotated into the link frame and summed with the
parallel-axis theorem.  Output in SI (kg, m, kg m^2).
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class Inertial:
    mass: float
    com: np.ndarray        # (3,) m, link frame
    inertia: np.ndarray    # (3,3) kg m^2 about com, link-frame axes
    n_parts: int


def link_inertials(robot) -> dict[str, Inertial]:
    out = {}
    for link in robot.links:
        parts = robot.instances_on(link)
        if not parts:
            raise ValueError(f"link {link} has no parts")
        ms, cs, Is = [], [], []
        for inst in parts:
            m, c, I = inst.variant.massprops()
            R, p = inst.T[:3, :3], inst.T[:3, 3]
            ms.append(m)
            cs.append(R @ c + p)
            Is.append(R @ I @ R.T)
        ms = np.array(ms)
        cs = np.array(cs)
        M = ms.sum()
        com = (ms[:, None] * cs).sum(0) / M
        I = np.zeros((3, 3))
        for m, c, Ic in zip(ms, cs, Is):
            d = c - com
            I += Ic + m * (d @ d * np.eye(3) - np.outer(d, d))
        out[link] = Inertial(M, com / 1000.0, I * 1e-6, len(parts))
    return out


def principal(I: np.ndarray):
    """Eigen-decomposition with a right-handed frame: (diag, R)."""
    w, V = np.linalg.eigh(I)
    if np.linalg.det(V) < 0:
        V[:, 0] *= -1
    return w, V
