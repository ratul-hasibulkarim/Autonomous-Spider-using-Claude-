"""Render the assembled robot (per-part colours) at a joint configuration.

    python tools/render_robot.py out.png [elev azim]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spider.robot import build, standing_pose  # noqa: E402
from tools.render import render  # noqa: E402


def robot_tris(q=None, links=None, tol=0.6):
    r = build()
    poses = r.link_poses(q if q is not None else standing_pose())
    groups = {}
    for inst in r.instances:
        if links and inst.link not in links:
            continue
        small = inst.variant.pdef.category in ("fastener", "electronic", "cable", "bearing")
        V, F = inst.variant.mesh(tol * 2, 0.9) if small else inst.variant.mesh(tol, 0.5)
        T = poses[inst.link] @ inst.T
        tris = (V @ T[:3, :3].T + T[:3, 3])[F]
        key = tuple(np.round(inst.variant.rgba[:3], 3))
        groups.setdefault(key, []).append(tris)
    return [(np.concatenate(v), k) for k, v in groups.items()]


if __name__ == "__main__":
    a = sys.argv[1:]
    el, az = (float(a[1]), float(a[2])) if len(a) > 2 else (24, -55)
    render(robot_tris(), a[0], el, az, size=(12, 8), dpi=120, zoom=1.25)
