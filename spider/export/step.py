"""STEP AP214 assembly of every part instance, posed in the standing stance,
with part names and colours (XCAF).  Identical parts share one shape."""
from __future__ import annotations

from pathlib import Path

import cadquery as cq

from spider.tf import to_location


def export_step(robot, q: dict, path: Path):
    poses = robot.link_poses(q)
    assy = cq.Assembly(name="autonomous_spider")
    for inst in robot.instances:
        T = poses[inst.link] @ inst.T
        assy.add(inst.variant.shape(), name=f"{inst.variant.number}:{inst.name.replace('/', ':')}",
                 loc=to_location(T), color=cq.Color(*inst.variant.rgba))
    assy.export(str(path), "STEP")
    return len(robot.instances)


def export_part_steps(robot, out_dir: Path):
    out_dir.mkdir(parents=True, exist_ok=True)
    seen = {}
    for inst in robot.instances:
        v = inst.variant
        if v.number not in seen:
            v.shape().exportStep(str(out_dir / f"{v.number}.step"))
            seen[v.number] = v
    return seen
