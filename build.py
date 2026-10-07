#!/usr/bin/env python3
"""Build the robot from its part programs.

    python build.py            # everything
    python build.py --no-step  # skip the (slow) STEP assembly

Outputs (build/):
    step/autonomous_spider.step   full assembly, 986 named + coloured parts, standing pose
    step/parts/*.step             one file per unique part
    model/spider.urdf             URDF (ROS)
    model/spider.xml              MJCF (MuJoCo physics model)
    model/meshes/*.stl            per-link visual meshes
    bom.csv                       bill of materials
    summary.json                  counts, masses, checks
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np

from spider import config as C
from spider.export.bom import write_bom
from spider.export.mesh import export_link_meshes
from spider.export.mjcf import write_mjcf
from spider.export.step import export_part_steps, export_step
from spider.export.urdf import write_urdf
from spider.robot import build, standing_pose

OUT = Path("build")


def validate(model_dir: Path, cad_mass: float, n_joints: int):
    import mujoco

    m = mujoco.MjModel.from_xml_path(str(model_dir / "spider.xml"))
    mj_mass = float(m.body_subtreemass[1])
    # MuJoCo's URDF importer fuses the (fixed-base) base_link into the world
    # unless told not to; load a copy with fusestatic off to count every link.
    urdf = (model_dir / "spider.urdf").read_text().replace(
        '<robot name="autonomous_spider">',
        '<robot name="autonomous_spider"><mujoco><compiler fusestatic="false" '
        f'meshdir="{model_dir.resolve()}" discardvisual="false"/></mujoco>', 1)
    tmp = model_dir / "_urdf_check.urdf"
    tmp.write_text(urdf)
    try:
        u = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()
    urdf_mass = float(u.body_mass.sum())
    checks = {
        "mjcf_loads": True,
        "mjcf_mass_kg": round(mj_mass, 4),
        "mjcf_actuators": int(m.nu),
        "mjcf_hinge_joints": int((m.jnt_type == mujoco.mjtJoint.mjJNT_HINGE).sum()),
        "urdf_loads_in_mujoco": True,
        "urdf_mass_kg": round(urdf_mass, 4),
        "urdf_bodies": int(u.nbody - 1),
    }
    assert abs(mj_mass - cad_mass) < 1e-3, (mj_mass, cad_mass)
    assert abs(urdf_mass - cad_mass) < 1e-3, (urdf_mass, cad_mass)
    assert m.nu == 26 and checks["mjcf_hinge_joints"] == 26
    return checks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-step", action="store_true")
    args = ap.parse_args()
    t0 = time.time()
    robot = build()
    print(f"robot: {len(robot.instances)} parts, {len(robot.links)} links, {len(robot.joints)} joints "
          f"({time.time() - t0:.1f}s)")

    model = OUT / "model"
    names = export_link_meshes(robot, model / "meshes")
    inert = write_urdf(robot, names, model / "spider.urdf")
    write_mjcf(robot, names, model / "spider.xml")
    rows = write_bom(robot, OUT / "bom.csv")
    cad_mass = sum(i.mass for i in inert.values())
    print(f"model: URDF + MJCF written, mass {cad_mass:.3f} kg ({time.time() - t0:.1f}s)")

    if not args.no_step:
        n_unique = len(export_part_steps(robot, OUT / "step" / "parts"))
        export_step(robot, standing_pose(), OUT / "step" / "autonomous_spider.step")
        print(f"step: assembly + {n_unique} part files ({time.time() - t0:.1f}s)")

    checks = validate(model, cad_mass, len(robot.joints))
    drivers = [i for i in robot.instances if i.variant.pdef.module.endswith("motor_driver")]
    driven = [j for j in robot.joints if j.type == "revolute"]
    assert {j.driver for j in driven} == {i.name for i in drivers}, "every driver powers exactly one joint"
    cats = {}
    for i in robot.instances:
        cats[i.variant.pdef.category] = cats.get(i.variant.pdef.category, 0) + 1
    summary = {
        "parts": len(robot.instances),
        "unique_part_numbers": len(rows),
        "part_programs": len({i.variant.pdef.module for i in robot.instances}),
        "parts_by_category": dict(sorted(cats.items(), key=lambda kv: -kv[1])),
        "links": len(robot.links),
        "joints": len(robot.joints),
        "joints_revolute": len(driven),
        "joints_fixed": len(robot.joints) - len(driven),
        "motor_drivers": len(drivers),
        "mass_kg": round(cad_mass, 3),
        "link_mass_kg": {k: round(v.mass, 4) for k, v in inert.items()},
        "checks": checks,
    }
    (OUT / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps({k: v for k, v in summary.items() if k != "link_mass_kg"}, indent=2))
    print(f"done in {time.time() - t0:.1f}s")


if __name__ == "__main__":
    main()
