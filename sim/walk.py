#!/usr/bin/env python3
"""Walk the spider across rock fields and report whether it ever fell.

    python -m sim.walk                         # 10 seeds x 'rocky' + 'boulders'
    python -m sim.walk --seeds 3 --difficulty boulders --video build/sim/walk.mp4

A trial is a FALL if the body rolls or pitches past 35 deg, or the chassis
comes within 12 cm of the ground under it, at any instant.  The walk ends
when the body has crossed the whole 10 m field (success) or time runs out.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

import mujoco
import numpy as np

from sim.controller import GaitParams, SpiderController
from sim.terrain import Field, make_model
from spider import config as C
from spider.kinematics import leg_ik

FIELD = Field()
FALL_TILT = math.radians(35)
FALL_CLEARANCE = 0.12


def stand(m, d, gait: GaitParams):
    """Place the robot standing just above the start area (x=0, flat)."""
    mujoco.mj_resetData(m, d)
    h = gait.body_height + C.FOOT_R_M + 0.01
    d.qpos[:3] = [0.0, 0.0, h]
    d.qpos[3:7] = [1, 0, 0, 0]
    q = leg_ik(np.array([gait.stand_radius, 0.0, -gait.body_height]))
    for leg in C.LEG_NAMES:
        for j, a in zip(("coxa", "femur", "knee"), q):
            d.qpos[m.joint(f"{leg}_{j}_joint").qposadr[0]] = a
    mujoco.mj_forward(m, d)


def ground_below(m, d, xy, top=2.0):
    gid = np.zeros(1, np.int32)
    dist = mujoco.mj_ray(m, d, np.array([xy[0], xy[1], top]), np.array([0, 0, -1.0]),
                         np.array([1, 0, 0, 0, 0, 0], np.uint8), 1, -1, gid)
    return top - dist if dist >= 0 else 0.0


def chassis_low_point(m, d, base_id):
    """Lowest point of the chassis collision boxes (world z)."""
    zs = []
    for g in range(m.ngeom):
        if m.geom_bodyid[g] == base_id and m.geom_contype[g] and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            R = d.geom_xmat[g].reshape(3, 3)
            ext = np.abs(R) @ m.geom_size[g]
            zs.append(d.geom_xpos[g][2] - ext[2])
    return min(zs)


def run_trial(seed=0, difficulty="rocky", t_max=110.0, video=None, perception=True, gait=None, verbose=True):
    gait = gait or GaitParams()
    m = make_model(seed, difficulty, FIELD, flat=(difficulty == "flat"))
    d = mujoco.MjData(m)
    stand(m, d, gait)
    ctl = SpiderController(m, gait, seed=seed, use_perception=perception)
    base = m.body("base_link").id
    robot_bodies = set(range(1, m.nbody))
    leg_of = {}
    for b in range(m.nbody):
        n = m.body(b).name
        if n[:2] in C.LEG_NAMES:
            leg_of[b] = n[:2]
    renderer = frames = None
    if video:
        renderer = mujoco.Renderer(m, 540, 960)
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.distance, cam.elevation, cam.azimuth = 3.2, -22, 125
        frames = []
    stats = dict(seed=seed, difficulty=difficulty, fell=False, fall_reason=None, max_roll_deg=0.0,
                 max_pitch_deg=0.0, min_chassis_clearance_m=9.0, chassis_terrain_contacts=0,
                 leg_leg_contacts=0, crossed=False)
    t, n_steps = 0.0, 0
    settle = 1.5
    t0 = time.time()
    end_x = FIELD.x0 + FIELD.length + 0.7
    while t < t_max:
        if not ctl.walking and t >= settle:
            ctl.walking, ctl.t_walk0 = True, t
        ctl(d, t)
        mujoco.mj_step(m, d)
        t += m.opt.timestep
        n_steps += 1
        if renderer is not None and n_steps % 17 == 0:     # ~29.4 fps of sim time
            pb = d.xpos[base]
            cam.lookat[:] = [pb[0], pb[1], 0.22]
            cam.azimuth = 125 + 50 * math.sin(t / 25)
            renderer.update_scene(d, cam)
            frames.append(renderer.render().copy())
        if n_steps % 10:
            continue
        R = d.xmat[base].reshape(3, 3)
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.asin(-np.clip(R[2, 0], -1, 1))
        stats["max_roll_deg"] = max(stats["max_roll_deg"], abs(math.degrees(roll)))
        stats["max_pitch_deg"] = max(stats["max_pitch_deg"], abs(math.degrees(pitch)))
        p = d.xpos[base]
        clearance = chassis_low_point(m, d, base) - ground_below(m, d, p[:2])
        stats["min_chassis_clearance_m"] = min(stats["min_chassis_clearance_m"], clearance)
        for c in d.contact[:d.ncon]:
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if base in (b1, b2) and (b1 == 0 or b2 == 0):
                stats["chassis_terrain_contacts"] += 1
            if b1 in leg_of and b2 in leg_of and leg_of[b1] != leg_of[b2]:
                stats["leg_leg_contacts"] += 1
        if max(abs(roll), abs(pitch)) > FALL_TILT or clearance < FALL_CLEARANCE:
            stats["fell"] = True
            stats["fall_reason"] = "tilt" if max(abs(roll), abs(pitch)) > FALL_TILT else "belly"
            break
        if p[0] > end_x:
            stats["crossed"] = True
            break
    p = d.xpos[base]
    walk_time = max(t - settle, 1e-6)
    stats.update(
        sim_time_s=round(t, 2), distance_m=round(float(p[0]), 3), lateral_drift_m=round(float(p[1]), 3),
        avg_speed_mps=round(float(p[0]) / walk_time, 3), wall_time_s=round(time.time() - t0, 1),
        rocks=int(sum(1 for g in range(m.ngeom) if m.geom(g).name.startswith("rock"))),
        max_motor_torque_nm=round(ctl.log["max_tau"], 1), touchdowns=ctl.log["touchdowns"],
        stumble_reflexes=ctl.log["stumbles"], foothold_searches=ctl.log["searches"],
        mech_energy_kj=round(ctl.log["energy"] / 1000, 2),
        cost_of_transport=round(ctl.log["energy"] / (ctl.mass * 9.81 * max(float(p[0]), 1e-3)), 2),
        map_cells=ctl.map.cells_seen)
    for k in ("max_roll_deg", "max_pitch_deg", "min_chassis_clearance_m"):
        stats[k] = round(stats[k], 3)
    if verbose:
        print(json.dumps(stats))
    if frames:
        _write_video(frames, video)
    return stats


def _write_video(frames, path):
    import subprocess
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    h, w, _ = frames[0].shape
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24",
                          "-s", f"{w}x{h}", "-r", "30", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                          "-crf", "26", str(path)], stdin=subprocess.PIPE)
    for f in frames:
        p.stdin.write(f.tobytes())
    p.stdin.close()
    p.wait()


def _job(a):
    seed, diff, perception = a
    return run_trial(seed, diff, perception=perception)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--difficulty", nargs="+", default=["rocky", "boulders"])
    ap.add_argument("--video", default=None)
    ap.add_argument("--no-perception", action="store_true")
    ap.add_argument("--out", default="build/sim/report.json")
    ap.add_argument("--jobs", type=int, default=os.cpu_count())
    args = ap.parse_args()
    jobs = [(s, diff, not args.no_perception) for diff in args.difficulty for s in range(args.seeds)]
    with ProcessPoolExecutor(max_workers=args.jobs) as ex:
        results = list(ex.map(_job, jobs))
    if args.video:
        run_trial(0, args.difficulty[-1], video=args.video, perception=not args.no_perception,
                  verbose=False)
    falls = sum(r["fell"] for r in results)
    crossed = sum(r["crossed"] for r in results)
    summary = dict(trials=len(results), falls=falls, crossed=crossed,
                   total_distance_m=round(sum(r["distance_m"] for r in results), 1),
                   worst_tilt_deg=max(max(r["max_roll_deg"], r["max_pitch_deg"]) for r in results),
                   min_chassis_clearance_m=min(r["min_chassis_clearance_m"] for r in results),
                   results=results)
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps(summary, indent=2) + "\n")
    print(f"\n{len(results)} trials, {crossed} crossed the field, {falls} falls -> {args.out}")


if __name__ == "__main__":
    main()
