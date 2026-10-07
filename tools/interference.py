"""Solid-solid interference check of the assembly at the standing pose.

Every pair of structural parts (fasteners and cable runs excluded: screws
sit in tap-drill holes and harness runs are schematic) whose bounding boxes
overlap is intersected with OpenCascade; any common volume above the
threshold is reported.

    python tools/interference.py [--threshold 1.0]
"""
from __future__ import annotations

import argparse
import json
import sys
from itertools import combinations
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from spider.robot import build, standing_pose  # noqa: E402
from spider.tf import to_location  # noqa: E402

SKIP = {"fastener", "cable"}
ALLOWED: set = set()   # intended overlaps (none needed: belt teeth/threads are not modelled)


def check(r, q, threshold, links_a=None, links_b=None):
    """Interferences at joint configuration q.  If links_a/links_b are given,
    only pairs with one part in each set are checked."""
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    poses = r.link_poses(q)
    items = []
    for inst in r.instances:
        if inst.variant.pdef.category in SKIP:
            continue
        if links_a is not None and inst.link not in links_a | links_b:
            continue
        T = poses[inst.link] @ inst.T
        V, _ = inst.variant.mesh(0.5, 0.5)
        W = V @ T[:3, :3].T + T[:3, 3]
        items.append((inst, T, W.min(0) - 0.05, W.max(0) + 0.05))
    cand = [(a, b) for a, b in combinations(items, 2)
            if np.all(a[2] <= b[3]) and np.all(b[2] <= a[3])
            and (links_a is None or (a[0].link in links_a) != (b[0].link in links_a))]
    hits = []
    for a, b in cand:
        if frozenset({a[0].variant.number, b[0].variant.number}) in ALLOWED:
            continue
        sa = a[0].variant.shape().moved(to_location(a[1])).wrapped
        sb = b[0].variant.shape().moved(to_location(b[1])).wrapped
        op = BRepAlgoAPI_Common(sa, sb)
        if not op.IsDone():
            continue
        p = GProp_GProps()
        BRepGProp.VolumeProperties_s(op.Shape(), p)
        if p.Mass() > threshold:
            hits.append(dict(a=a[0].name, b=b[0].name, volume_mm3=round(p.Mass(), 1)))
    hits.sort(key=lambda h: -h["volume_mm3"])
    return len(items), len(cand), hits


def sweeps(r, threshold):
    """Clearance through each leg joint's range (one leg - all eight are
    identical) and between neighbouring legs swung toward each other."""
    from spider import config as C
    q0 = standing_pose()
    base, cx, fe, ti = "base_link", "L1_coxa", "L1_femur", "L1_tibia"
    out = []
    for name, joint, lim, a, b in [
        ("femur", "L1_femur_joint", C.FEMUR_LIMIT, {fe, ti}, {base, cx}),
        ("knee", "L1_knee_joint", C.KNEE_LIMIT, {ti}, {base, cx, fe}),
    ]:
        for ang in np.linspace(lim[0], lim[1], 7):
            q = dict(q0, **{joint: float(ang)})
            _, _, hits = check(r, q, threshold, a, b)
            out.append(dict(sweep=name, angle_deg=round(float(np.degrees(ang)), 1), hits=hits))
    # neighbours: each leg turned 25 deg toward the next one on its side
    for l1, l2 in [("L1", "L2"), ("L2", "L3"), ("L3", "L4")]:
        q = dict(q0, **{f"{l1}_coxa_joint": -np.radians(25), f"{l2}_coxa_joint": np.radians(25)})
        A = {f"{l1}_{s}" for s in ("coxa", "femur", "tibia")}
        B = {f"{l2}_{s}" for s in ("coxa", "femur", "tibia")}
        _, _, hits = check(r, q, threshold, A, B)
        out.append(dict(sweep=f"{l1}-{l2} coxa +-25deg", angle_deg=25.0, hits=hits))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--threshold", type=float, default=1.0, help="mm^3")
    ap.add_argument("--sweep", action="store_true", help="also sweep joint ranges")
    ap.add_argument("--out", default="build/interference.json")
    args = ap.parse_args()
    r = build()
    n, nc, hits = check(r, standing_pose(), args.threshold)
    out = dict(parts_checked=n, bbox_pairs=nc, interferences=len(hits), pairs=hits)
    print(f"{n} parts, {nc} candidate pairs, {len(hits)} interferences > {args.threshold} mm^3")
    for h in hits[:40]:
        print(f"  {h['volume_mm3']:>10.1f}  {h['a']}  <->  {h['b']}")
    if args.sweep:
        out["sweeps"] = sweeps(r, args.threshold)
        for s in out["sweeps"]:
            worst = s["hits"][0] if s["hits"] else None
            print(f"  sweep {s['sweep']:>22} {s['angle_deg']:>7}: "
                  + (f"{len(s['hits'])} hits, worst {worst['volume_mm3']} mm^3 {worst['a']} <-> {worst['b']}"
                     if worst else "clear"))
    Path(args.out).write_text(json.dumps(out, indent=2) + "\n")


if __name__ == "__main__":
    main()
