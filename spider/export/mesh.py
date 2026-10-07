"""Per-link visual meshes: every part's tessellation, placed in the link frame,
merged into one binary STL (metres).  Identical links share one file."""
from __future__ import annotations

import hashlib
import struct
from pathlib import Path

import numpy as np


def link_triangles(robot, link: str, tol=0.4):
    tris = []
    for inst in robot.instances_on(link):
        small = inst.variant.pdef.category in ("fastener", "electronic", "cable", "bearing")
        V, F = inst.variant.mesh(tol * 2, 0.9) if small else inst.variant.mesh(tol, 0.5)
        if len(F) == 0:
            continue
        Vw = V @ inst.T[:3, :3].T + inst.T[:3, 3]
        tris.append(Vw[F])
    return np.concatenate(tris) / 1000.0


def link_signature(robot, link: str) -> str:
    h = hashlib.sha1()
    for inst in sorted(robot.instances_on(link), key=lambda i: (i.variant.number, tuple(np.round(i.T, 3).ravel()))):
        h.update(inst.variant.number.encode())
        h.update(np.round(inst.T, 3).tobytes())
    return h.hexdigest()[:12]


def write_stl(path: Path, tris: np.ndarray):
    tris = tris.astype(np.float32)
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    ln = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.where(ln > 0, n / np.maximum(ln, 1e-12), 0).astype(np.float32)
    rec = np.zeros(len(tris), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    rec["n"], rec["v"] = n, tris
    with open(path, "wb") as f:
        f.write(b"spider link mesh".ljust(80, b" "))
        f.write(struct.pack("<I", len(tris)))
        f.write(rec.tobytes())


def export_link_meshes(robot, out_dir: Path, tol=0.4) -> dict[str, str]:
    """Write meshes; returns link -> file name."""
    out_dir.mkdir(parents=True, exist_ok=True)
    by_sig, names = {}, {}
    for link in robot.links:
        sig = link_signature(robot, link)
        if sig not in by_sig:
            kind = link.split("_", 1)[1] if link[:2] in {"L1", "L2", "L3", "L4", "R1", "R2", "R3", "R4"} else link
            fname = f"{kind}.stl"
            write_stl(out_dir / fname, link_triangles(robot, link, tol))
            by_sig[sig] = fname
        names[link] = by_sig[sig]
    return names
