"""Minimal offscreen mesh renderer (matplotlib, flat Lambert shading).

    python tools/render.py build/parts/SP-1001-top.stl out.png [elev azim]
"""
from __future__ import annotations

import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402


def load_stl(path):
    data = open(path, "rb").read()
    if data[:5] == b"solid" and b"facet" in data[:300]:
        vs = [list(map(float, ln.split()[1:4])) for ln in data.decode().splitlines()
              if ln.strip().startswith("vertex")]
        return np.array(vs).reshape(-1, 3, 3)
    n = int(np.frombuffer(data[80:84], np.uint32)[0])
    rec = np.frombuffer(data[84:84 + n * 50], dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].astype(float)


def render(tris_list, out, elev=28, azim=-60, size=(10, 7), dpi=110, title=None, zoom=1.0):
    """tris_list: list of (tris (n,3,3), rgb)."""
    fig = plt.figure(figsize=size, dpi=dpi)
    ax = fig.add_subplot(111, projection="3d", computed_zorder=False)
    light = np.array([0.4, -0.5, 0.9])
    light /= np.linalg.norm(light)
    allv = np.concatenate([t.reshape(-1, 3) for t, _ in tris_list])
    polys, cols = [], []
    for tris, rgb in tris_list:
        nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
        ln = np.linalg.norm(nrm, axis=1, keepdims=True)
        ok = ln[:, 0] > 1e-12
        nrm = nrm[ok] / ln[ok]
        shade = 0.35 + 0.65 * np.abs(nrm @ light)
        polys.append(tris[ok])
        cols.append(np.clip(np.outer(shade, rgb[:3]), 0, 1))
    polys = np.concatenate(polys)
    cols = np.concatenate(cols)
    # painter's sort along view direction
    e, a = np.radians(elev), np.radians(azim)
    view = np.array([np.cos(e) * np.cos(a), np.cos(e) * np.sin(a), np.sin(e)])
    order = np.argsort(polys.mean(axis=1) @ view)
    pc = Poly3DCollection(polys[order], facecolors=cols[order], edgecolors="none", linewidths=0)
    ax.add_collection3d(pc)
    lo, hi = allv.min(0), allv.max(0)
    c, r = (lo + hi) / 2, (hi - lo).max() / 2 / zoom
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev, azim)
    ax.set_axis_off()
    if title:
        ax.set_title(title)
    fig.subplots_adjust(0, 0, 1, 1)
    fig.savefig(out, facecolor="white")
    plt.close(fig)


if __name__ == "__main__":
    args = sys.argv[1:]
    el, az = (float(args[2]), float(args[3])) if len(args) > 3 else (28, -60)
    render([(load_stl(args[0]), (0.6, 0.62, 0.66))], args[1], el, az)
