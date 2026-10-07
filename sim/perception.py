"""LiDAR -> 2.5D elevation map -> foothold scoring.

The 128-channel LiDAR on the pan/tilt head is simulated with MuJoCo ray
casting (``mj_multiRay``) against geom group 0 (terrain only - the robot's
own geoms are in other groups), with Gaussian range noise.  Hits are binned
into a world-frame elevation grid.  The foothold planner scores candidate
cells around the nominal foothold by local roughness (max-min height under
the foot's footprint), distance from nominal and map confidence.
"""
from __future__ import annotations

import math

import mujoco
import numpy as np

from spider import config as C


class Lidar:
    def __init__(self, m: mujoco.MjModel, rings=28, az_step_deg=2.0, el_range=(-62.0, 8.0),
                 max_range=6.0, noise=0.008, seed=0):
        self.site = m.site("lidar").id
        el = np.radians(np.linspace(*el_range, rings))
        az = np.radians(np.arange(0.0, 360.0, az_step_deg))
        E, A = np.meshgrid(el, az, indexing="ij")
        self.dirs = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
        self.n = len(self.dirs)
        self.max_range = max_range
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self.group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.geomid = np.zeros(self.n, dtype=np.int32)
        self.dist = np.zeros(self.n)

    def scan(self, m, d) -> np.ndarray:
        """World-frame hit points (N,3)."""
        p = d.site_xpos[self.site].copy()
        R = d.site_xmat[self.site].reshape(3, 3)
        vec = (self.dirs @ R.T).ravel()
        mujoco.mj_multiRay(m, d, p, vec, self.group, 1, -1, self.geomid, self.dist, None,
                           self.n, self.max_range)
        ok = (self.geomid >= 0) & (self.dist > 0.05)
        r = self.dist[ok] + self.rng.normal(0, self.noise, ok.sum())
        return p + (vec.reshape(-1, 3)[ok]) * r[:, None]


class HeightMap:
    def __init__(self, x0=-2.0, x1=14.0, y0=-3.0, y1=3.0, res=0.02):
        self.x0, self.y0, self.res = x0, y0, res
        self.nx, self.ny = int((x1 - x0) / res), int((y1 - y0) / res)
        self.h = np.full((self.nx, self.ny), np.nan)
        self.cells_seen = 0

    def idx(self, xy):
        xy = np.atleast_2d(xy)
        i = np.floor((xy[:, 0] - self.x0) / self.res).astype(int)
        j = np.floor((xy[:, 1] - self.y0) / self.res).astype(int)
        return np.clip(i, 0, self.nx - 1), np.clip(j, 0, self.ny - 1)

    def update(self, pts: np.ndarray):
        if len(pts) == 0:
            return
        i, j = self.idx(pts[:, :2])
        fresh = np.full_like(self.h, np.nan)
        np.fmax.at(fresh, (i, j), pts[:, 2])
        hit = ~np.isnan(fresh)
        # latest scan wins (handles noise drift), max within a scan (top of rock)
        self.h[hit] = np.where(np.isnan(self.h[hit]), fresh[hit], 0.5 * self.h[hit] + 0.5 * fresh[hit])
        self.cells_seen = int((~np.isnan(self.h)).sum())

    def window(self, xy, r):
        i, j = self.idx(np.asarray(xy)[None])
        k = int(math.ceil(r / self.res))
        i0, i1 = max(i[0] - k, 0), min(i[0] + k + 1, self.nx)
        j0, j1 = max(j[0] - k, 0), min(j[0] + k + 1, self.ny)
        return self.h[i0:i1, j0:j1], i0, j0

    def max_along(self, a, b, width=0.04, fallback=0.0):
        """Highest terrain point in a corridor between xy points a and b."""
        n = max(int(np.linalg.norm(np.subtract(b, a)) / self.res) + 1, 2)
        best = -np.inf
        for t in np.linspace(0, 1, n):
            w, _, _ = self.window(np.add(a, np.subtract(b, a) * t), width)
            if np.isfinite(w).any():
                best = max(best, np.nanmax(w))
        return best if np.isfinite(best) else fallback

    def best_foothold(self, xy, ground, radius=0.08):
        """Pick a foothold near xy.  Returns (xy, surface z, score info)."""
        foot = C.FOOT_R_M
        k = int(math.ceil(foot / self.res))        # footprint half-width in cells
        win, i0, j0 = self.window(xy, radius + foot)
        if win.size == 0:
            return np.asarray(xy), ground, dict(known=0.0)
        known = ~np.isnan(win)
        filled = np.where(known, win, ground)
        # footprint max / min via sliding windows
        pad = np.pad(filled, k, mode="edge")
        sw = np.lib.stride_tricks.sliding_window_view(pad, (2 * k + 1, 2 * k + 1))
        top, low = sw.max(axis=(-1, -2)), sw.min(axis=(-1, -2))
        kn = np.lib.stride_tricks.sliding_window_view(np.pad(known, k), (2 * k + 1, 2 * k + 1)).mean(axis=(-1, -2))
        xs = self.x0 + (np.arange(win.shape[0]) + i0 + 0.5) * self.res
        ys = self.y0 + (np.arange(win.shape[1]) + j0 + 0.5) * self.res
        X, Y = np.meshgrid(xs, ys, indexing="ij")
        dist = np.hypot(X - xy[0], Y - xy[1])
        rough = top - low
        cost = (rough / 0.015) ** 2 + 2.0 * (dist / radius) ** 2 + 1.5 * (1 - kn)
        cost[dist > radius] = np.inf
        a = np.unravel_index(np.argmin(cost), cost.shape)
        return np.array([X[a], Y[a]]), float(top[a]), dict(known=float(kn[a]), rough=float(rough[a]))
