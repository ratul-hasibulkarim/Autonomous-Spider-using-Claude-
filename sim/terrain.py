"""Rock-field scenes for the generated physics model.

The robot model (build/model/spider.xml) is loaded with MjSpec and a random
field of half-buried rocks is added to the world: ellipsoid boulders, tilted
slabs and angular blocks.  Rocks are in geom group 0 (the only group the
simulated LiDAR can see); the robot's collision geoms live in group 3.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

MODEL = Path(__file__).resolve().parents[1] / "build" / "model" / "spider.xml"

DIFFICULTY = {
    #           max height (m), coverage, max slab tilt (deg)
    "easy":     (0.06, 0.22, 10.0),
    "rocky":    (0.12, 0.34, 20.0),
    "boulders": (0.16, 0.38, 25.0),
}


@dataclass
class Field:
    x0: float = 1.0
    length: float = 10.0
    half_width: float = 1.6


def _quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def rocks(seed: int, difficulty: str = "rocky", field: Field = Field()):
    """Return a list of rock dicts (type, size, pos, quat)."""
    hmax, coverage, tilt = DIFFICULTY[difficulty]
    rng = np.random.default_rng(seed)
    area = field.length * 2 * field.half_width
    out, covered = [], 0.0
    while covered < coverage * area:
        kind = rng.choice(["ellipsoid", "slab", "block"], p=[0.55, 0.25, 0.20])
        x = field.x0 + rng.uniform(0, field.length)
        y = rng.uniform(-field.half_width, field.half_width)
        yaw = rng.uniform(0, 2 * math.pi)
        h = hmax * rng.uniform(0.25, 1.0) ** 0.8          # visible height
        if kind == "ellipsoid":
            a, b = rng.uniform(0.05, 0.20, size=2)
            c = h / rng.uniform(0.5, 0.8)                 # partially buried
            out.append(dict(type="ellipsoid", size=[a, b, c], pos=[x, y, h - c],
                            quat=_quat(rng.normal(0, 0.15), rng.normal(0, 0.15), yaw)))
            covered += math.pi * a * b
        elif kind == "slab":
            a, b = rng.uniform(0.08, 0.25), rng.uniform(0.06, 0.18)
            t = rng.uniform(0.02, 0.05)
            r, p = np.radians(rng.uniform(-tilt, tilt, size=2))
            out.append(dict(type="box", size=[a, b, t], pos=[x, y, h - t - 0.5 * a * abs(math.sin(p))],
                            quat=_quat(r, p, yaw)))
            covered += 4 * a * b
        else:
            s = rng.uniform(0.04, 0.10, size=3)
            r, p = rng.uniform(-0.6, 0.6, size=2)
            out.append(dict(type="box", size=list(s), pos=[x, y, h - np.linalg.norm(s) * 0.9],
                            quat=_quat(r, p, yaw)))
            covered += 4 * s[0] * s[1]
    return out


def make_model(seed: int = 0, difficulty: str = "rocky", field: Field = Field(), flat=False):
    spec = mujoco.MjSpec.from_file(str(MODEL))
    spec.option.timestep = 0.002
    spec.visual.global_.offwidth, spec.visual.global_.offheight = 1280, 720
    spec.visual.quality.shadowsize = 4096
    spec.visual.headlight.ambient = [0.45, 0.45, 0.45]
    spec.visual.headlight.diffuse = [0.5, 0.5, 0.5]
    tex = spec.add_texture(name="grid", type=mujoco.mjtTexture.mjTEXTURE_2D,
                           builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, width=512, height=512,
                           rgb1=[0.42, 0.38, 0.32], rgb2=[0.38, 0.34, 0.29])
    spec.add_texture(name="sky", type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
                     builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT, width=512, height=512,
                     rgb1=[0.62, 0.74, 0.88], rgb2=[0.95, 0.95, 0.97])
    mat = spec.add_material(name="ground")
    mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "grid"
    mat.texrepeat = [20, 20]
    del tex
    wb = spec.worldbody
    light = wb.add_light()
    light.pos, light.dir, light.diffuse = [2, -2, 6], [-0.2, 0.3, -1], [0.6, 0.6, 0.6]
    light.mode = mujoco.mjtCamLight.mjCAMLIGHT_TRACKCOM
    light.castshadow = True
    floor = wb.add_geom()
    floor.name, floor.type, floor.size = "floor", mujoco.mjtGeom.mjGEOM_PLANE, [30, 10, 0.1]
    floor.material, floor.contype, floor.conaffinity = "ground", 1, 2
    floor.friction = [1.0, 0.02, 0.002]
    if not flat:
        for k, rk in enumerate(rocks(seed, difficulty, field)):
            g = wb.add_geom()
            g.name = f"rock{k}"
            g.type = mujoco.mjtGeom.mjGEOM_ELLIPSOID if rk["type"] == "ellipsoid" else mujoco.mjtGeom.mjGEOM_BOX
            g.size, g.pos, g.quat = rk["size"], rk["pos"], rk["quat"]
            shade = 0.45 + 0.25 * ((k * 7919) % 100) / 100
            g.rgba = [shade, shade * 0.95, shade * 0.88, 1]
            g.contype, g.conaffinity = 1, 2
            g.friction = [0.9, 0.02, 0.002]
    return spec.compile()
