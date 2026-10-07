"""The part-program convention.

Every part in the robot is a Python program: a module in ``spider/parts`` that
exposes one decorated ``make(**params)`` function returning CadQuery geometry.
The decorator attaches the engineering metadata (part number, material or
datasheet mass, category).  Running a module directly builds that part on its
own and writes ``build/parts/<number>.step`` + ``.stl``::

    python -m spider.parts.femur_side_plate
    python -m spider.parts.shcs d=4 L=12

Geometry is in millimetres.  Mass properties are computed from the exact
B-rep (OpenCascade) times material density; for bought-in (COTS) parts the
datasheet mass is used and the envelope geometry supplies the distribution.
"""
from __future__ import annotations

import functools
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable

import numpy as np

from spider.materials import MATERIALS

REGISTRY: dict[str, "PartDef"] = {}

CATEGORIES = ("machined", "sheet", "composite", "printed", "cast", "cots",
              "electronic", "fastener", "bearing", "cable")


@dataclass(eq=False)
class PartDef:
    number: str                 # format string, e.g. "ISO4762-M{d}x{L}"
    name: str                   # format string
    material: str
    category: str
    fn: Callable
    module: str
    mass: float | Callable | None = None   # kg, datasheet (COTS)
    color: tuple | None = None
    vendor: str = "in-house"

    def __call__(self, **params) -> "Variant":
        return Variant(self, tuple(sorted(params.items())))

    @property
    def defaults(self) -> dict:
        import inspect
        sig = inspect.signature(self.fn)
        return {k: v.default for k, v in sig.parameters.items()
                if v.default is not inspect.Parameter.empty}


@dataclass(frozen=True)
class Variant:
    """A part definition plus concrete parameters == one orderable item."""
    pdef: PartDef = field(compare=True, hash=True)
    params: tuple = ()

    @property
    def kwargs(self) -> dict:
        return {**self.pdef.defaults, **dict(self.params)}

    @property
    def number(self) -> str:
        return self.pdef.number.format(**self.kwargs)

    @property
    def name(self) -> str:
        return self.pdef.name.format(**self.kwargs)

    @property
    def rgba(self):
        return self.pdef.color or MATERIALS[self.pdef.material].rgba

    def shape(self):
        return _shape(self)

    def massprops(self):
        """(mass kg, com mm (3,), inertia about com kg*mm^2 (3,3))."""
        return _massprops(self)

    def mesh(self, tol=0.25, ang=0.35):
        return _mesh(self, tol, ang)


@functools.lru_cache(maxsize=None)
def _shape(v: Variant):
    import cadquery as cq
    out = v.pdef.fn(**dict(v.params))
    if isinstance(out, cq.Workplane):
        solids = [o for o in out.vals() if isinstance(o, cq.Shape)]
        out = solids[0] if len(solids) == 1 else cq.Compound.makeCompound(solids)
    if not out.isValid():
        raise ValueError(f"{v.number}: invalid B-rep")
    return out


@functools.lru_cache(maxsize=None)
def _massprops(v: Variant):
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps

    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(v.shape().wrapped, props)
    vol = props.Mass()  # mm^3
    if vol <= 0:
        raise ValueError(f"{v.number}: non-positive volume")
    c = props.CentreOfMass()
    m = props.MatrixOfInertia()  # about centre of mass, density 1
    inertia = np.array([[m.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)])
    pd = v.pdef
    if pd.mass is not None:
        mass = pd.mass(**v.kwargs) if callable(pd.mass) else pd.mass
    else:
        mass = MATERIALS[pd.material].density * vol * 1e-9
    rho = mass / vol  # kg / mm^3
    return mass, np.array([c.X(), c.Y(), c.Z()]), inertia * rho


@functools.lru_cache(maxsize=None)
def _mesh(v: Variant, tol: float, ang: float):
    verts, tris = v.shape().tessellate(tol, ang)
    V = np.array([(p.x, p.y, p.z) for p in verts], dtype=np.float64).reshape(-1, 3)
    F = np.array(tris, dtype=np.int32).reshape(-1, 3)
    return V, F


def part(number: str, name: str, *, material: str, category: str,
         mass=None, color=None, vendor="in-house"):
    assert category in CATEGORIES, category
    assert material in MATERIALS, material

    def deco(fn):
        pd = PartDef(number, name, material, category, fn, fn.__module__,
                     mass, color, vendor)
        REGISTRY[f"{fn.__module__}"] = pd
        return pd
    return deco


def main(pdef: PartDef, argv=None):
    """Standalone entry point shared by every part module."""
    argv = sys.argv[1:] if argv is None else argv
    params = {}
    for a in argv:
        k, val = a.split("=", 1)
        try:
            val = int(val)
        except ValueError:
            try:
                val = float(val)
            except ValueError:
                pass
        params[k] = val
    v = pdef(**params)
    mass, com, inertia = v.massprops()
    out = Path("build/parts")
    out.mkdir(parents=True, exist_ok=True)
    v.shape().exportStep(str(out / f"{v.number}.step"))
    v.shape().exportStl(str(out / f"{v.number}.stl"), 0.1, 0.2)
    print(f"{v.number}  {v.name}")
    print(f"  material   {pdef.material}  ({pdef.category}, {pdef.vendor})")
    print(f"  mass       {mass * 1000:.1f} g")
    print(f"  CoM (mm)   {np.round(com, 2)}")
    print(f"  I (kg mm2) diag {np.round(np.diag(inertia), 3)}")
    print(f"  wrote      {out / v.number}.step / .stl")
