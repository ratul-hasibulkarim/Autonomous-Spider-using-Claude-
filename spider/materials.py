"""Engineering materials.  Density in kg/m^3, colour as RGBA 0..1."""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    name: str
    density: float
    rgba: tuple[float, float, float, float]


MATERIALS = {m.name: m for m in [
    Material("Al6061-T6", 2700, (0.72, 0.74, 0.77, 1)),
    Material("Al7075-T6", 2810, (0.62, 0.64, 0.68, 1)),
    Material("Al6061-anodized-black", 2700, (0.10, 0.10, 0.11, 1)),
    Material("Steel-12.9", 7850, (0.15, 0.15, 0.16, 1)),
    Material("Stainless-A2", 7900, (0.80, 0.80, 0.82, 1)),
    Material("Steel-52100", 7810, (0.70, 0.70, 0.72, 1)),
    Material("Spring-steel", 7850, (0.25, 0.25, 0.27, 1)),
    Material("CFRP", 1550, (0.06, 0.06, 0.07, 1)),
    Material("PA12-MJF", 1010, (0.20, 0.20, 0.22, 1)),
    Material("PETG", 1270, (0.95, 0.45, 0.08, 1)),
    Material("POM", 1410, (0.92, 0.92, 0.90, 1)),
    Material("UHMW-PE", 940, (0.95, 0.95, 0.95, 1)),
    Material("PU-85A", 1150, (0.12, 0.12, 0.12, 1)),
    Material("Nylon-66", 1140, (0.90, 0.90, 0.88, 1)),
    Material("Rubber-belt", 1250, (0.08, 0.08, 0.08, 1)),
    # COTS parts: mass comes from the datasheet, density is back-solved.
    Material("COTS", 0, (0.25, 0.25, 0.28, 1)),
    Material("PCB", 0, (0.05, 0.35, 0.15, 1)),
    Material("Cable", 0, (0.10, 0.10, 0.10, 1)),
]}
