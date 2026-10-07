"""Bill of materials, grouped by part number."""
from __future__ import annotations

import csv
from collections import OrderedDict
from pathlib import Path


def bom_rows(robot):
    rows = OrderedDict()
    for inst in robot.instances:
        v = inst.variant
        r = rows.setdefault(v.number, dict(number=v.number, name=v.name, category=v.pdef.category,
                                           material=v.pdef.material, vendor=v.pdef.vendor,
                                           program=v.pdef.module.replace(".", "/") + ".py",
                                           qty=0, unit_g=v.massprops()[0] * 1000))
        r["qty"] += 1
    out = sorted(rows.values(), key=lambda r: (r["category"], r["number"]))
    for r in out:
        r["total_g"] = r["qty"] * r["unit_g"]
    return out


def write_bom(robot, path: Path):
    rows = bom_rows(robot)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["number", "name", "qty", "unit_g", "total_g", "category",
                                          "material", "vendor", "program"])
        w.writeheader()
        for r in rows:
            w.writerow({**r, "unit_g": f"{r['unit_g']:.2f}", "total_g": f"{r['total_g']:.1f}"})
    return rows
