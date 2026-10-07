"""Twist-release emergency stop, 22 mm panel mount, with yellow guard."""
import cadquery as cq

from spider.parts._base import main, part


@part("ESTOP-40", "Emergency stop 40 mm mushroom, NC x2",
      material="COTS", category="electronic", mass=0.11,
      color=(0.85, 0.08, 0.06, 1), vendor="(spec) IEC 60947-5-5")
def make():
    base = cq.Workplane("XY").rect(50, 50).extrude(6).edges("|Z").fillet(6)
    stem = cq.Workplane("XY").workplane(offset=6).circle(14).extrude(14)
    cap = cq.Workplane("XY").workplane(offset=20).circle(20).extrude(10).edges(">Z").fillet(5)
    body = cq.Workplane("XY").circle(11).extrude(-34)          # 22 mm panel cut-out
    return base.union(stem).union(cap).union(body)


if __name__ == "__main__":
    main(make)
