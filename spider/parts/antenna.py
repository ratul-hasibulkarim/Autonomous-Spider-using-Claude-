"""Dual-band 2.4/5 GHz articulated antenna, RP-SMA (mesh radio link)."""
import cadquery as cq

from spider.parts._base import main, part


@part("ANT-DB-195", "Dual-band antenna 195 mm",
      material="COTS", category="electronic", mass=0.028, vendor="(spec) antenna")
def make():
    nut = cq.Workplane("XY").polygon(6, 9.2).extrude(6)
    whip = cq.Workplane("XY").workplane(offset=6).circle(6).extrude(190).edges(">Z").fillet(5)
    return nut.union(whip)


if __name__ == "__main__":
    main(make)
