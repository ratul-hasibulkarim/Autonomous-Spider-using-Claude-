"""Snap-in harness clip for 14 mm bundle, M3 screw tab.  PA12-MJF."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-3001", "Harness clip 14 mm", material="PA12-MJF", category="printed")
def make():
    ring = cq.Workplane("XZ").center(0, 10).circle(9).circle(7).extrude(5, both=True)
    gap = cq.Workplane("XZ").center(0, 19).rect(8, 6).extrude(6, both=True)
    tab = (cq.Workplane("XY").rect(26, 10).extrude(2.5)
           .faces(">Z").workplane().pushPoints([(-9, 0)]).hole(3.3))
    return ring.cut(gap).union(tab)


if __name__ == "__main__":
    main(make)
