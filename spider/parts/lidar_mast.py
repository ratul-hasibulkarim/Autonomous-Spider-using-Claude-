"""Sensor mast: base flange + 40x2 tube + motor seat, welded 6061, T6 after weld.
Frame: base on z=0; top (pan-motor seat) at z = MAST_H - GIMBAL_T."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-1060", "Sensor mast weldment", material="Al6061-T6", category="machined")
def make():
    top = C.MAST_H - C.GIMBAL_T
    base = (cq.Workplane("XY").rect(62, 62).extrude(6).edges("|Z").fillet(6)
            .faces(">Z").workplane().rect(46, 46, forConstruction=True).vertices().hole(5.5))
    tube = cq.Workplane("XY").workplane(offset=6).circle(20).circle(18).extrude(top - 12)
    seat = (cq.Workplane("XY").workplane(offset=top - 6).circle(C.GIMBAL_D / 2).circle(8).extrude(6)
            .faces(">Z").workplane().polarArray(C.GIMBAL_D / 2 - 7, 45, 360, 4).hole(2.8))
    gussets = (cq.Workplane("XY").workplane(offset=6).polarArray(22, 0, 360, 4)
               .rect(10, 3).extrude(25))
    cable = cq.Workplane("XZ").workplane(offset=-30).center(0, 40).slot2D(30, 12, 90).extrude(60)
    return base.union(tube).union(seat).union(gussets).cut(cable)


if __name__ == "__main__":
    main(make)
