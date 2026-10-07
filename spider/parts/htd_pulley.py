"""HTD-5M 30-tooth timing pulley, 15 mm belt, flanged, bore {bore}.
Hub drilled for 4x M3 face bolts on a 34 mm circle.  Frame: axis y, centred."""
import math

import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("HTD5M-30T-15-B{bore}", "Timing pulley HTD-5M 30T bore {bore}",
      material="Al6061-T6", category="machined", vendor="in-house (from stock)")
def make(bore=25):
    od = C.PULLEY_PD - 1.14
    body = cq.Workplane("XZ").circle(od / 2).circle(bore / 2).extrude(C.BELT_W / 2 + 0.5, both=True)
    teeth = (cq.Workplane("XZ").polarArray(od / 2, 0, 360, C.PULLEY_TEETH)
             .rect(2.1, 4.2).extrude(C.BELT_W / 2 + 0.6, both=True))
    flanges = (cq.Workplane("XZ").workplane(offset=-10).circle(od / 2 + 3.5).circle(bore / 2).extrude(1.5)
               .union(cq.Workplane("XZ").workplane(offset=8.5).circle(od / 2 + 3.5).circle(bore / 2).extrude(1.5)))
    bolts = (cq.Workplane("XZ").workplane(offset=-11).polarArray(17, 45, 360, 4).circle(1.7)
             .extrude(22))
    light = (cq.Workplane("XZ").workplane(offset=-11).polarArray(18.5, 0, 360, 4).circle(3.6)
             .extrude(22))
    return body.cut(teeth).union(flanges).cut(bolts).cut(light)


if __name__ == "__main__":
    main(make)
