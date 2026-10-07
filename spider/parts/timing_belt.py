"""HTD-5M timing belt, 15 mm wide, closed loop around two equal pulleys
at centre distance CD (mm).  Frame: first pulley axis on y, at origin; second
at +x.  Belt width along y."""
import math

import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("HTD5M-{length}-15", "Timing belt HTD-5M {length} mm x 15",
      material="Rubber-belt", category="cots", vendor="Gates")
def make(CD=C.FEMUR_L, length=int(2 * C.FEMUR_L + math.pi * C.PULLEY_PD)):
    r = C.PULLEY_PD / 2
    ro, ri = r + 2.4, r - 0.55       # back of belt / tooth land (teeth not modelled)
    outer = (cq.Workplane("XZ").center(CD / 2, 0).slot2D(CD + 2 * ro, 2 * ro)
             .extrude(C.BELT_W / 2, both=True))
    inner = (cq.Workplane("XZ").center(CD / 2, 0).slot2D(CD + 2 * ri, 2 * ri)
             .extrude(C.BELT_W, both=True))
    return outer.cut(inner)


if __name__ == "__main__":
    main(make)
