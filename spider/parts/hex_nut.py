"""ISO 4032 hex nut / DIN 985 nylon-insert lock nut.  Origin: bearing face, body along +z."""
import math

import cadquery as cq

from spider.parts._base import main, part

#      s    m(ISO4032)  h(DIN985)
DIMS = {3: (5.5, 2.4, 4.0), 4: (7.0, 3.2, 5.0), 5: (8.0, 4.7, 5.0), 6: (10.0, 5.2, 6.0)}


@part("DIN985-M{d}", "Nylon-insert lock nut M{d} A2",
      material="Stainless-A2", category="fastener", vendor="DIN 985")
def make(d=5):
    s, _, h = DIMS[d]
    nut = (cq.Workplane("XY").polygon(6, s / math.cos(math.pi / 6)).extrude(h)
           .faces(">Z").workplane().hole(d))
    return nut.edges("not |Z").chamfer(0.3)


if __name__ == "__main__":
    main(make)
