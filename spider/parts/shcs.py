"""ISO 4762 socket head cap screw.  Origin: under-head bearing face, shank along -z."""
import math

import cadquery as cq

from spider.parts._base import main, part

#        dk     k    s(hex key)
HEAD = {3: (5.5, 3.0, 2.5), 4: (7.0, 4.0, 3.0), 5: (8.5, 5.0, 4.0),
        6: (10.0, 6.0, 5.0), 8: (13.0, 8.0, 6.0)}


@part("ISO4762-M{d}x{L}", "Socket head cap screw M{d}x{L} 12.9",
      material="Steel-12.9", category="fastener", vendor="ISO 4762")
def make(d=4, L=12):
    dk, k, s = HEAD[d]
    head = (cq.Workplane("XY").circle(dk / 2).extrude(k)
            .faces(">Z").workplane()
            .polygon(6, s / math.cos(math.pi / 6)).cutBlind(-0.6 * k)
            .edges(">Z").fillet(0.08 * dk))
    shank = cq.Workplane("XY").circle(d / 2).extrude(-L)
    return head.union(shank)


if __name__ == "__main__":
    main(make)
