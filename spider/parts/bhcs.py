"""ISO 7380 button head screw (stainless).  Origin: under-head face, shank along -z."""
import math

import cadquery as cq

from spider.parts._base import main, part

HEAD = {3: (5.7, 1.65, 2.0), 4: (7.6, 2.2, 2.5), 5: (9.5, 2.75, 3.0)}


@part("ISO7380-M{d}x{L}", "Button head screw M{d}x{L} A2",
      material="Stainless-A2", category="fastener", vendor="ISO 7380")
def make(d=4, L=8):
    dk, k, s = HEAD[d]
    head = (cq.Workplane("XZ").moveTo(0, 0).lineTo(dk / 2, 0)
            .threePointArc((dk * 0.36, k * 0.75), (0, k)).close()
            .revolve(360, (0, 0, 0), (0, 1, 0)))
    head = head.cut(cq.Workplane("XY").workplane(offset=k * 0.4)
                    .polygon(6, s / math.cos(math.pi / 6)).extrude(k))
    return head.union(cq.Workplane("XY").circle(d / 2).extrude(-L))


if __name__ == "__main__":
    main(make)
