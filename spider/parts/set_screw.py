"""ISO 4029 hex socket set screw, cup point.  Origin at socket face, along -z."""
import math

import cadquery as cq

from spider.parts._base import main, part


@part("ISO4029-M{d}x{L}", "Set screw M{d}x{L} cup point", material="Steel-12.9",
      category="fastener", vendor="ISO 4029")
def make(d=4, L=6):
    s = {3: 1.5, 4: 2.0, 5: 2.5}[d]
    body = cq.Workplane("XY").circle(d / 2).extrude(-L)
    sock = cq.Workplane("XY").polygon(6, s / math.cos(math.pi / 6)).extrude(-0.5 * L)
    return body.cut(sock)


if __name__ == "__main__":
    main(make)
