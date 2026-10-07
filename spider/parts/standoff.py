"""Hex standoff, female-female (aluminium).  Base at origin, along +z."""
import math

import cadquery as cq

from spider.parts._base import main, part


@part("SO-M{d}-{L}", "Hex standoff M{d} x {L} mm F-F",
      material="Al6061-T6", category="fastener", vendor="Keystone")
def make(d=5, L=50.0, af=8.0):
    return (cq.Workplane("XY").polygon(6, af / math.cos(math.pi / 6)).extrude(L)
            .faces(">Z").workplane().hole(d * 0.85, L * 0.35)
            .faces("<Z").workplane().hole(d * 0.85, L * 0.35))


if __name__ == "__main__":
    main(make)
