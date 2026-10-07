"""Compression load cell, 500 N, 25 mm button type - foot contact sensing.
Frame: base at z=0, axis +z."""
import cadquery as cq

from spider.parts._base import main, part


@part("LC-500N-25", "Button load cell 500 N, foot contact",
      material="COTS", category="electronic", mass=0.028,
      color=(0.70, 0.70, 0.72, 1), vendor="(spec) load cell")
def make():
    body = cq.Workplane("XY").circle(12.5).extrude(10)
    button = cq.Workplane("XY").workplane(offset=10).circle(4).extrude(2).edges(">Z").fillet(1.5)
    exit_ = cq.Workplane("XY").center(13.5, 0).box(5, 4, 4).translate((0, 0, 4))
    return body.union(button).union(exit_)


if __name__ == "__main__":
    main(make)
