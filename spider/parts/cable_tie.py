"""Nylon cable tie 2.5 x 100 (closed loop as installed, around a bundle of diameter D)."""
import cadquery as cq

from spider.parts._base import main, part


@part("CT-2.5x100", "Cable tie 2.5 x 100 mm, PA66",
      material="Nylon-66", category="fastener", vendor="HellermannTyton")
def make(D=14.0):
    loop = cq.Workplane("XY").circle(D / 2 + 1.0).circle(D / 2).extrude(2.5)
    head = cq.Workplane("XY").center(D / 2 + 3, 0).rect(5, 4).extrude(2.5)
    return loop.union(head)


if __name__ == "__main__":
    main(make)
