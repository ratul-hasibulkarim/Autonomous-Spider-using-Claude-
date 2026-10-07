"""DIN 471 external retaining ring for a shaft of diameter d."""
import cadquery as cq

from spider.parts._base import main, part


@part("DIN471-{d}", "Retaining ring DIN 471 for {d} mm shaft",
      material="Spring-steel", category="fastener", vendor="DIN 471")
def make(d=25):
    ring = cq.Workplane("XY").circle(d / 2 + 3.2).circle(d / 2 - 1.0).extrude(1.2)
    gap = cq.Workplane("XY").center(0, d / 2 + 1).rect(d * 0.3, 8).extrude(1.2)
    lugs = (cq.Workplane("XY").pushPoints([(-d * 0.2, d / 2 + 2.5), (d * 0.2, d / 2 + 2.5)])
            .circle(2.6).circle(0.9).extrude(1.2))
    return ring.cut(gap).union(lugs)


if __name__ == "__main__":
    main(make)
