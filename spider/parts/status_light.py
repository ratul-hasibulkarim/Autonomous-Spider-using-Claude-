"""RGB status beacon (state machine: idle / walking / fault)."""
import cadquery as cq

from spider.parts._base import main, part


@part("LED-BEACON-40", "RGB status beacon 40 mm",
      material="COTS", category="electronic", mass=0.045,
      color=(0.2, 0.8, 0.9, 1), vendor="(spec) beacon")
def make():
    return (cq.Workplane("XY").circle(20).extrude(10)
            .union(cq.Workplane("XY").workplane(offset=10).circle(17).extrude(28).edges(">Z").fillet(10)))


if __name__ == "__main__":
    main(make)
