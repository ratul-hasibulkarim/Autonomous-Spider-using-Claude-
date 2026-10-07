"""Deep-groove ball bearing, thin section (6805-2RS by default)."""
import cadquery as cq

from spider.parts._base import main, part

SERIES = {"6805": (25, 37, 7), "6800": (10, 19, 5), "625": (5, 16, 5), "6000": (10, 26, 8)}
MASS = {"6805": 0.022, "6800": 0.0055, "625": 0.0047, "6000": 0.019}  # kg, catalogue


@part("BRG-{series}-2RS", "Ball bearing {series}-2RS",
      material="Steel-52100", category="bearing", vendor="SKF",
      mass=lambda series: MASS[series])
def make(series="6805"):
    d, D, B = SERIES[series]
    t = (D - d) / 2
    outer = cq.Workplane("XY").circle(D / 2).circle(D / 2 - t * 0.3).extrude(B)
    inner = cq.Workplane("XY").circle(d / 2 + t * 0.3).circle(d / 2).extrude(B)
    seal = (cq.Workplane("XY").workplane(offset=0.4).circle(D / 2 - t * 0.3)
            .circle(d / 2 + t * 0.3).extrude(B - 0.8))
    return outer.union(inner).union(seal).translate((0, 0, -B / 2))


if __name__ == "__main__":
    main(make)
