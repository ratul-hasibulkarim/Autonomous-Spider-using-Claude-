"""ISO 7089 plain washer."""
import cadquery as cq

from spider.parts._base import main, part

DIMS = {3: (3.2, 7.0, 0.5), 4: (4.3, 9.0, 0.8), 5: (5.3, 10.0, 1.0), 6: (6.4, 12.0, 1.6)}


@part("ISO7089-{d}", "Plain washer {d} mm A2",
      material="Stainless-A2", category="fastener", vendor="ISO 7089")
def make(d=5):
    d1, d2, h = DIMS[d]
    return cq.Workplane("XY").circle(d2 / 2).circle(d1 / 2).extrude(h)


if __name__ == "__main__":
    main(make)
