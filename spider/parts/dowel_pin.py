"""ISO 8734 hardened dowel pin.  Centred on origin along z."""
import cadquery as cq

from spider.parts._base import main, part


@part("ISO8734-{d}x{L}", "Dowel pin {d}x{L} m6",
      material="Steel-52100", category="fastener", vendor="ISO 8734")
def make(d=4, L=12):
    return (cq.Workplane("XY").circle(d / 2).extrude(L).translate((0, 0, -L / 2))
            .edges().chamfer(0.3))


if __name__ == "__main__":
    main(make)
