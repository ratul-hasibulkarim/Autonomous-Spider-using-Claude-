"""Battery end clamp: 3 mm angle with EPDM-lined face (lining not modelled).
Frame: flange on z=0, bolt line on x=0 (holes at y=+-25), face at x=+9.6..12.6."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-1020", "Battery end clamp w/ PEM M4 nuts", material="Al6061-T6", category="sheet")
def make():
    t = 3.0
    flange = cq.Workplane("XY").center(1.3, 0).box(22.6, 80, t, centered=(True, True, False))
    leg = cq.Workplane("XY").center(11.1, 0).box(t, 80, 36, centered=(True, True, False))
    holes = cq.Workplane("XY").pushPoints([(0, -25), (0, 25)]).circle(2.2).extrude(t)
    return flange.union(leg).cut(holes).edges("|X").fillet(1.0)


if __name__ == "__main__":
    main(make)
