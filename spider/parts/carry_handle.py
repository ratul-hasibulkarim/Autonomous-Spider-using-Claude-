"""Carry handle (two-person lift, 55 kg robot).  PA12-GF, MJF printed.
Frame: feet on z=0, span along x, centred."""
import cadquery as cq

from spider import layout as LY
from spider.parts._base import main, part


@part("SP-1040", "Carry handle", material="PA12-MJF", category="printed")
def make():
    s = LY.HANDLE_SPAN
    feet = cq.Workplane("XY").pushPoints([(-s / 2, 0), (s / 2, 0)]).rect(34, 22).extrude(8)
    posts = cq.Workplane("XY").pushPoints([(-s / 2, 0), (s / 2, 0)]).rect(20, 16).extrude(40)
    bar = (cq.Workplane("YZ").workplane(offset=-s / 2 - 10).center(0, 40).ellipse(10, 8)
           .extrude(s + 20))
    grip = posts.union(bar).edges("|Y").fillet(6)
    holes = cq.Workplane("XY").pushPoints([(-s / 2, 0), (s / 2, 0)]).circle(3.3).extrude(8)
    return feet.union(grip).cut(holes)


if __name__ == "__main__":
    main(make)
