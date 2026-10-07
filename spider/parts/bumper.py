"""Front/rear bumper fairing, PU-85A cast.  Protects lidar mast and bulkheads.
Frame: inner face against the bulkhead web at x=0, bottom on z=0."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-1050", "Bumper fairing", material="PU-85A", category="cast")
def make():
    H = C.PLATE_GAP + 2 * C.PLATE_T + 10
    outer = cq.Workplane("XY").center(-60, 0).ellipse(90, 150).extrude(H)
    inner = cq.Workplane("XY").center(-60, 0).ellipse(82, 142).extrude(H)
    keep = cq.Workplane("XY").center(30, 0).rect(60, 320).extrude(H)
    shell = outer.cut(inner).intersect(keep)
    holes = cq.Workplane("YZ").pushPoints([(y, C.PLATE_GAP / 2 + 5) for y in (-80, 0, 80)]).circle(2.2).extrude(30)
    return shell.cut(holes).translate((0, 0, -C.PLATE_T - 5))


if __name__ == "__main__":
    main(make)
