"""Front/rear bulkhead: 2 mm 5052 C-channel closing the plate sandwich.
Frame: web on x=+9 (outboard), flanges run inboard to x=-9; hole line at x=0."""
import cadquery as cq

from spider import config as C
from spider import layout as LY
from spider.parts._base import main, part


@part("SP-1010", "Bulkhead C-channel w/ PEM M4 nuts", material="Al6061-anodized-black", category="sheet")
def make():
    t, H, W = 2.0, C.PLATE_GAP, LY.BULKHEAD_W
    prof = (cq.Workplane("XZ").moveTo(-9, 0).lineTo(11, 0).lineTo(11, H).lineTo(-9, H)
            .lineTo(-9, H - t).lineTo(11 - t, H - t).lineTo(11 - t, t).lineTo(-9, t).close())
    ch = prof.extrude(W / 2, both=True)
    holes = cq.Workplane("XY").workplane(offset=-1).pushPoints([(0, y) for y in LY.BULKHEAD_SCREW_Y]).circle(2.2).extrude(H + 2)
    vents = cq.Workplane("YZ").workplane(offset=5).pushPoints([(y, H / 2) for y in (-35, 35)]).slot2D(40, 14).extrude(10)
    bumper_holes = cq.Workplane("YZ").workplane(offset=5).pushPoints([(y, H / 2) for y in (-80, 0, 80)]).circle(2.2).extrude(10)
    return ch.cut(holes).cut(vents).cut(bumper_holes)


if __name__ == "__main__":
    main(make)
