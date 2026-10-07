"""Compute-bay cover: 1.0 mm 5052, folded, with louvres and cable glands.
Frame: flange underside on z=0, centred on the enclosure footprint."""
import cadquery as cq

from spider import layout as LY
from spider.parts._base import main, part


@part("SP-1030", "Compute bay cover", material="Al6061-anodized-black", category="sheet")
def make():
    w, h, H = LY.ENCLOSURE_SIZE
    t, f = 1.0, 14.0
    shell = (cq.Workplane("XY").box(w, h, H, centered=(True, True, False))
             .edges("|Z").fillet(10).faces("<Z").shell(-t))
    flange = (cq.Workplane("XY").rect(w + 2 * f, h + 2 * f).extrude(t).edges("|Z").fillet(18)
              .cut(cq.Workplane("XY").rect(w - 2, h - 2).extrude(t)))
    cx, cy = LY.ENCLOSURE_XY
    holes = [(x - cx, y - cy) for x, y in LY.enclosure_flange_holes()]
    flange = flange.cut(cq.Workplane("XY").pushPoints(holes).circle(2.2).extrude(t))
    louvres = (cq.Workplane("XY").workplane(offset=H - 2).center(60, 0)
               .rarray(14, 1, 10, 1).slot2D(110, 6, 90).extrude(4))
    side = (cq.Workplane("XZ").workplane(offset=-h / 2 - 2).center(-80, H / 2)
            .rarray(12, 1, 8, 1).slot2D(40, 5, 90).extrude(-6))
    glands = (cq.Workplane("YZ").workplane(offset=w / 2 - 2).pushPoints([(-70, 25), (0, 25), (70, 25)])
              .circle(8).extrude(6))
    return shell.union(flange).cut(louvres).cut(side).cut(glands)


if __name__ == "__main__":
    main(make)
