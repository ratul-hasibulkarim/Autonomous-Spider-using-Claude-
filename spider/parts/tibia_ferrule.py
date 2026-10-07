"""Foot-end ferrule, bonded (DP490) to the tibia tube; carries the load cell.
6061-T6.  Frame: tibia frame; x in [342, 368]."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-2203", "Tibia foot ferrule", material="Al6061-T6", category="machined")
def make():
    body = cq.Workplane("YZ").workplane(offset=342).circle(17.2).extrude(26)
    bore = cq.Workplane("YZ").workplane(offset=342).circle(16.1).extrude(14)
    holes = cq.Workplane("YZ").workplane(offset=360).pushPoints([(0, -9), (0, 9)]).circle(1.4).extrude(9)
    return body.cut(bore).cut(holes)


if __name__ == "__main__":
    main(make)
