"""Motor-driver card cage, PA12-GF (MJF).  Boards slide into card guides and
are retained by a snap latch; two M3 screws hold the cage down.  One cage per
leg (3 slots) inside the chassis, plus one 2-slot cage for the gimbal drivers.
Frame: base on z=0, centred; board k sits at z = 4 + 14 k."""
import cadquery as cq

from spider.parts._base import main, part

PITCH, BASE = 14.0, 4.0


@part("SP-3010-{slots}", "Driver card cage, {slots}-slot", material="PA12-MJF", category="printed")
def make(slots=3):
    H = BASE + slots * PITCH
    outer = cq.Workplane("XY").box(76, 80, H, centered=(True, True, False))
    inner = cq.Workplane("XY").workplane(offset=BASE).box(66, 84, H, centered=(True, True, False))
    cage = outer.cut(inner)
    for k in range(slots):
        z = BASE + k * PITCH
        rails = (cq.Workplane("XY").workplane(offset=z - 1.5).pushPoints([(-33.5, 0), (33.5, 0)])
                 .rect(4, 80).extrude(1.5))
        cage = cage.union(rails)
    ears = (cq.Workplane("XY").pushPoints([(-30, 0), (30, 0)]).circle(1.7).extrude(BASE))
    vents = (cq.Workplane("YZ").workplane(offset=-40).pushPoints([(y, H / 2 + 1) for y in (-22, 0, 22)])
             .slot2D(H - 12, 10, 90).extrude(80))
    latch = cq.Workplane("XY").workplane(offset=H - 2).center(0, -38).rect(30, 4).extrude(6)
    return cage.cut(ears).cut(vents).union(latch)


if __name__ == "__main__":
    main(make)
