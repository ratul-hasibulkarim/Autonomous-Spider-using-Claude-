"""Knee axle, 25 h6, grooved for DIN 471 rings.  Turns with the tibia in two
6805 bearings in the femur plates.  Frame: axis y, centred."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-2104", "Knee axle", material="Al7075-T6", category="machined")
def make(L=46.0):
    shaft = cq.Workplane("XZ").circle(12.5).circle(7.0).extrude(L / 2, both=True).edges().chamfer(0.5)
    for y in (-21.3, 21.3 - 1.3):
        shaft = shaft.cut(cq.Workplane("XZ").workplane(offset=-y - 1.3).circle(12.5).circle(11.8).extrude(1.3))
    return shaft


if __name__ == "__main__":
    main(make)
