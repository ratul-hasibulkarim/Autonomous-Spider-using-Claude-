"""Knee drive shaft: knee-motor rotor flange + 25 mm shaft into the hip pulley.
17-4PH would be nicer; 7075-T6 is plenty at 110 N m (tau_max = 52 MPa).
Frame: femur-axis frame, axis y; flange y in [24,32], shaft to y=-10."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-2103", "Knee drive shaft", material="Al7075-T6", category="machined")
def make():
    flange = (cq.Workplane("XZ").workplane(offset=-32).circle(27).circle(8).extrude(8)
              .faces("<Y").workplane().polarArray(C.QDD_OUT_BOLT_CIRCLE / 2, 0, 360, 6).hole(3.4))
    shaft = cq.Workplane("XZ").workplane(offset=-24).circle(12.5).circle(8.5).extrude(34)
    key = cq.Workplane("XZ").workplane(offset=-12).center(0, 12.5).rect(6, 3).extrude(22)
    return flange.union(shaft).cut(key)


if __name__ == "__main__":
    main(make)
