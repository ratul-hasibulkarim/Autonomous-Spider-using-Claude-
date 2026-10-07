"""Femur side plate (belt box wall).  7075-T6, waterjet + CNC, 5 mm.

Two of these, 30 mm apart, enclose the 1:1 HTD-5M belt that carries the
knee motor's torque from the hip to the knee pulley.
side="hub":     bolts straight to the femur actuator rotor (6x M4 on BC40).
side="bearing": rides on a 6805 bearing around the knee drive shaft.
Frame: femur frame, plate centred on y=0 (assembly offsets to y=+-PLATE_Y).
"""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part

R_HIP, R_KNEE = 48.0, 36.0
SPACERS_XZ = [(75.0, 33.0), (75.0, -33.0), (240.0, 31.5), (240.0, -31.5)]
IDLER_XZ = (100.0, 35.8)   # roller just touching the belt back
PLATE_Y = C.FEMUR_INNER_W / 2 + C.FEMUR_PLATE_T / 2


def half_height(x):
    return R_HIP - (R_HIP - R_KNEE) * x / C.FEMUR_L


@part("SP-2102-{side}", "Femur side plate, {side} side", material="Al7075-T6",
      category="machined")
def make(side="hub"):
    t, L = C.FEMUR_PLATE_T, C.FEMUR_L
    prof = (cq.Workplane("XZ").circle(R_HIP).extrude(t / 2, both=True)
            .union(cq.Workplane("XZ").center(L, 0).circle(R_KNEE).extrude(t / 2, both=True))
            .union(cq.Workplane("XZ").polyline([(0, R_HIP), (L, R_KNEE), (L, -R_KNEE), (0, -R_HIP)])
                   .close().extrude(t / 2, both=True)))
    cut = cq.Workplane("XZ").workplane(offset=-t)
    if side == "hub":
        prof = prof.cut(cut.circle(9).extrude(2 * t))
        prof = prof.cut(cut.polarArray(C.QDD_OUT_BOLT_CIRCLE / 2, 0, 360, 6).circle(2.2).extrude(2 * t))
    else:
        prof = prof.cut(cut.circle(18.5).extrude(2 * t))
    prof = prof.cut(cut.center(L, 0).circle(18.5).extrude(2 * t))
    prof = prof.cut(cut.pushPoints(SPACERS_XZ + [IDLER_XZ]).circle(2.7).extrude(2 * t))
    # lightening slots between the spacers (the belt runs inside the box)
    for x0, x1 in ((36.0, 60.0), (92.0, 186.0), (206.0, 226.0), (256.0, 296.0)):
        h = min(60.0, 2 * (half_height(x1) - 8.0))
        w = cut.center((x0 + x1) / 2, 0)
        w = w.slot2D(x1 - x0, h) if x1 - x0 > h + 1 else w.circle(min(h, x1 - x0) / 2)
        prof = prof.cut(w.extrude(2 * t))
    return prof


if __name__ == "__main__":
    main(make)
