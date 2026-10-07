"""QDD-96 quasi-direct-drive leg actuator (bought in).

Frameless 48 V BLDC + 9:1 single-stage planetary + dual absolute encoders in
one sealed housing.  110 N m peak / 42 N m continuous, 14 rad/s.
Frame: output face at z=0 facing -z, housing extends to z=+48; connector on -x.
"""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("QDD-96-9", "QDD-96 leg actuator, 9:1, 110 N m peak",
      material="COTS", category="cots", mass=C.QDD_MASS,
      color=(0.16, 0.17, 0.19, 1), vendor="(spec) QDD-96")
def make():
    D, T = C.QDD_D, C.QDD_T
    housing = (cq.Workplane("XY").workplane(offset=2).circle(D / 2).extrude(T - 2)
               .edges(">Z").fillet(4).edges("<Z").fillet(2))
    # cooling fins on the stator ring
    fins = (cq.Workplane("XY").workplane(offset=10).polarArray(D / 2, 0, 360, 36)
            .rect(2.0, 3.0).extrude(T - 20))
    out = (cq.Workplane("XY").circle(30).circle(9).extrude(2.0)
           .faces("<Z").workplane().polarArray(C.QDD_OUT_BOLT_CIRCLE / 2, 0, 360, 6)
           .hole(3.3, 1.8))
    back_holes = (cq.Workplane("XY").workplane(offset=T)
                  .polarArray(C.QDD_BOLT_CIRCLE / 2, 30, 360, 6).circle(1.65)
                  .extrude(-7))
    connector = cq.Workplane("XY").center(-D / 2 - 4, 0).box(10, 22, 14).translate((0, 0, T - 12))
    return housing.union(fins).union(out).union(connector).cut(back_holes)


if __name__ == "__main__":
    main(make)
