"""Coxa yoke: monolithic 5-axis 7075-T6 U-yoke.

Top boss bolts to the coxa actuator rotor (6x M4 from below, counterbored);
the two ring cheeks carry the femur actuator (-y) and the knee actuator (+y)
by their back faces; a tubular tie bar closes the U.  Hollow boss passes the
leg harness.  Frame: coxa (hip) frame, mm.
"""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part

CHEEK_N = (-73.0, -67.0)   # femur-actuator cheek, y range
CHEEK_P = (80.0, 86.0)     # knee-actuator cheek, y range
TIE_XZ = (-15.0, -50.0)
WEB_Z = (-20.0, -16.0)
NOTCH_X0, NOTCH_HALF_Y = 26.0, 24.0


def _cheek(y0, y1):
    fx, fz = C.COXA_X, C.COXA_Z
    w = cq.Workplane("XZ").workplane(offset=-y1)
    ring = w.center(fx, fz).circle(44.5).circle(31).extrude(y1 - y0)
    neck = (w.polyline([(fx - 40, WEB_Z[0] - 0.01), (fx + 40, WEB_Z[0] - 0.01),
                        (fx + 30, fz + 30), (fx - 30, fz + 30)]).close().extrude(y1 - y0))
    neck = neck.cut(w.center(fx, fz).circle(36).extrude(y1 - y0))
    lug = w.polyline([(TIE_XZ[0] - 10, WEB_Z[0]), (TIE_XZ[0] + 22, WEB_Z[0]),
                      (fx - 40, fz + 10), (TIE_XZ[0] + 8, TIE_XZ[1] - 9),
                      (TIE_XZ[0] - 10, TIE_XZ[1] - 9)]).close().extrude(y1 - y0)
    body = ring.union(neck).union(lug)
    cuts = (w.workplane(offset=-1).center(fx, fz).polarArray(C.QDD_BOLT_CIRCLE / 2, 30, 360, 6)
            .circle(2.2).extrude(y1 - y0 + 2))
    return body.cut(cuts)


@part("SP-2001", "Coxa yoke, monolithic", material="Al7075-T6", category="machined")
def make():
    y0, y1 = CHEEK_N[0], CHEEK_P[1]
    boss = cq.Workplane("XY").workplane(offset=WEB_Z[0]).circle(29).circle(13).extrude(22)
    web = (cq.Workplane("XY").workplane(offset=WEB_Z[0]).center(35, (y0 + y1) / 2)
           .rect(130, y1 - y0).extrude(WEB_Z[1] - WEB_Z[0]).edges("|Z").fillet(14))
    body = boss.union(web).union(_cheek(*CHEEK_N)).union(_cheek(*CHEEK_P))
    tie = (cq.Workplane("XZ").workplane(offset=-CHEEK_P[0]).center(*TIE_XZ).circle(8).circle(5)
           .extrude(CHEEK_P[0] - CHEEK_N[1]))
    body = body.union(tie)
    rotor = (cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1)
             .polarArray(C.QDD_OUT_BOLT_CIRCLE / 2, 0, 360, 6).circle(2.2).extrude(25))
    cb = (cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1)
          .polarArray(C.QDD_OUT_BOLT_CIRCLE / 2, 0, 360, 6).circle(4.0).extrude(7))
    scallops = (cq.Workplane("XY").workplane(offset=WEB_Z[1])
                .polarArray(27, 30, 360, 6).circle(4.5).extrude(10))
    # femur-box sweep notch: the femur pitches up to 55 deg between the arms
    notch = (cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1).center(NOTCH_X0 + 50, 0)
             .rect(100, 2 * NOTCH_HALF_Y).extrude(8).edges("|Z").fillet(6))
    # web pockets: >= 8 mm from every edge, the boss, the notch and the cheek roots
    pockets = notch.union(cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1)
                          .pushPoints([(62, 55)]).rect(40, 26).extrude(8).edges("|Z").fillet(8))
    pockets = pockets.union(cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1)
                            .pushPoints([(62, -48)]).rect(40, 20).extrude(8).edges("|Z").fillet(6))
    pockets = pockets.union(cq.Workplane("XY").workplane(offset=WEB_Z[0] - 1)
                            .pushPoints([(-8, 56), (-8, -46)]).rect(28, 28).extrude(8)
                            .edges("|Z").fillet(6))
    return body.cut(rotor).cut(cb).cut(scallops).cut(pockets)


if __name__ == "__main__":
    main(make)
