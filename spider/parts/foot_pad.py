"""Foot pad: cast PU-85A over-moulded boot.  Contact radius 30 mm.
Frame: tibia frame; sphere centre at x = TIBIA_L."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-2205", "Foot pad PU 85A", material="PU-85A", category="cast")
def make():
    L, R = C.TIBIA_L, C.FOOT_R
    ball = cq.Workplane("XY").sphere(R).translate((L, 0, 0))
    ball = ball.cut(cq.Workplane("YZ").workplane(offset=L - R).rect(80, 80).extrude(R * 0.55))
    boot = cq.Workplane("YZ").workplane(offset=L - 36).circle(21).circle(18).extrude(R * 0.55 + 6)
    core = cq.Workplane("XY").sphere(19.2).translate((L, 0, 0))
    stem = cq.Workplane("YZ").workplane(offset=L - 32).circle(13).extrude(14)
    grip = (cq.Workplane("YZ").workplane(offset=L + R - 3).rarray(6, 6, 5, 5).rect(2, 2).extrude(4))
    return ball.union(boot).cut(core).cut(stem).cut(grip)


if __name__ == "__main__":
    main(make)
