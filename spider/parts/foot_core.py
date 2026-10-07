"""Foot core: 6061 ball with stem; presses the load-cell button.
Frame: tibia frame; sphere centre at x = TIBIA_L."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-2204", "Foot core", material="Al6061-T6", category="machined")
def make():
    L = C.TIBIA_L
    ball = cq.Workplane("XY").sphere(19).translate((L, 0, 0))
    ball = ball.cut(cq.Workplane("YZ").workplane(offset=L - 25).rect(60, 60).extrude(12))
    stem = cq.Workplane("YZ").workplane(offset=L - 20).circle(10).extrude(10)
    hollow = cq.Workplane("XY").sphere(15).translate((L, 0, 0))
    return ball.union(stem).cut(hollow)


if __name__ == "__main__":
    main(make)
