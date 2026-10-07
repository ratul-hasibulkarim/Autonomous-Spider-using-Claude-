"""GM-62 hollow-shaft gimbal motor with magnetic encoder (sensor-head pan/tilt).
Frame: output face at z=0 facing -z, stator extends to z=+26."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("GM-62-ENC", "GM-62 gimbal motor with 14-bit encoder",
      material="COTS", category="cots", mass=C.GIMBAL_MASS,
      color=(0.12, 0.12, 0.14, 1), vendor="(spec) GM-62")
def make():
    D, T = C.GIMBAL_D, C.GIMBAL_T
    body = (cq.Workplane("XY").workplane(offset=1.5).circle(D / 2).circle(6).extrude(T - 1.5)
            .edges(">Z").fillet(2))
    rotor = cq.Workplane("XY").circle(D / 2 - 4).circle(6).extrude(1.5)
    holes = (cq.Workplane("XY").workplane(offset=T).polarArray(D / 2 - 7, 0, 360, 4)
             .circle(1.3).extrude(-5))
    return body.union(rotor).cut(holes)


if __name__ == "__main__":
    main(make)
