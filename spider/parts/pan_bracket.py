"""Pan bracket: carries the tilt motor.  7075, CNC.
Frame: pan-joint frame; base plate on the pan rotor (z in [0,5]); tilt axis
along y at z=TILT_DZ; tilt-motor seat on the inner face of the upright (y=-53)."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-1070", "Pan bracket", material="Al7075-T6", category="machined")
def make():
    z = C.TILT_DZ
    base = (cq.Workplane("XY").circle(C.GIMBAL_D / 2 - 2).extrude(5)
            .faces(">Z").workplane().polarArray(C.GIMBAL_D / 2 - 11, 0, 360, 4).hole(2.8))
    arm = cq.Workplane("XY").center(0, -42).rect(44, 28).extrude(5)
    upright = (cq.Workplane("XZ").workplane(offset=53).moveTo(-22, 0).lineTo(22, 0)
               .lineTo(22, z).threePointArc((0, z + 34), (-22, z)).close().extrude(5))
    seat_holes = (cq.Workplane("XZ").workplane(offset=50).pushPoints(
        [(x, z + y) for x, y in [(-21, 0), (21, 0), (0, 21), (0, -21)]]).circle(1.4).extrude(12))
    gusset = (cq.Workplane("YZ").moveTo(-53, 5).lineTo(-30, 5).lineTo(-53, 30).close()
              .extrude(3, both=True))
    return base.union(arm).union(upright).union(gusset).cut(seat_holes)


if __name__ == "__main__":
    main(make)
