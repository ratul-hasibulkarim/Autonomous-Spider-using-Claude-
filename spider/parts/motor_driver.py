"""FOC motor driver board: 48 V / 40 A, CAN-FD, on-board encoder interface.

One per actuator (26 total).  80 x 64 mm board with heat spreader.
Frame: board bottom on z=0, centred.
"""
import cadquery as cq

from spider.parts._base import main, part


@part("DRV-FOC-48V40A", "FOC motor driver 48 V 40 A CAN-FD",
      material="PCB", category="electronic", mass=0.072, vendor="(spec) FOC-40")
def make():
    pcb = (cq.Workplane("XY").rect(64, 64).extrude(1.6).edges("|Z").fillet(3)
           .faces(">Z").workplane().rect(56, 56, forConstruction=True).vertices().hole(3.2))
    fets = cq.Workplane("XY").workplane(offset=1.6).rarray(10, 12, 3, 2).rect(6, 5).extrude(1.2)
    caps = (cq.Workplane("XY").workplane(offset=1.6).center(-18, 18).rarray(9, 1, 3, 1)
            .circle(4).extrude(11))
    spreader = (cq.Workplane("XY").workplane(offset=2.8).center(4, -6).rect(36, 26).extrude(4)
                .faces(">Z").workplane().rarray(3.5, 1, 9, 1).rect(1.4, 24).cutBlind(-3))
    conn = (cq.Workplane("XY").workplane(offset=1.6).pushPoints([(24, 20), (24, -20)])
            .rect(10, 14).extrude(7))
    return pcb.union(fets).union(caps).union(spreader).union(conn)


if __name__ == "__main__":
    main(make)
