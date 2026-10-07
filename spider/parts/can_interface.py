"""4-channel CAN-FD to USB/Ethernet interface, one bus per leg pair."""
import cadquery as cq

from spider.parts._base import main, part


@part("CANFD-4CH", "CAN-FD interface 4-channel",
      material="PCB", category="electronic", mass=0.06, vendor="(spec) CAN-FD")
def make():
    pcb = (cq.Workplane("XY").rect(80, 50).extrude(1.6)
           .faces(">Z").workplane().rect(72, 42, forConstruction=True).vertices().hole(3.2))
    conns = cq.Workplane("XY").workplane(offset=1.6).center(34, 0).rarray(1, 11, 1, 4).rect(10, 8).extrude(8)
    return pcb.union(conns)


if __name__ == "__main__":
    main(make)
