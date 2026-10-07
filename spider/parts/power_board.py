"""Power distribution board: 48 V bus, 4 fused leg branches, pre-charge,
e-stop contactor, current sensing.  Frame: PCB bottom at z=0."""
import cadquery as cq

from spider.parts._base import main, part


@part("PDB-48V-4CH", "Power distribution board 48 V 4-branch",
      material="PCB", category="electronic", mass=0.28, vendor="in-house PCBA")
def make():
    pcb = (cq.Workplane("XY").rect(120, 90).extrude(2.0)
           .faces(">Z").workplane().rect(110, 80, forConstruction=True).vertices().hole(3.2))
    contactor = cq.Workplane("XY").workplane(offset=2).center(-30, 10).box(34, 30, 32, centered=(True, True, False))
    fuses = cq.Workplane("XY").workplane(offset=2).center(28, 0).rarray(1, 16, 1, 4).rect(26, 9).extrude(14)
    bus = cq.Workplane("XY").workplane(offset=2).center(0, -32).rect(90, 8).extrude(10)
    return pcb.union(contactor).union(fuses).union(bus)


if __name__ == "__main__":
    main(make)
