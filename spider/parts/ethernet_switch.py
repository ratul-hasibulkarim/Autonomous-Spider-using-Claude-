"""5-port gigabit Ethernet switch board (LiDAR, Jetson, radio, debug)."""
import cadquery as cq

from spider.parts._base import main, part


@part("ETH-SW-5G", "Gigabit Ethernet switch 5-port",
      material="PCB", category="electronic", mass=0.05, vendor="(spec) GbE switch")
def make():
    pcb = (cq.Workplane("XY").rect(70, 55).extrude(1.6)
           .faces(">Z").workplane().rect(62, 47, forConstruction=True).vertices().hole(3.2))
    jacks = cq.Workplane("XY").workplane(offset=1.6).center(-28, 0).rarray(1, 10.5, 1, 5).rect(8, 9).extrude(10)
    return pcb.union(jacks)


if __name__ == "__main__":
    main(make)
