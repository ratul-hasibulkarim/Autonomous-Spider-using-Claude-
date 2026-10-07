"""NVIDIA Jetson AGX Orin 64 GB module on carrier, with active heatsink.
Frame: carrier bottom at z=0, centred."""
import cadquery as cq

from spider.parts._base import main, part


@part("JETSON-AGX-ORIN-64", "Jetson AGX Orin 64GB + carrier + heatsink",
      material="COTS", category="electronic", mass=1.10,
      color=(0.30, 0.55, 0.15, 1), vendor="NVIDIA")
def make():
    carrier = (cq.Workplane("XY").rect(130, 115).extrude(1.6).edges("|Z").fillet(3)
               .faces(">Z").workplane().rect(118, 103, forConstruction=True).vertices().hole(3.2))
    module = cq.Workplane("XY").workplane(offset=1.6 + 6).rect(100, 87).extrude(10)
    sink = (cq.Workplane("XY").workplane(offset=17.6).rect(105, 105).extrude(28)
            .faces(">Z").workplane().rarray(5, 1, 18, 1).rect(2.6, 100).cutBlind(-22))
    fan = cq.Workplane("XY").workplane(offset=45.6).circle(38).extrude(12)
    ports = cq.Workplane("XY").workplane(offset=1.6).center(-58, 0).rarray(1, 18, 1, 5).rect(14, 13).extrude(13)
    return carrier.union(module).union(sink).union(fan).union(ports)


if __name__ == "__main__":
    main(make)
