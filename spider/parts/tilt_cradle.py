"""Tilt cradle: cantilever arm from the tilt rotor up to the LiDAR deck.
Frame: tilt-joint frame (axis y). Rotor face at y=-27; deck top at z=+35."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-1080", "LiDAR tilt cradle", material="Al7075-T6", category="machined")
def make():
    hub = cq.Workplane("XZ").workplane(offset=27).circle(26).extrude(-5)
    arm = cq.Workplane("XZ").workplane(offset=27).center(0, 15).rect(52, 34).extrude(-5)
    deck = (cq.Workplane("XY").workplane(offset=31).center(0, 0).rect(100, 104).extrude(4)
            .edges("|Z").fillet(14)
            .faces(">Z").workplane().rect(50, 50, forConstruction=True).vertices().hole(4.5)
            .faces(">Z").workplane().circle(18).cutThruAll())
    rib = cq.Workplane("YZ").moveTo(-27, 0).lineTo(-27, 31).lineTo(-5, 31).close().extrude(4, both=True)
    holes = (cq.Workplane("XZ").workplane(offset=30).polarArray(20, 45, 360, 4).circle(1.4).extrude(-10))
    return hub.union(arm).union(deck).union(rib).cut(holes)


if __name__ == "__main__":
    main(make)
