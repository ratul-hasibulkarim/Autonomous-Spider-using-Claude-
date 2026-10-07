"""Isolated DC-DC brick, 48 V in -> {Vout} V out, base-plate cooled."""
import cadquery as cq

from spider.parts._base import main, part


@part("DCDC-48-{Vout}V", "DC-DC converter 48 V -> {Vout} V",
      material="COTS", category="electronic", mass=0.135, vendor="(spec) DC-DC brick")
def make(Vout=12):
    body = (cq.Workplane("XY").box(61, 58, 13, centered=(True, True, False))
            .faces(">Z").workplane().rect(50, 48, forConstruction=True).vertices().hole(3.2))
    pins = cq.Workplane("XY").workplane(offset=13).center(-26, 0).rarray(1, 9, 1, 5).circle(1).extrude(4)
    return body.union(pins)


if __name__ == "__main__":
    main(make)
