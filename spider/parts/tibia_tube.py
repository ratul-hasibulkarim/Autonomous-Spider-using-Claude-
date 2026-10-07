"""Tibia tube: roll-wrapped high-modulus CFRP, 32 OD x 2 wall.
Frame: tibia frame; tube spans x in [70, 352]."""
import cadquery as cq

from spider.parts._base import main, part

X0, X1 = 70.0, 352.0


@part("SP-2202", "Tibia tube CFRP 32x2", material="CFRP", category="composite",
      vendor="(spec) roll-wrapped CF")
def make():
    return cq.Workplane("YZ").workplane(offset=X0).circle(16).circle(14).extrude(X1 - X0)


if __name__ == "__main__":
    main(make)
