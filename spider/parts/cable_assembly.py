"""Shielded power + CAN-FD cable assembly (2x16 AWG + 2x24 AWG twisted pair).

Modelled as a straight run of length L (mm) with over-moulded ends.
Mass from datasheet: 46 g/m + 9 g of connectors.  Frame: from origin along +x.
"""
import cadquery as cq

from spider.parts._base import main, part


@part("HAR-PWRCAN-{L}", "Power/CAN harness {L} mm",
      material="Cable", category="cable", mass=lambda L: 0.046 * L / 1000 + 0.009,
      vendor="in-house harness")
def make(L=300):
    run = cq.Workplane("YZ").circle(4).extrude(L)
    ends = cq.Workplane("YZ").pushPoints([(0, 0)]).rect(14, 10).extrude(16)
    ends2 = cq.Workplane("YZ").workplane(offset=L - 16).rect(14, 10).extrude(16)
    return run.union(ends).union(ends2)


if __name__ == "__main__":
    main(make)
