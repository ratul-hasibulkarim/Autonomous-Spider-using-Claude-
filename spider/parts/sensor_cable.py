"""Foot load-cell cable, shielded 4-core 26 AWG, length L."""
import cadquery as cq

from spider.parts._base import main, part


@part("HAR-SENS-{L}", "Load-cell cable {L} mm", material="Cable", category="cable",
      mass=lambda L: 0.021 * L / 1000 + 0.004, vendor="in-house harness")
def make(L=900):
    return (cq.Workplane("YZ").circle(2.2).extrude(L)
            .union(cq.Workplane("YZ").rect(9, 7).extrude(12)))


if __name__ == "__main__":
    main(make)
