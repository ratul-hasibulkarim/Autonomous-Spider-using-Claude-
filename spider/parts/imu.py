"""Tactical-grade 6-axis IMU (+ magnetometer), aluminium case.  Frame: base at z=0."""
import cadquery as cq

from spider.parts._base import main, part


@part("IMU-6DOF-T", "Tactical IMU 6-DOF 400 Hz",
      material="COTS", category="electronic", mass=0.055,
      color=(0.55, 0.10, 0.10, 1), vendor="(spec) IMU")
def make():
    body = cq.Workplane("XY").box(38, 38, 22, centered=(True, True, False)).edges("|Z").fillet(3)
    flange = (cq.Workplane("XY").rect(50, 30).extrude(3)
              .faces(">Z").workplane().pushPoints([(-21, 0), (21, 0)]).hole(3.2))
    return body.union(flange)


if __name__ == "__main__":
    main(make)
