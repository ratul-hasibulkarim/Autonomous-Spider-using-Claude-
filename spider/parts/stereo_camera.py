"""Global-shutter stereo depth camera, 95 mm baseline, IMU.
Frame: optical centre at origin, looking along +x."""
import cadquery as cq

from spider.parts._base import main, part


@part("CAM-STEREO-95", "Stereo depth camera 95 mm baseline",
      material="COTS", category="electronic", mass=0.072,
      color=(0.15, 0.15, 0.16, 1), vendor="(spec) stereo cam")
def make():
    body = cq.Workplane("XY").box(25, 124, 29).edges("|X").fillet(10)
    lenses = (cq.Workplane("YZ").workplane(offset=12.5).pushPoints([(-47.5, 0), (47.5, 0), (0, 0)])
              .circle(5).extrude(1.5))
    tripod = cq.Workplane("XY").workplane(offset=-14.5).circle(3).extrude(-4)
    return body.union(lenses).union(tripod)


if __name__ == "__main__":
    main(make)
