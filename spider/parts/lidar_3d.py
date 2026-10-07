"""360 deg x 90 deg 3D LiDAR, 128 channels, 0.1-70 m (Livox Mid-360-class optics,
Ouster-class housing).  Frame: optical centre at origin, base at z=-42."""
import cadquery as cq

from spider.parts._base import main, part


@part("LIDAR-3D-128", "3D LiDAR 128-ch 360x90 deg",
      material="COTS", category="electronic", mass=0.83,
      color=(0.18, 0.18, 0.20, 1), vendor="(spec) 3D-LiDAR")
def make():
    base = cq.Workplane("XY").workplane(offset=-42).circle(43).extrude(22).edges("<Z").fillet(3)
    window = cq.Workplane("XY").workplane(offset=-20).circle(41).extrude(34)
    cap = cq.Workplane("XY").workplane(offset=14).circle(43).extrude(16).edges(">Z").fillet(6)
    mount = (cq.Workplane("XY").workplane(offset=-42).rect(50, 50, forConstruction=True)
             .vertices().circle(2).extrude(8))
    return base.union(window).union(cap).cut(mount)


if __name__ == "__main__":
    main(make)
