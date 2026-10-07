"""Stereo camera bracket, 15 deg down-tilt.  PA12-MJF.
Frame: foot on z=0, holes at (-10, +-20) relative to CAMERA_X."""
import cadquery as cq

from spider.parts._base import main, part


@part("SP-1090", "Stereo camera bracket", material="PA12-MJF", category="printed")
def make():
    foot = (cq.Workplane("XY").center(-10, 0).rect(30, 70).extrude(4)
            .faces(">Z").workplane().pushPoints([(-10, -20), (-10, 20)]).hole(4.5))
    post = cq.Workplane("XY").center(0, 0).rect(8, 36).extrude(30)
    head = (cq.Workplane("XY").workplane(offset=26).center(2, 0).rect(12, 40).extrude(4)
            .rotate((0, 0, 28), (0, 1, 28), 15))
    return foot.union(post).union(head)


if __name__ == "__main__":
    main(make)
