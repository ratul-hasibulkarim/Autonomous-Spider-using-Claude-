"""Chassis plates (top / bottom).  6061-T6, 3 mm, waterjet + CNC.

The two plates and 14 hex standoffs form a torsion box that carries the eight
coxa actuators between them.  Every fastener hole is generated from
``spider.layout`` - the same lists the assembly uses to place the screws.
The top plate is delivered with PEM CLS-M5/M6 clinch nuts pressed in (mast
and carry handles), so those nuts are not separate assembly items.
Frame: base_link; plate occupies z in [0, 3] (shifted by the assembly).
"""
import math

import cadquery as cq

from spider import config as C
from spider import layout as LY
from spider.parts._base import main, part


def _outline(t):
    return (cq.Workplane("XY").rect(C.CHASSIS_LEN, C.CHASSIS_WID).extrude(t)
            .edges("|Z").fillet(C.CHASSIS_CORNER_R))


def _keepouts(which):
    """(x, y, radius) circles that lightening pockets must avoid."""
    k = [(x, y, 52.0 if which == "top" else 50.0) for x, y, _ in LY.HIPS]
    k += [(x, y, 16.0) for x, y in LY.STANDOFFS]
    k += [(x, y, 12.0) for x, y in LY.bulkhead_holes()]
    if which == "top":
        k += [(x, y, 10.0) for x, y in LY.board_holes() + LY.enclosure_flange_holes()
              + LY.handle_holes() + LY.cage_holes(LY.GIMBAL_CAGE_XY)]
        k += [(LY.MAST_XY[0], LY.MAST_XY[1], 45.0), (*LY.ESTOP_XY, 32.0), (*LY.BEACON_XY, 26.0),
              (LY.CAMERA_X, 0.0, 30.0), (*LY.IMU_XY, 34.0)]
        k += [(x, y, 12.0) for x, y in LY.ANTENNA_XY]
    else:
        k += [(x, y, 12.0) for x, y in LY.battery_clamp_holes()]
        k += [(x, y, 10.0) for c in LY.cage_positions().values() for x, y in LY.cage_holes(c)]
    return k


def _pockets(which, pitch=45.0, d=40.0):
    keep = _keepouts(which)
    hl, hw, rc = C.CHASSIS_LEN / 2 - 45, C.CHASSIS_WID / 2 - 45, C.CHASSIS_CORNER_R - 45
    pts = []
    ny, nx = int(2 * hw / pitch) + 1, int(2 * hl / pitch) + 1
    for i in range(nx):
        for j in range(ny):
            x = -hl + d / 2 + i * pitch + (pitch / 2 if j % 2 else 0)
            y = -hw + d / 2 + j * pitch * 0.866
            if abs(x) > hl - d / 2 or abs(y) > hw - d / 2:
                continue
            cx, cy = max(abs(x) - (hl - rc), 0), max(abs(y) - (hw - rc), 0)
            if math.hypot(cx, cy) > rc - d / 2:
                continue
            if all(math.hypot(x - kx, y - ky) > kr + d / 2 for kx, ky, kr in keep):
                pts.append((x, y))
    return pts


@part("SP-1001-{which}", "Chassis plate, {which}", material="Al6061-anodized-black",
      category="machined")
def make(which="top"):
    t = C.PLATE_T
    plate = _outline(t)
    wp = lambda: cq.Workplane("XY").workplane(offset=-1)
    holes = [(x, y, 5.5) for x, y in LY.STANDOFFS]
    holes += [(x, y, 4.5) for x, y in LY.bulkhead_holes()]
    if which == "top":
        holes += [(x, y, 4.5) for x, y in LY.hip_bolt_holes()]
        holes += [(x, y, 30.0) for x, y, _ in LY.HIPS]              # harness pass-through
        holes += [(x, y, 2.5) for x, y in LY.board_holes()]          # tapped M3
        holes += [(x, y, 3.4) for x, y in LY.cage_holes(LY.GIMBAL_CAGE_XY)]
        holes += [(x, y, 4.5) for x, y in LY.enclosure_flange_holes()]
        holes += [(x, y, 8.75) for x, y in LY.handle_holes()]         # PEM CLS-M6 mounting hole
        holes += [(x, y, 7.0) for x, y in LY.mast_holes()]            # PEM CLS-M5 mounting hole
        holes += [(*LY.ESTOP_XY, 22.5), (*LY.BEACON_XY, 12.0)]
        holes += [(x, y, 3.4) for x, y in LY.imu_holes()]
        holes += [(x, y, 6.5) for x, y in LY.ANTENNA_XY]
        holes += [(LY.CAMERA_X - 10, sy * 20.0, 4.5) for sy in (-1, 1)]
    else:
        holes += [(x, y, 84.0) for x, y, _ in LY.HIPS]              # coxa adapter clearance
        holes += [(x, y, 4.5) for x, y in LY.battery_clamp_holes()]
        holes += [(x, y, 3.4) for c in LY.cage_positions().values() for x, y in LY.cage_holes(c)]
    for dia in sorted({h[2] for h in holes}):
        pts = [(x, y) for x, y, d in holes if d == dia]
        plate = plate.cut(wp().pushPoints(pts).circle(dia / 2).extrude(t + 2))
    pockets = _pockets(which)
    if pockets:
        plate = plate.cut(wp().pushPoints(pockets).polygon(6, 40.0).extrude(t + 2))
    return plate


if __name__ == "__main__":
    main(make)
