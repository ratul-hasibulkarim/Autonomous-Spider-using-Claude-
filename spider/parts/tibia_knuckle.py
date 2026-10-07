"""Tibia knuckle: twin cheeks around the knee pulley joined into the tube
socket.  7075-T6, 5-axis.  Frame: tibia frame (origin knee axis, x along tibia)."""
import cadquery as cq

from spider.parts._base import main, part

SOCKET_X0, SOCKET_X1 = 62.0, 108.0
CLAMP_BOLTS_X = (78.0, 98.0)


@part("SP-2201", "Tibia knuckle", material="Al7075-T6", category="machined")
def make():
    cheeks = None
    for y0 in (10.0, -13.0):   # 3 mm cheeks either side of the 20 mm pulley
        c = (cq.Workplane("XZ").workplane(offset=-y0 - 3).circle(21).extrude(3)
             .union(cq.Workplane("XZ").workplane(offset=-y0 - 3).center(35, 0).rect(70, 30).extrude(3))
             .cut(cq.Workplane("XZ").workplane(offset=-y0 - 5).circle(12.6).extrude(6))
             .cut(cq.Workplane("XZ").workplane(offset=-y0 - 5).polarArray(17, 45, 360, 4).circle(1.7).extrude(6)))
        cheeks = c if cheeks is None else cheeks.union(c)
    socket = (cq.Workplane("YZ").workplane(offset=SOCKET_X0).circle(19.5).extrude(SOCKET_X1 - SOCKET_X0)
              .faces(">X").workplane().hole(32.2, SOCKET_X1 - SOCKET_X0 - 8))
    lug = cq.Workplane("XY").center((SOCKET_X0 + SOCKET_X1) / 2 + 6, 0).box(34, 18, 10).translate((0, 0, 22))
    slot = cq.Workplane("XY").center(95, 0).box(40, 2.0, 14).translate((0, 0, 21))
    bolts = cq.Workplane("XZ").pushPoints([(x, 23) for x in CLAMP_BOLTS_X]).circle(2.7).extrude(20, both=True)
    web = cq.Workplane("XY").center(63, 0).box(14, 26, 26)   # ends at the tube end (x=70)
    return cheeks.union(web).union(socket).union(lug).cut(slot).cut(bolts)


if __name__ == "__main__":
    main(make)
