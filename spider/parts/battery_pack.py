"""48 V Li-ion pack, 13S{P}P 21700 (5.0 Ah cells), smart BMS, aluminium case.

Lies flat between the chassis plates.  Mass = cells (69 g each) + case/BMS.
Frame: base on z=0, centred in x/y.
"""
import cadquery as cq

from spider.parts._base import main, part

CELL_G = 0.069


def _mass(P=5):
    return 13 * P * CELL_G + 0.62


@part("BAT-13S{P}P-21700", "Battery 48 V 13S{P}P 21700 Li-ion w/ BMS",
      material="COTS", category="cots", mass=_mass,
      color=(0.12, 0.30, 0.55, 1), vendor="(spec) custom pack")
def make(P=5):
    L, W, H = 13 * 21.6 + 14, P * 21.6 * 1.62 + 14, 46.0
    case = cq.Workplane("XY").box(L, W, H, centered=(True, True, False)).edges("|Z").fillet(6)
    ribs = (cq.Workplane("XY").workplane(offset=H).rarray(L / 7, 1, 6, 1)
            .rect(3, W - 20).extrude(-1.2))
    conn = cq.Workplane("XY").box(30, 16, 22).translate((L / 2 + 8, W / 2 - 25, 14))
    xt = cq.Workplane("XY").box(16, 10, 10).translate((L / 2 + 5, -W / 2 + 25, 14))
    return case.cut(ribs).union(conn).union(xt)


if __name__ == "__main__":
    main(make)
