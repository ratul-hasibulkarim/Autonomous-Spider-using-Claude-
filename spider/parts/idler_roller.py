"""Belt tensioner roller: crowned PA sleeve on a 625 bearing.  Frame: axis y."""
import cadquery as cq

from spider import config as C
from spider.parts._base import main, part


@part("SP-2105", "Belt idler roller", material="POM", category="machined")
def make():
    return (cq.Workplane("XZ").circle(9.5).circle(8).extrude(C.BELT_W / 2 + 1, both=True)
            .edges().fillet(0.4))


if __name__ == "__main__":
    main(make)
