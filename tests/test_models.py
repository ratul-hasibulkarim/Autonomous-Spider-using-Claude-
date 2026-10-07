"""The exported physics model and URDF load and agree with the CAD."""
from pathlib import Path

import mujoco
import pytest

from spider.export.mesh import export_link_meshes
from spider.export.mjcf import write_mjcf
from spider.export.urdf import write_urdf
from spider.robot import build


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    out = tmp_path_factory.mktemp("model")
    r = build()
    names = export_link_meshes(r, out / "meshes", tol=1.5)
    inert = write_urdf(r, names, out / "spider.urdf")
    write_mjcf(r, names, out / "spider.xml")
    return out, sum(i.mass for i in inert.values())


def test_mjcf(models):
    out, mass = models
    m = mujoco.MjModel.from_xml_path(str(out / "spider.xml"))
    assert m.nu == 26
    assert m.ntendon == 8                     # belt-coupled knees
    assert m.body_subtreemass[1] == pytest.approx(mass, rel=1e-5)     # MJCF stores 6 sig. figs
    assert m.body_subtreemass[1] == pytest.approx(55.0, abs=0.05)


def test_urdf(models):
    out, mass = models
    txt = (out / "spider.urdf").read_text()
    assert txt.count('type="revolute"') == 26 and txt.count('type="fixed"') == 4
    assert txt.count("<link ") == 31
    txt = txt.replace('<robot name="autonomous_spider">',
                      f'<robot name="autonomous_spider"><mujoco><compiler fusestatic="false" '
                      f'meshdir="{out}"/></mujoco>', 1)
    p = Path(out / "check.urdf")
    p.write_text(txt)
    u = mujoco.MjModel.from_xml_path(str(p))
    assert u.body_mass.sum() == pytest.approx(mass, rel=1e-5)        # URDF stores 6 sig. figs
