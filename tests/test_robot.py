"""Spec tests: the numbers the robot is built to, checked from the part programs."""
import importlib
import pkgutil
import subprocess
import sys

import numpy as np
import pytest

import spider.parts
from spider import config as C
from spider.kinematics import leg_fk, leg_ik, leg_jacobian
from spider.parts._base import PartDef
from spider.robot import build, standing_pose


@pytest.fixture(scope="module")
def robot():
    return build()


def test_part_count(robot):
    assert len(robot.instances) == 986


def test_joints(robot):
    kinds = [j.type for j in robot.joints]
    assert len(robot.joints) == 30
    assert kinds.count("revolute") == 26 and kinds.count("fixed") == 4
    assert len(robot.links) == len(robot.joints) + 1          # a tree


def test_motor_drivers(robot):
    drivers = {i.name for i in robot.instances if i.variant.pdef.module.endswith("motor_driver")}
    assert len(drivers) == 26
    driven = [j.driver for j in robot.joints if j.type == "revolute"]
    assert sorted(driven) == sorted(drivers)                   # one driver per actuated joint


def test_mass(robot):
    mass = sum(i.variant.massprops()[0] for i in robot.instances)
    assert mass == pytest.approx(55.0, abs=0.05)


def test_sensors_and_compute(robot):
    nums = {i.variant.number for i in robot.instances}
    assert {"LIDAR-3D-128", "JETSON-AGX-ORIN-64", "IMU-6DOF-T", "CAM-STEREO-95"} <= nums


def test_every_part_is_a_program(robot):
    modules = {i.variant.pdef.module for i in robot.instances}
    for info in pkgutil.iter_modules(spider.parts.__path__):
        if info.name.startswith("_"):
            continue
        mod = importlib.import_module(f"spider.parts.{info.name}")
        assert isinstance(mod.make, PartDef), info.name
        assert 'if __name__ == "__main__":' in open(mod.__file__).read(), info.name
    assert len(modules) >= 45


def test_part_program_runs_standalone(tmp_path):
    out = subprocess.run([sys.executable, "-m", "spider.parts.shcs", "d=3", "L=8"], cwd=tmp_path,
                         capture_output=True, text=True, env={"PYTHONPATH": str(C.__file__).rsplit("/spider/", 1)[0]})
    assert out.returncode == 0, out.stderr
    assert (tmp_path / "build/parts/ISO4762-M3x8.step").exists()


def test_ik_fk_roundtrip():
    rng = np.random.default_rng(0)
    q = np.stack([rng.uniform(-0.6, 0.6, 500), rng.uniform(-0.9, 1.0, 500), rng.uniform(0.2, 2.1, 500)], -1)
    rho = C.COXA_M + C.FEMUR_L_M * np.cos(q[:, 1]) + C.TIBIA_L_M * np.cos(q[:, 1] + q[:, 2])
    q = q[rho > C.COXA_M + 0.02]          # foot outboard of the femur axis (else yaw is ambiguous)
    assert len(q) > 300
    assert np.allclose(leg_ik(leg_fk(q), clamp=False), q, atol=1e-9)


def test_jacobian():
    q = np.array([0.2, -0.3, 1.4])
    J = leg_jacobian(q)
    e = 1e-6
    Jn = np.column_stack([(leg_fk(q + e * np.eye(3)[k]) - leg_fk(q)) / e for k in range(3)])
    assert np.allclose(J, Jn, atol=1e-5)


def test_standing_pose_within_limits():
    q = standing_pose()
    for j, (lo, hi) in [("femur", C.FEMUR_LIMIT), ("knee", C.KNEE_LIMIT), ("coxa", C.COXA_LIMIT)]:
        for leg in C.LEG_NAMES:
            assert lo <= q[f"{leg}_{j}_joint"] <= hi


def test_identical_legs(robot):
    """One leg SKU: all eight legs carry the same parts in the same places."""
    def sig(leg):
        return sorted((i.link[3:], i.variant.number, tuple(np.round(i.T, 6).ravel()))
                      for i in robot.instances if i.link.startswith(leg + "_"))
    ref = sig("L1")
    for leg in C.LEG_NAMES[1:]:
        assert sig(leg) == ref
