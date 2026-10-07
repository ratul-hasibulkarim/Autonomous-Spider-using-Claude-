"""Closed-loop smoke test: the controller walks on flat ground and over rocks."""
from pathlib import Path

import pytest

MODEL = Path(__file__).resolve().parents[1] / "build" / "model" / "spider.xml"
pytestmark = pytest.mark.skipif(not MODEL.exists(), reason="run `python build.py --no-step` first")


def test_flat_walk():
    from sim.walk import run_trial
    r = run_trial(0, "flat", t_max=8.0, verbose=False)
    assert not r["fell"]
    assert r["distance_m"] > 0.9            # 0.16 m/s for 6.5 s
    assert r["max_roll_deg"] < 5 and r["max_pitch_deg"] < 5


def test_rock_walk_short():
    from sim.walk import run_trial
    r = run_trial(3, "boulders", t_max=20.0, verbose=False)
    assert not r["fell"]
    assert r["distance_m"] > 2.5
    assert r["chassis_terrain_contacts"] == 0
