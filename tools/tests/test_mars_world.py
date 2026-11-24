"""The MuJoCo terrain and the Nav2 slope map must describe the same ground at the same (x, y).
If the heightfield were flipped, Nav2 would plan around hills that are somewhere else."""
import pathlib
import sys

import mujoco
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import make_mars_world as mw  # noqa: E402

WORLD = ROOT / "src/mars_yard_worlds/worlds/mars.xml"


@pytest.fixture(scope="module")
def sim():
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def test_heightfield_matches_dem_orientation(sim):
    """Downward rays on the MuJoCo terrain hit the DEM height looked up with the Nav2 convention."""
    model, data = sim
    t = mw.Terrain()
    z0 = t.height(0, 0)
    terrain_id = model.geom("terrain").id
    geomid = np.zeros(1, np.int32)
    errors = []
    for x, y in np.random.default_rng(0).uniform(-20, 20, (100, 2)):
        dist = mujoco.mj_ray(model, data, np.array([x, y, 50.0]), np.array([0, 0, -1.0]),
                             None, 1, model.body("base_link").id, geomid)
        if geomid[0] == terrain_id:
            errors.append(abs(50 - dist - (t.height(x, y) - z0)))
    assert len(errors) > 50
    assert np.median(errors) < 0.01  # nearest-pixel lookup vs MuJoCo's interpolation; a flip gives ~0.6 m


def test_spawn_ground_is_at_zero(sim):
    model, data = sim
    geomid = np.zeros(1, np.int32)
    dist = mujoco.mj_ray(model, data, np.array([0, 0, 5.0]), np.array([0, 0, -1.0]),
                         None, 1, model.body("base_link").id, geomid)
    assert 5 - dist == pytest.approx(0.0, abs=0.03)
