"""The MuJoCo terrain and the Nav2 slope map must describe the same ground at the same (x, y).
If the heightfield were flipped, Nav2 would plan around hills that are somewhere else.
Also checks that the 10x10 collision tiles match the visual full field (including on seams)
and that no wheel starts below the ground at spawn."""
import json
import pathlib
import sys

import mujoco
import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import make_mars_world as mw  # noqa: E402

WORLD = ROOT / "src/mars_yard_worlds/worlds/mars.xml"

COLLISION_GROUPS = np.array([1, 1, 0, 1, 0, 0], dtype=np.uint8)
TILE_ONLY_GROUPS = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
VISUAL_ONLY_GROUPS = np.array([0, 0, 1, 0, 0, 0], dtype=np.uint8)


@pytest.fixture(scope="module")
def sim():
    model = mujoco.MjModel.from_xml_path(str(WORLD))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def _tile_geom_ids(model):
    return {
        model.geom(i).id
        for i in range(model.ngeom)
        if model.geom(i).name.startswith("terrain_tile_")
    }


def test_heightfield_matches_dem_orientation(sim):
    """Downward rays on the collision tiles hit the DEM height looked up with the Nav2 convention."""
    model, data = sim
    t = mw.Terrain()
    z0 = mw.spawn_ground_height(t)
    tile_ids = _tile_geom_ids(model)
    assert len(tile_ids) == mw.TILE_GRID * mw.TILE_GRID

    # Verify visual full terrain is non-colliding group 2.
    terrain_id = model.geom("terrain").id
    assert model.geom_group[terrain_id] == 2
    assert model.geom_contype[terrain_id] == 0
    assert model.geom_conaffinity[terrain_id] == 0

    geomid = np.zeros(1, np.int32)
    errors = []
    for x, y in np.random.default_rng(0).uniform(-20, 20, (100, 2)):
        dist = mujoco.mj_ray(
            model, data, np.array([x, y, 50.0]), np.array([0, 0, -1.0]),
            COLLISION_GROUPS, 1, model.body("base_link").id, geomid,
        )
        if geomid[0] in tile_ids:
            errors.append(abs(50.0 - dist - (t.height(x, y) - z0)))
    assert len(errors) > 50
    assert np.median(errors) < 0.01  # nearest-pixel lookup vs MuJoCo's interpolation; a flip gives ~0.6 m


def test_collision_tiles_match_visual_full_field(sim):
    """At 200 random xy points plus tile seams, collision tiles match the visual full field within 1 mm."""
    model, data = sim
    t = mw.Terrain()
    terrain_id = model.geom("terrain").id
    tile_ids = _tile_geom_ids(model)

    rng = np.random.default_rng(42)
    pts = []
    # 200 random points across the terrain (avoiding the rover's visual-only footprint at origin).
    while len(pts) < 200:
        x, y = rng.uniform(-t.half + 0.05, t.half - 0.05, size=2)
        if abs(x) < 0.6 and abs(y) < 0.6:
            continue
        pts.append((float(x), float(y)))

    # Add explicit points on interior tile seams (vertical, horizontal, and seam intersections).
    step = (2.0 * t.half) / (t.n - 1)
    bounds = mw.tile_bounds(t.n, mw.TILE_GRID)
    seam_x = [-t.half + c * step for c in bounds[1:-1]]
    seam_y = [t.half - r * step for r in bounds[1:-1]]
    for sx in seam_x:
        y = float(rng.uniform(-t.half + 1.0, t.half - 1.0))
        if abs(sx) < 0.6 and abs(y) < 0.6:
            y = 2.0
        pts.append((sx, y))
    for sy in seam_y:
        x = float(rng.uniform(-t.half + 1.0, t.half - 1.0))
        if abs(x) < 0.6 and abs(sy) < 0.6:
            x = 2.0
        pts.append((x, sy))
    for sx, sy in zip(seam_x, seam_y):
        if abs(sx) < 0.6 and abs(sy) < 0.6:
            continue
        pts.append((sx, sy))

    geomid_vis = np.zeros(1, np.int32)
    geomid_col = np.zeros(1, np.int32)
    diffs = []
    for x, y in pts:
        d_vis = mujoco.mj_ray(
            model, data, np.array([x, y, 50.0]), np.array([0.0, 0.0, -1.0]),
            VISUAL_ONLY_GROUPS, 1, -1, geomid_vis,
        )
        d_col = mujoco.mj_ray(
            model, data, np.array([x, y, 50.0]), np.array([0.0, 0.0, -1.0]),
            TILE_ONLY_GROUPS, 1, -1, geomid_col,
        )
        assert d_vis > 0 and geomid_vis[0] == terrain_id, f"visual ray missed terrain at ({x}, {y})"
        assert d_col > 0 and geomid_col[0] in tile_ids, f"collision tile ray missed at ({x}, {y})"
        diffs.append(abs((50.0 - d_col) - (50.0 - d_vis)))

    assert max(diffs) < 1e-3, f"max tile-vs-full height diff {max(diffs)*1000:.3f} mm >= 1 mm"


def test_spawn_ground_is_at_zero(sim):
    """Highest ground point under the 4 spawn wheel centres sits at z = 0."""
    model, data = sim
    geomid = np.zeros(1, np.int32)
    zs = []
    for x, y in mw.SPAWN_WHEEL_XY:
        dist = mujoco.mj_ray(
            model, data, np.array([x, y, 5.0]), np.array([0.0, 0.0, -1.0]),
            TILE_ONLY_GROUPS, 1, -1, geomid,
        )
        assert dist > 0
        zs.append(5.0 - dist)
    assert max(zs) == pytest.approx(0.0, abs=0.01)


def test_no_wheel_below_ground_at_spawn(sim):
    """At the spawn pose, every wheel bottom clears the terrain and has no initial penetration."""
    model, data = sim
    wheel_names = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")
    wheel_ids = {model.geom(name).id for name in wheel_names}
    geomid = np.zeros(1, np.int32)

    for name in wheel_names:
        gid = model.geom(name).id
        wx, wy, wz = data.geom_xpos[gid]
        radius = float(model.geom_size[gid][0])
        wheel_bottom = wz - radius
        dist = mujoco.mj_ray(
            model, data, np.array([wx, wy, 5.0]), np.array([0.0, 0.0, -1.0]),
            TILE_ONLY_GROUPS, 1, -1, geomid,
        )
        assert dist > 0
        ground_z = 5.0 - dist
        assert wheel_bottom >= ground_z, (
            f"{name} starts below ground: bottom={wheel_bottom:.4f} m < ground={ground_z:.4f} m"
        )

    for ci in range(data.ncon):
        c = data.contact[ci]
        if c.geom1 in wheel_ids or c.geom2 in wheel_ids:
            assert c.dist >= 0.0, (
                f"initial wheel penetration at spawn: "
                f"{model.geom(c.geom1).name} <-> {model.geom(c.geom2).name} dist={c.dist*1000:.2f} mm"
            )


def test_boulders_present_rayhit_and_preserve_existing_obstacles(sim):
    """Boulders are 0.30-0.50 m exposed, ray-hit, >=2 near spawn, and original 40 rocks + 3 sand are unchanged."""
    model, data = sim
    obs = json.loads((mw.TERRAIN / "obstacles.json").read_text())
    t = mw.Terrain()

    # Original 40 rocks and 3 sand patches must keep their exact seeded positions.
    rng = np.random.default_rng(obs["seed"])
    expected_rocks = mw.place_rocks(t, rng, 40)
    expected_sand = mw.place_sand(t, rng, 3)
    assert obs["rocks"][:40] == expected_rocks
    assert obs["sand"] == expected_sand

    boulders = [r for r in obs["rocks"] if r.get("kind") == "boulder"]
    assert len(boulders) == 8

    near_count = 0
    geomid = np.zeros(1, np.int32)
    z0 = mw.spawn_ground_height(t)
    for idx, b in enumerate(boulders):
        dist_spawn = float(np.hypot(b["x"], b["y"]))
        assert dist_spawn >= mw.SPAWN_CLEAR
        if 3.0 <= dist_spawn <= 10.0:
            near_count += 1
        assert t.slope[t.index(b["x"], b["y"])] >= 200
        exposed = 0.7 * b["size"][2]
        assert 0.299 <= exposed <= 0.501, f"boulder_{idx} exposed height {exposed:.3f} m out of [0.30, 0.50]"

        gid = model.geom(b["name"]).id
        assert int(model.geom_group[gid]) in (0, 1, 3)
        mat_id = int(model.geom_matid[gid])
        alpha = float(model.mat_rgba[mat_id, 3]) if mat_id >= 0 else float(model.geom_rgba[gid, 3])
        assert alpha > 0.0

        # Downward ray at boulder centre must hit the boulder geom.
        ray_d = mujoco.mj_ray(
            model, data, np.array([b["x"], b["y"], 50.0]), np.array([0.0, 0.0, -1.0]),
            COLLISION_GROUPS, 1, -1, geomid,
        )
        assert ray_d > 0 and geomid[0] == gid, f"downward ray missed {b['name']}"
        hit_z = 50.0 - ray_d
        expected_top = (b["z"] - z0) + b["size"][2]
        assert hit_z == pytest.approx(expected_top, abs=0.01)

    assert near_count >= 2, f"expected >=2 boulders within 3-10 m of spawn, got {near_count}"
