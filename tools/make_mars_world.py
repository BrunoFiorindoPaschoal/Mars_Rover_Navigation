"""Generate src/mars_yard_worlds/worlds/mars.xml from the terrain assets (Phase 2 / 3c / 5a).

    .venv/bin/python tools/make_mars_world.py [--seed 7] [--rocks 40] [--boulders 8] [--sand 3]

Inputs, from tools/dem_to_world.py, in src/mars_yard_worlds/terrain/:
heightmap.png, texture.png, terrain.json and slope.pgm.

It writes:
- mars.xml, with:
  * the full Jezero heightfield kept as visual-only (group 2, contype=0, conaffinity=0) so the
    HiRISE ortho texture maps seamlessly across the whole 50 x 50 m yard;
  * a 10 x 10 grid of 5 m collision tiles at the full 0.1 m resolution (group 3, invisible,
    1-cell overlap between neighbours so there are no gaps) for fast lidar raycasts;
  * rocks and low-friction sand patches.
  The terrain (visual field, collision tiles, sand, and rocks) is shifted vertically so that the
  HIGHEST ground point under the 4 rover wheel contact centres at spawn (x = +/-0.25, y = +/-0.32
  relative to base_link) sits at z = 0. With base_link spawning at z = 0.275 m (wheel bottoms at
  z = +0.005 m), no wheel starts inside the terrain and the rover drops at most a few cm on settle.
- heightmap.bin, terrain/tiles/terrain_tile_<i>_<j>.bin, and sand_<k>.bin, float32 heightfields
  for MuJoCo (see write_hfield_bin).
- obstacles.json, with the rock and sand positions, for the benchmark scenarios and the metrics.

Rocks and sand are placed with a fixed seed, so the world is reproducible.
"""
import argparse
import json
import pathlib

import numpy as np
from PIL import Image

ROOT = pathlib.Path(__file__).resolve().parents[1]
TERRAIN = ROOT / "src/mars_yard_worlds/terrain"
TILES = TERRAIN / "tiles"
OUT = ROOT / "src/mars_yard_worlds/worlds/mars.xml"

SAND_LIFT = 0.005      # m, sand surface sits this far above the main terrain
SAND_FRICTION = 0.35   # vs 0.8 on regolith: loose sand, wheels slip
SPAWN_CLEAR = 3.0      # m, no rocks or sand this close to the spawn point
TILE_GRID = 10         # 10 x 10 collision tiles (~5 m each across the 50 m terrain)

# Wheel contact centres at spawn pose (base_link at x=0, y=0), matching rover.xml.
SPAWN_WHEEL_XY = (
    (0.25, 0.32),
    (0.25, -0.32),
    (-0.25, 0.32),
    (-0.25, -0.32),
)


def write_hfield_bin(z, path):
    """Write MuJoCo's binary heightfield: int32 nrow, int32 ncol, float32 data.

    `z` comes in image convention (row 0 = north); the binary format stores row 0 = south
    (verified by tools/tests/test_mars_world.py), so the rows are flipped here.

    MuJoCo reads a PNG heightfield as 8-bit, i.e. 256 height steps. Over 7.5 m that is about
    3 cm terraces, which you can see and which shake the wheels, so we write floats instead.
    """
    z = np.asarray(z, dtype=np.float32)
    span = float(z.max() - z.min()) or 1.0
    norm = np.flipud((z - z.min()) / span).astype(np.float32)   # MuJoCo scales [0, 1] by size[2]
    with open(path, "wb") as f:
        np.array(norm.shape, dtype=np.int32).tofile(f)
        norm.tofile(f)


class Terrain:
    """Height lookup on the DEM grid. Image row 0 = north (+y), column 0 = west (-x)."""

    def __init__(self):
        self.meta = json.loads((TERRAIN / "terrain.json").read_text())
        png = np.asarray(Image.open(TERRAIN / "heightmap.png"), dtype=np.float64)
        self.z = png / 65535.0 * self.meta["z_range_m"]   # relative height, 0 = lowest point
        self.slope = np.asarray(Image.open(TERRAIN / "slope.pgm"))  # 254 free ... 0 lethal
        self.n = self.z.shape[0]
        self.half = self.meta["size_x_m"] / 2

    def index(self, x, y):
        """World (x, y) -> (row, col) in the image."""
        col = int(np.clip((x + self.half) / (2 * self.half) * (self.n - 1), 0, self.n - 1))
        row = int(np.clip((self.half - y) / (2 * self.half) * (self.n - 1), 0, self.n - 1))
        return row, col

    def height(self, x, y):
        return self.z[self.index(x, y)]


def spawn_ground_height(t):
    """Highest DEM ground elevation under the 4 wheel contact points at spawn."""
    return max(float(t.height(x, y)) for x, y in SPAWN_WHEEL_XY)


def tile_bounds(n, n_tiles=TILE_GRID):
    """Grid index boundaries partitioning [0 .. n-1] into n_tiles segments."""
    return [int(round(k * (n - 1) / n_tiles)) for k in range(n_tiles + 1)]


def build_collision_tiles(t, n_tiles=TILE_GRID):
    """Write n_tiles x n_tiles full-resolution collision tiles with 1-cell overlap.

    Neighbouring tiles share an overlap band of 1 grid cell (2 shared vertex rows/columns)
    so no wheel or ray can slip through a seam. Each tile's .bin is written with
    write_hfield_bin() and given its exact local [lo, lo + span] vertical mapping so its
    world surface matches the full heightfield identically.
    """
    TILES.mkdir(parents=True, exist_ok=True)
    n = t.n
    step = (2.0 * t.half) / (n - 1)
    bounds = tile_bounds(n, n_tiles)
    tiles = []

    for i in range(n_tiles):
        r0 = bounds[i]
        r1 = min(n - 1, bounds[i + 1] + 1) if i < n_tiles - 1 else bounds[i + 1]
        y_min = t.half - r1 * step   # south edge (image row r1)
        y_max = t.half - r0 * step   # north edge (image row r0)
        cy = 0.5 * (y_min + y_max)
        ry = 0.5 * (y_max - y_min)

        for j in range(n_tiles):
            c0 = bounds[j]
            c1 = min(n - 1, bounds[j + 1] + 1) if j < n_tiles - 1 else bounds[j + 1]
            x_min = -t.half + c0 * step  # west edge (col c0)
            x_max = -t.half + c1 * step  # east edge (col c1)
            cx = 0.5 * (x_min + x_max)
            rx = 0.5 * (x_max - x_min)

            crop = t.z[r0:r1 + 1, c0:c1 + 1]
            crop_f32 = np.asarray(crop, dtype=np.float32)
            lo = float(crop_f32.min())
            span = float(crop_f32.max() - lo) or 1.0

            name = f"terrain_tile_{i}_{j}"
            write_hfield_bin(crop, TILES / f"{name}.bin")
            tiles.append({
                "name": name,
                "file": f"../terrain/tiles/{name}.bin",
                "cx": cx,
                "cy": cy,
                "rx": rx,
                "ry": ry,
                "z_lo": lo,
                "z_span": span,
            })
    return tiles


def place_rocks(t, rng, count):
    """Random rocks on drivable ground (rocks on lethal slopes are already off-limits)."""
    rocks = []
    while len(rocks) < count:
        x, y = rng.uniform(-t.half + 2, t.half - 2, size=2)
        if np.hypot(x, y) < SPAWN_CLEAR or t.slope[t.index(x, y)] < 200:
            continue
        r = rng.uniform(0.1, 0.35)                     # horizontal radius, m
        size = [r, r * rng.uniform(0.6, 1.0), r * rng.uniform(0.4, 0.8)]
        yaw = rng.uniform(0, np.pi)
        z = t.height(x, y) - 0.3 * size[2]             # 30 % buried, like real rocks
        rocks.append({"x": round(x, 3), "y": round(y, 3), "z": round(z, 4),
                      "size": [round(s, 3) for s in size], "yaw": round(yaw, 3)})
    return rocks


def place_boulders(t, rng, count, existing_rocks=(), near_count=2):
    """Larger rocks (0.30-0.50 m exposed height) placed from a separate RNG stream.

    At least `near_count` boulders are placed within 3-10 m of spawn on drivable ground
    (outside the 3 m spawn clearance and clear of the +x straight-drive test corridor),
    with the first one in the forward-facing sector so 3D lidar sees it from spawn.
    """
    boulders = []
    placed_xyr = [(r["x"], r["y"], max(r["size"][0], r["size"][1])) for r in existing_rocks]

    def _valid(x, y, sx):
        if abs(x) > t.half - 2.5 or abs(y) > t.half - 2.5:
            return False
        if np.hypot(x, y) < SPAWN_CLEAR + sx:
            return False
        # Keep the +x straight-drive verification lane (0..5.5 m, |y| < 1.5 m) collision-free.
        if -0.8 <= x <= 5.5 and abs(y) < 1.5 + sx:
            return False
        if t.slope[t.index(x, y)] < 200:
            return False
        for ox, oy, orad in placed_xyr:
            if np.hypot(x - ox, y - oy) < sx + orad + 0.5:
                return False
        return True

    while len(boulders) < count:
        idx = len(boulders)
        exposed_h = float(rng.uniform(0.30, 0.50))
        sz = exposed_h / 0.7                           # 30 % buried => 0.7 * sz exposed above ground
        sx = sz * float(rng.uniform(0.9, 1.3))
        sy = sx * float(rng.uniform(0.65, 1.0))

        if idx == 0 and near_count >= 1:
            # First near boulder in the forward lidar sector (4.2-6.0 m, +/-50 deg)
            dist = float(rng.uniform(4.2, 6.0))
            ang = float(rng.uniform(-np.radians(50.0), np.radians(50.0)))
            x, y = dist * np.cos(ang), dist * np.sin(ang)
        elif idx < near_count:
            # Additional near-spawn boulder(s) within 3-10 m of spawn
            dist = float(rng.uniform(4.0, 9.0))
            ang = float(rng.uniform(-np.pi, np.pi))
            x, y = dist * np.cos(ang), dist * np.sin(ang)
        else:
            x, y = rng.uniform(-t.half + 2.5, t.half - 2.5, size=2)
            x, y = float(x), float(y)

        if not _valid(x, y, sx):
            continue

        yaw = float(rng.uniform(0.0, np.pi))
        z = float(t.height(x, y)) - 0.3 * sz
        boulders.append({
            "name": f"boulder_{idx}",
            "kind": "boulder",
            "x": round(x, 3),
            "y": round(y, 3),
            "z": round(z, 4),
            "size": [round(sx, 3), round(sy, 3), round(sz, 3)],
            "yaw": round(yaw, 3),
            "exposed_height": round(exposed_h, 3),
        })
        placed_xyr.append((x, y, sx))
    return boulders


def place_sand(t, rng, count, side=4.0):
    """Square sand patches on gentle ground, written as heightfield crops lifted by SAND_LIFT."""
    patches = []
    npx = int(round(side / (2 * t.half) * (t.n - 1))) + 1
    while len(patches) < count:
        x, y = rng.uniform(-t.half + side, t.half - side, size=2)
        r0, c0 = t.index(x - side / 2, y + side / 2)   # north-west corner
        crop = t.z[r0:r0 + npx, c0:c0 + npx]
        if np.hypot(x, y) < SPAWN_CLEAR + side or crop.shape != (npx, npx):
            continue
        if (t.slope[r0:r0 + npx, c0:c0 + npx] < 254).mean() > 0.1:  # mostly free ground
            continue
        k = len(patches)
        lo, hi = crop.min(), crop.max()
        span = max(hi - lo, 1e-3)
        write_hfield_bin(crop, TERRAIN / f"sand_{k}.bin")
        patches.append({"name": f"sand_{k}", "x": round(x, 3), "y": round(y, 3), "side": side,
                        "z_lo": round(lo + SAND_LIFT, 4), "z_span": round(span, 4)})
    return patches


def world_xml(t, rocks, sand, tiles):
    """Assemble the MJCF text."""
    m = t.meta
    z0 = spawn_ground_height(t)  # shift so highest of the 4 spawn wheel ground points is at z = 0
    assets = [f'    <hfield name="jezero" file="../terrain/heightmap.bin" '
              f'size="{t.half} {t.half} {m["z_range_m"]} 1"/>']
    for tile in tiles:
        assets.append(
            f'    <hfield name="{tile["name"]}" file="{tile["file"]}" '
            f'size="{tile["rx"]:.6f} {tile["ry"]:.6f} {tile["z_span"]:.6f} 1"/>'
        )
    geoms = [
        # Full textured heightfield kept for visuals only; collision and lidar rays use group-3 tiles.
        f'    <geom name="terrain" type="hfield" hfield="jezero" pos="0 0 {-z0:.6f}" '
        f'material="regolith" contype="0" conaffinity="0" group="2"/>'
    ]
    for tile in tiles:
        geoms.append(
            f'    <geom name="{tile["name"]}" type="hfield" hfield="{tile["name"]}" '
            f'pos="{tile["cx"]:.6f} {tile["cy"]:.6f} {tile["z_lo"] - z0:.6f}" '
            # group="3" hides tiles in default rendering; alpha=0.001 (>0) is required because MuJoCo mj_ray/mj_multiRay skips geoms with rgba[3]==0.
            f'group="3" rgba="0 0 0 0.001" friction="0.8 0.02 0.001"/>'
        )
    for s in sand:
        h = s["side"] / 2
        assets.append(f'    <hfield name="{s["name"]}" file="../terrain/{s["name"]}.bin" size="{h} {h} {s["z_span"]} 0.05"/>')
        geoms.append(f'    <geom name="{s["name"]}" type="hfield" hfield="{s["name"]}" '
                     f'pos="{s["x"]} {s["y"]} {s["z_lo"] - z0:.4f}" material="sand" friction="{SAND_FRICTION} 0.02 0.001"/>')
    for i, r in enumerate(rocks):
        name = r.get("name", f"rock_{i}")
        geoms.append(f'    <geom name="{name}" type="ellipsoid" size="{r["size"][0]} {r["size"][1]} {r["size"][2]}" '
                     f'pos="{r["x"]} {r["y"]} {r["z"] - z0:.4f}" euler="0 0 {r["yaw"]}" material="rock"/>')
    return f"""<!-- GENERATED by tools/make_mars_world.py from terrain/ (HiRISE {m["source_products"]["dtm"]}). Do not edit by hand. -->
<!-- HiRISE data: NASA/JPL-Caltech/UArizona. -->
<mujoco model="mars">
  <include file="../../mars_yard_description/mjcf/rover.xml"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -3.721"/>

  <visual>
    <headlight diffuse="0.5 0.5 0.5" ambient="0.25 0.2 0.18" specular="0 0 0"/>
    <map znear="0.01" zfar="200"/>
    <quality shadowsize="4096"/>
    <global azimuth="140" elevation="-25" offwidth="1920" offheight="1080"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.85 0.65 0.45" rgb2="0.3 0.2 0.15" width="512" height="3072"/>
    <texture name="ortho" type="2d" file="../terrain/texture.png"/>
    <!-- HiRISE is greyscale: rgba tints it Mars-red -->
    <material name="regolith" texture="ortho" rgba="1.0 0.62 0.42 1"/>
    <material name="sand" rgba="0.85 0.62 0.4 1"/>  <!-- untextured: each small hfield would stretch the whole ortho over itself -->
    <material name="rock" rgba="0.35 0.25 0.2 1"/>
{chr(10).join(assets)}
  </asset>

  <worldbody>
    <light pos="0 0 30" dir="-0.5 0.3 -1" directional="true" castshadow="true" diffuse="0.9 0.8 0.7"/>
{chr(10).join(geoms)}
  </worldbody>
</mujoco>
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--rocks", type=int, default=40)
    parser.add_argument("--boulders", type=int, default=8,
                        help="Number of larger boulders (0.30-0.50 m exposed height, 0 to disable)")
    parser.add_argument("--sand", type=int, default=3)
    args = parser.parse_args()

    t = Terrain()
    write_hfield_bin(t.z, TERRAIN / "heightmap.bin")
    tiles = build_collision_tiles(t, TILE_GRID)
    rng = np.random.default_rng(args.seed)
    rocks = place_rocks(t, rng, args.rocks)
    sand = place_sand(t, rng, args.sand)
    # Separate RNG stream so the 40 rocks and 3 sand patches keep their exact positions.
    boulder_rng = np.random.default_rng(args.seed + 10_000)
    boulders = place_boulders(t, boulder_rng, args.boulders, existing_rocks=rocks)
    all_rocks = rocks + boulders
    OUT.write_text(world_xml(t, all_rocks, sand, tiles))
    z0 = spawn_ground_height(t)
    (TERRAIN / "obstacles.json").write_text(json.dumps(
        {
            "seed": args.seed,
            "boulder_seed": args.seed + 10_000,
            "ground_offset_m": round(-z0, 4),
            "rocks": all_rocks,
            "sand": sand,
        }, indent=2) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}: {len(tiles)} collision tiles, {len(rocks)} rocks, "
          f"{len(boulders)} boulders, {len(sand)} sand patches, "
          f"spawn ground z offset {-z0:.4f} m, spawn slope pixel {t.slope[t.index(0, 0)]} (254 = free)")


if __name__ == "__main__":
    main()
