"""Generate src/mars_yard_worlds/worlds/mars.xml from the terrain assets (Phase 2).

    .venv/bin/python tools/make_mars_world.py [--seed 7] [--rocks 40] [--sand 3]

Inputs, from tools/dem_to_world.py, in src/mars_yard_worlds/terrain/:
heightmap.png, texture.png, terrain.json and slope.pgm.

It writes:
- mars.xml, with the Jezero heightfield, rocks and low-friction sand patches. The terrain is
  shifted so that the ground under the rover spawn (x = y = 0) is at z = 0.
- heightmap.bin and sand_<k>.bin, float32 heightfields for MuJoCo (see write_hfield_bin).
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
OUT = ROOT / "src/mars_yard_worlds/worlds/mars.xml"

SAND_LIFT = 0.005      # m, sand surface sits this far above the main terrain
SAND_FRICTION = 0.35   # vs 0.8 on regolith: loose sand, wheels slip
SPAWN_CLEAR = 3.0      # m, no rocks or sand this close to the spawn point


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


def world_xml(t, rocks, sand):
    """Assemble the MJCF text."""
    m = t.meta
    z0 = t.height(0.0, 0.0)  # shift so the ground under the spawn is z = 0
    assets = [f'    <hfield name="jezero" file="../terrain/heightmap.bin" '
              f'size="{t.half} {t.half} {m["z_range_m"]} 1"/>']
    geoms = [f'    <geom name="terrain" type="hfield" hfield="jezero" pos="0 0 {-z0:.4f}" material="regolith" '
             f'friction="0.8 0.02 0.001"/>']
    for s in sand:
        h = s["side"] / 2
        assets.append(f'    <hfield name="{s["name"]}" file="../terrain/{s["name"]}.bin" size="{h} {h} {s["z_span"]} 0.05"/>')
        geoms.append(f'    <geom name="{s["name"]}" type="hfield" hfield="{s["name"]}" '
                     f'pos="{s["x"]} {s["y"]} {s["z_lo"] - z0:.4f}" material="sand" friction="{SAND_FRICTION} 0.02 0.001"/>')
    for i, r in enumerate(rocks):
        geoms.append(f'    <geom name="rock_{i}" type="ellipsoid" size="{r["size"][0]} {r["size"][1]} {r["size"][2]}" '
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
    parser.add_argument("--sand", type=int, default=3)
    args = parser.parse_args()

    t = Terrain()
    write_hfield_bin(t.z, TERRAIN / "heightmap.bin")
    rng = np.random.default_rng(args.seed)
    rocks = place_rocks(t, rng, args.rocks)
    sand = place_sand(t, rng, args.sand)
    OUT.write_text(world_xml(t, rocks, sand))
    z0 = t.height(0.0, 0.0)
    (TERRAIN / "obstacles.json").write_text(json.dumps(
        {"seed": args.seed, "ground_offset_m": round(-z0, 4), "rocks": rocks, "sand": sand}, indent=2) + "\n")
    print(f"wrote {OUT.relative_to(ROOT)}: {len(rocks)} rocks, {len(sand)} sand patches, "
          f"spawn ground z offset {-z0:.3f} m, spawn slope pixel {t.slope[t.index(0, 0)]} (254 = free)")


if __name__ == "__main__":
    main()
