#!/usr/bin/env python3
"""Create 20 fixed benchmark scenarios on the Mars world (5 per terrain group).

Groups:
  - flat  : slope < 4 deg along the entire route corridor
  - slope : route crosses 8-18 deg ground, staying below the 20 deg lethal limit
  - rocks : straight line passes within 1.0 m of a boulder or hazard rock cluster,
            with a drivable bypass corridor around it
  - sand  : route crosses one of the low-friction sand patches from obstacles.json

All scenarios satisfy:
  - start-goal distance in [6.0, 15.0] m
  - start and goal inside [-22.5, +22.5] m with >= 1.3 m clearance from every
    rock/boulder surface and from any lethal slope cell (slope >= 20 deg)
  - start and goal local slope <= 14 deg so initial settling is gentle
  - fixed random seed for reproducibility

Usage:
  .venv/bin/python tools/make_scenarios.py [--seed 42] [--out src/mars_yard_benchmark/config/scenarios.yaml]
"""

import argparse
import json
import math
import pathlib
from typing import Dict, List, Optional, Tuple

import numpy as np
from PIL import Image
import yaml


ROOT = pathlib.Path(__file__).resolve().parents[1]
TERRAIN_DIR = ROOT / "src/mars_yard_worlds/terrain"
DEFAULT_OUT = ROOT / "src/mars_yard_benchmark/config/scenarios.yaml"


class MarsTerrainMap:
    def __init__(self, terrain_dir: pathlib.Path = TERRAIN_DIR) -> None:
        self.meta = json.loads((terrain_dir / "terrain.json").read_text(encoding="utf-8"))
        self.obs = json.loads((terrain_dir / "obstacles.json").read_text(encoding="utf-8"))
        png = np.asarray(Image.open(terrain_dir / "heightmap.png"), dtype=np.float64)
        self.z = png / 65535.0 * float(self.meta["z_range_m"])
        self.res = float(self.meta["res_m"])
        dz_dy, dz_dx = np.gradient(self.z, self.res, self.res)
        self.slope_deg = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))
        self.pgm = np.asarray(Image.open(terrain_dir / "slope.pgm"), dtype=np.uint8)
        self.n = self.z.shape[0]
        self.half = float(self.meta["size_x_m"]) / 2.0
        self.rocks = self.obs["rocks"]
        self.sand = self.obs["sand"]

        # Precompute rock arrays for fast clearance queries.
        self.rock_xy = np.array([[r["x"], r["y"]] for r in self.rocks], dtype=np.float64)
        self.rock_rad = np.array([max(r["size"][0], r["size"][1]) for r in self.rocks], dtype=np.float64)
        self.rock_exp_h = np.array([0.7 * r["size"][2] for r in self.rocks], dtype=np.float64)

        # Precompute lethal mask (slope >= 20 deg or pgm == 0)
        self.lethal = (self.pgm == 0) | (self.slope_deg >= 20.0)

    def world_to_rc(self, x: float, y: float) -> Tuple[int, int]:
        col = int(np.clip((x + self.half) / (2.0 * self.half) * (self.n - 1), 0, self.n - 1))
        row = int(np.clip((self.half - y) / (2.0 * self.half) * (self.n - 1), 0, self.n - 1))
        return row, col

    def in_bounds(self, x: float, y: float, margin: float = 2.5) -> bool:
        return abs(x) <= (self.half - margin) and abs(y) <= (self.half - margin)

    def slope_at(self, x: float, y: float) -> float:
        r, c = self.world_to_rc(x, y)
        return float(self.slope_deg[r, c])

    def patch_stats(self, x: float, y: float, radius: float) -> Tuple[float, bool, int]:
        """Return (max_slope_deg, any_lethal, min_pgm) inside square of half-size radius."""
        r0, c0 = self.world_to_rc(x - radius, y + radius)
        r1, c1 = self.world_to_rc(x + radius, y - radius)
        rmn, rmx = min(r0, r1), max(r0, r1)
        cmn, cmx = min(c0, c1), max(c0, c1)
        sl = self.slope_deg[rmn : rmx + 1, cmn : cmx + 1]
        lt = self.lethal[rmn : rmx + 1, cmn : cmx + 1]
        pg = self.pgm[rmn : rmx + 1, cmn : cmx + 1]
        return float(sl.max()), bool(np.any(lt)), int(pg.min())

    def min_rock_surface_dist(self, x: float, y: float, only_hazards: bool = False) -> float:
        d = np.hypot(self.rock_xy[:, 0] - x, self.rock_xy[:, 1] - y) - self.rock_rad
        if only_hazards:
            d = d[self.rock_exp_h >= 0.12]
        return float(d.min()) if d.size > 0 else math.inf

    def endpoint_valid(self, x: float, y: float, clearance: float = 1.3, max_local_slope: float = 14.0) -> bool:
        if not self.in_bounds(x, y, margin=2.5):
            return False
        if self.min_rock_surface_dist(x, y, only_hazards=False) < clearance:
            return False
        max_sl, any_lethal, min_pgm = self.patch_stats(x, y, clearance)
        if any_lethal or min_pgm <= 20:
            return False
        if self.slope_at(x, y) > max_local_slope or max_sl > 18.0:
            return False
        return True

    def sample_segment(self, x0: float, y0: float, x1: float, y1: float, step: float = 0.1) -> np.ndarray:
        dist = math.hypot(x1 - x0, y1 - y0)
        n_pts = max(2, int(math.ceil(dist / step)) + 1)
        ts = np.linspace(0.0, 1.0, n_pts)
        xs = x0 + ts * (x1 - x0)
        ys = y0 + ts * (y1 - y0)
        return np.column_stack((xs, ys))

    def corridor_max_slope_and_lethal(
        self, x0: float, y0: float, x1: float, y1: float, half_width: float = 0.8
    ) -> Tuple[float, float, bool]:
        """Return (max_slope_deg, p90_slope_deg, any_lethal) along a corridor of given half_width."""
        pts = self.sample_segment(x0, y0, x1, y1, step=0.15)
        dx, dy = x1 - x0, y1 - y0
        norm = math.hypot(dx, dy)
        nx, ny = -dy / norm, dx / norm
        offsets = np.linspace(-half_width, half_width, 5)
        slopes = []
        any_lethal = False
        for px, py in pts:
            for off in offsets:
                sx, sy = px + off * nx, py + off * ny
                if not self.in_bounds(sx, sy, margin=1.2):
                    return 99.0, 99.0, True
                r, c = self.world_to_rc(sx, sy)
                slopes.append(float(self.slope_deg[r, c]))
                if self.lethal[r, c]:
                    any_lethal = True
        arr = np.asarray(slopes, dtype=np.float64)
        return float(arr.max()), float(np.percentile(arr, 90)), any_lethal

    def segment_rock_distances(self, x0: float, y0: float, x1: float, y1: float) -> Tuple[np.ndarray, np.ndarray]:
        """Return per-rock (min_surface_dist_to_segment, min_center_dist_to_segment)."""
        pts = self.sample_segment(x0, y0, x1, y1, step=0.1)
        # pts: (P, 2), rock_xy: (R, 2)
        diff = pts[:, None, :] - self.rock_xy[None, :, :]
        d_ctr = np.hypot(diff[:, :, 0], diff[:, :, 1]).min(axis=0)
        d_surf = d_ctr - self.rock_rad
        return d_surf, d_ctr

    def sand_crossing_length(self, x0: float, y0: float, x1: float, y1: float) -> Tuple[float, Optional[str]]:
        pts = self.sample_segment(x0, y0, x1, y1, step=0.1)
        best_len = 0.0
        best_name = None
        for s in self.sand:
            sx, sy, h = float(s["x"]), float(s["y"]), 0.5 * float(s["side"])
            inside = (np.abs(pts[:, 0] - sx) <= h) & (np.abs(pts[:, 1] - sy) <= h)
            length = float(np.sum(inside)) * 0.1
            if length > best_len:
                best_len = length
                best_name = s["name"]
        return best_len, best_name


def _distinct(cand, existing, min_sep: float = 2.5) -> bool:
    sx, sy, gx, gy = cand["start"]["x"], cand["start"]["y"], cand["goal"]["x"], cand["goal"]["y"]
    for prev in existing:
        psx, psy = prev["start"]["x"], prev["start"]["y"]
        pgx, pgy = prev["goal"]["x"], prev["goal"]["y"]
        # Reject near-duplicate or reversed identical pairs.
        if (math.hypot(sx - psx, sy - psy) < min_sep and math.hypot(gx - pgx, gy - pgy) < min_sep) or (
            math.hypot(sx - pgx, sy - pgy) < min_sep and math.hypot(gx - psx, gy - psy) < min_sep
        ):
            return False
    return True


def _make_entry(
    sc_id: str,
    group: str,
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    yaw_start: Optional[float] = None,
    yaw_goal: Optional[float] = None,
    meta: Optional[Dict] = None,
) -> Dict:
    heading = math.atan2(y1 - y0, x1 - x0)
    ys = heading if yaw_start is None else yaw_start
    yg = heading if yaw_goal is None else yaw_goal
    entry = {
        "id": sc_id,
        "group": group,
        "start": {"x": round(float(x0), 2), "y": round(float(y0), 2), "yaw": round(float(ys), 3)},
        "goal": {"x": round(float(x1), 2), "y": round(float(y1), 2), "yaw": round(float(yg), 3)},
        "straight_dist": round(float(math.hypot(x1 - x0, y1 - y0)), 3),
    }
    if meta:
        entry["meta"] = meta
    return entry


def build_scenarios(seed: int = 42) -> List[Dict]:
    tm = MarsTerrainMap()
    rng = np.random.default_rng(seed)
    all_scenarios: List[Dict] = []

    # --- Group 1: flat (5 scenarios, slope < 4 deg along entire route corridor) ---
    flat_list: List[Dict] = []
    # Pre-sample valid flat points (< 4 deg)
    flat_pts = []
    for x in np.linspace(-21.5, 21.5, 87):
        for y in np.linspace(-21.5, 21.5, 87):
            if tm.endpoint_valid(x, y, clearance=1.35, max_local_slope=3.8):
                max_sl, _, _ = tm.patch_stats(x, y, 0.8)
                if max_sl < 4.0:
                    flat_pts.append((float(x), float(y)))
    rng.shuffle(flat_pts)

    for _ in range(50000):
        if len(flat_list) >= 5:
            break
        i, j = rng.integers(0, len(flat_pts), size=2)
        if i == j:
            continue
        x0, y0 = flat_pts[i]
        x1, y1 = flat_pts[j]
        dist = math.hypot(x1 - x0, y1 - y0)
        if not (6.5 <= dist <= 13.5):
            continue
        max_sl, p90_sl, any_lethal = tm.corridor_max_slope_and_lethal(x0, y0, x1, y1, half_width=0.7)
        if any_lethal or max_sl >= 4.0:
            continue
        d_surf, _ = tm.segment_rock_distances(x0, y0, x1, y1)
        if float(d_surf[tm.rock_exp_h >= 0.12].min()) < 1.2:
            continue
        sand_len, _ = tm.sand_crossing_length(x0, y0, x1, y1)
        if sand_len > 0.1:
            continue
        cand = _make_entry(
            f"flat_{len(flat_list) + 1:02d}",
            "flat",
            x0,
            y0,
            x1,
            y1,
            meta={"max_corridor_slope_deg": round(max_sl, 2), "p90_slope_deg": round(p90_sl, 2)},
        )
        if _distinct(cand, flat_list, min_sep=3.0):
            flat_list.append(cand)

    if len(flat_list) < 5:
        raise RuntimeError(f"Only found {len(flat_list)}/5 flat scenarios.")
    all_scenarios.extend(flat_list)

    # --- Group 2: slope (5 scenarios, route crosses 8-18 deg ground, below 20 deg lethal limit) ---
    slope_list: List[Dict] = []
    valid_pts = []
    for x in np.linspace(-21.5, 21.5, 87):
        for y in np.linspace(-21.5, 21.5, 87):
            if tm.endpoint_valid(x, y, clearance=1.4, max_local_slope=13.5):
                valid_pts.append((float(x), float(y)))
    rng.shuffle(valid_pts)

    for _ in range(80000):
        if len(slope_list) >= 5:
            break
        i, j = rng.integers(0, len(valid_pts), size=2)
        if i == j:
            continue
        x0, y0 = valid_pts[i]
        x1, y1 = valid_pts[j]
        dist = math.hypot(x1 - x0, y1 - y0)
        if not (7.0 <= dist <= 14.0):
            continue
        max_sl, p90_sl, any_lethal = tm.corridor_max_slope_and_lethal(x0, y0, x1, y1, half_width=0.9)
        if any_lethal or max_sl >= 18.0 or max_sl < 10.5 or p90_sl < 8.5:
            continue
        # Ensure substantial fraction of route is in 8-18 deg
        pts = tm.sample_segment(x0, y0, x1, y1, step=0.2)
        seg_sl = np.array([tm.slope_at(px, py) for px, py in pts])
        frac_slope = float(np.mean((seg_sl >= 8.0) & (seg_sl <= 18.0)))
        if frac_slope < 0.35:
            continue
        d_surf, _ = tm.segment_rock_distances(x0, y0, x1, y1)
        if float(d_surf[tm.rock_exp_h >= 0.12].min()) < 1.2:
            continue
        sand_len, _ = tm.sand_crossing_length(x0, y0, x1, y1)
        if sand_len > 0.1:
            continue
        cand = _make_entry(
            f"slope_{len(slope_list) + 1:02d}",
            "slope",
            x0,
            y0,
            x1,
            y1,
            meta={
                "max_corridor_slope_deg": round(max_sl, 2),
                "p90_slope_deg": round(p90_sl, 2),
                "frac_8_18_deg": round(frac_slope, 2),
            },
        )
        if _distinct(cand, slope_list, min_sep=4.0):
            slope_list.append(cand)

    if len(slope_list) < 5:
        raise RuntimeError(f"Only found {len(slope_list)}/5 slope scenarios.")
    all_scenarios.extend(slope_list)

    # --- Group 3: rocks (5 scenarios, straight line passes within 1.0 m of a boulder / rock cluster) ---
    # Include the reference boulder corridor (boulder_0 + boulder_6) and 4 other distinct boulders/clusters.
    rocks_list: List[Dict] = []
    # Anchor boulders (sorted cleanly, skipping boulders right next to map borders or steep cliffs)
    boulders = [r for r in tm.rocks if r.get("kind") == "boulder"]
    # Candidate rock targets: boulders first, plus prominent rocks (exp_h >= 0.13 m)
    prominent_rocks = [
        (r.get("name", f"rock_{idx}"), r)
        for idx, r in enumerate(tm.rocks)
        if r.get("kind") == "boulder" or (0.7 * r["size"][2] >= 0.14)
    ]

    # Try generating a clean crossing across each distinct boulder/cluster with guaranteed drivable bypass.
    used_targets = set()
    # First scenario: classic Phase 5b corridor from (0, 0) -> (9.0, -0.6) passing boulder_0 and boulder_6.
    if tm.endpoint_valid(0.0, 0.0, clearance=1.3) and tm.endpoint_valid(9.0, -0.6, clearance=1.3):
        rocks_list.append(
            _make_entry(
                "rocks_01",
                "rocks",
                0.0,
                0.0,
                9.0,
                -0.6,
                yaw_start=0.0,
                yaw_goal=0.0,
                meta={"target": "boulder_0,boulder_6", "min_straight_rock_dist_m": 0.0},
            )
        )
        used_targets.add("boulder_0")
        used_targets.add("boulder_6")

    rng.shuffle(prominent_rocks)
    for name, rock in prominent_rocks:
        if len(rocks_list) >= 5:
            break
        if name in used_targets:
            continue
        rx, ry = float(rock["x"]), float(rock["y"])
        rad = float(max(rock["size"][0], rock["size"][1]))
        # Try random headings passing within 0.0 - 0.6 m of the rock surface, with bypass clearance on at least one side.
        placed = False
        for _ in range(400):
            ang = float(rng.uniform(-math.pi, math.pi))
            d_before = float(rng.uniform(3.5, 6.0))
            d_after = float(rng.uniform(3.5, 6.0))
            lat_off = float(rng.uniform(-0.4, 0.4))
            ca, sa = math.cos(ang), math.sin(ang)
            nx, ny = -sa, ca
            x0 = rx - d_before * ca + lat_off * nx
            y0 = ry - d_before * sa + lat_off * ny
            x1 = rx + d_after * ca + lat_off * nx
            y1 = ry + d_after * sa + lat_off * ny
            dist = math.hypot(x1 - x0, y1 - y0)
            if not (7.0 <= dist <= 13.5):
                continue
            if not (tm.endpoint_valid(x0, y0, clearance=1.4, max_local_slope=12.0)
                    and tm.endpoint_valid(x1, y1, clearance=1.4, max_local_slope=12.0)):
                continue
            # Check straight line passes within 1.0 m of this rock surface
            d_surf, _ = tm.segment_rock_distances(x0, y0, x1, y1)
            # Ensure wide bypass corridor (2.2 m on at least one side of the rock) is free of lethal slopes & other boulders
            max_sl, _, any_lethal = tm.corridor_max_slope_and_lethal(x0, y0, x1, y1, half_width=1.5)
            if any_lethal or max_sl >= 17.5:
                continue
            bypass_ok = False
            for side in (-1.0, 1.0):
                bx = rx + side * (rad + 1.45) * nx
                by = ry + side * (rad + 1.45) * ny
                if tm.endpoint_valid(bx, by, clearance=0.9, max_local_slope=15.0):
                    bypass_ok = True
                    break
            if not bypass_ok:
                continue
            # Ensure target rock is genuinely close to the straight line (< 0.8 m surface dist)
            pts = tm.sample_segment(x0, y0, x1, y1, step=0.1)
            target_surf = float(np.hypot(pts[:, 0] - rx, pts[:, 1] - ry).min()) - rad
            if target_surf > 0.8:
                continue
            cand = _make_entry(
                f"rocks_{len(rocks_list) + 1:02d}",
                "rocks",
                x0,
                y0,
                x1,
                y1,
                meta={
                    "target": name,
                    "min_straight_rock_dist_m": round(max(0.0, target_surf), 3),
                    "max_corridor_slope_deg": round(max_sl, 2),
                },
            )
            if _distinct(cand, rocks_list, min_sep=4.0):
                rocks_list.append(cand)
                used_targets.add(name)
                placed = True
                break
        if not placed:
            continue

    if len(rocks_list) < 5:
        raise RuntimeError(f"Only found {len(rocks_list)}/5 rocks scenarios.")
    all_scenarios.extend(rocks_list)

    # --- Group 4: sand (5 scenarios, route crosses a sand patch from obstacles.json) ---
    sand_list: List[Dict] = []
    # Spread across all 3 sand patches: target up to 2 per patch (total 5).
    patch_counts = {s["name"]: 0 for s in tm.sand}
    for attempt in range(40000):
        if len(sand_list) >= 5:
            break
        s = tm.sand[attempt % len(tm.sand)]
        if patch_counts[s["name"]] >= 2:
            continue
        sx, sy = float(s["x"]), float(s["y"])
        ang = float(rng.uniform(-math.pi, math.pi))
        d_before = float(rng.uniform(3.2, 6.5))
        d_after = float(rng.uniform(3.2, 6.5))
        lat_off = float(rng.uniform(-0.8, 0.8))
        ca, sa = math.cos(ang), math.sin(ang)
        nx, ny = -sa, ca
        x0 = sx - d_before * ca + lat_off * nx
        y0 = sy - d_before * sa + lat_off * ny
        x1 = sx + d_after * ca + lat_off * nx
        y1 = sy + d_after * sa + lat_off * ny
        dist = math.hypot(x1 - x0, y1 - y0)
        if not (6.5 <= dist <= 14.0):
            continue
        if not (tm.endpoint_valid(x0, y0, clearance=1.4, max_local_slope=12.0)
                and tm.endpoint_valid(x1, y1, clearance=1.4, max_local_slope=12.0)):
            continue
        # Ensure start and goal are outside the sand patch so rover enters & crosses it
        h = 0.5 * float(s["side"])
        if (abs(x0 - sx) <= h and abs(y0 - sy) <= h) or (abs(x1 - sx) <= h and abs(y1 - sy) <= h):
            continue
        sand_len, hit_name = tm.sand_crossing_length(x0, y0, x1, y1)
        if hit_name != s["name"] or sand_len < 2.5:
            continue
        max_sl, _, any_lethal = tm.corridor_max_slope_and_lethal(x0, y0, x1, y1, half_width=0.9)
        if any_lethal or max_sl >= 17.5:
            continue
        d_surf, _ = tm.segment_rock_distances(x0, y0, x1, y1)
        if float(d_surf[tm.rock_exp_h >= 0.12].min()) < 1.0:
            continue
        cand = _make_entry(
            f"sand_{len(sand_list) + 1:02d}",
            "sand",
            x0,
            y0,
            x1,
            y1,
            meta={
                "sand_patch": s["name"],
                "sand_crossing_m": round(sand_len, 2),
                "max_corridor_slope_deg": round(max_sl, 2),
            },
        )
        if _distinct(cand, sand_list, min_sep=3.0):
            sand_list.append(cand)
            patch_counts[s["name"]] += 1

    if len(sand_list) < 5:
        raise RuntimeError(f"Only found {len(sand_list)}/5 sand scenarios (counts={patch_counts}).")
    all_scenarios.extend(sand_list)

    return all_scenarios


def main() -> None:
    parser = argparse.ArgumentParser(description="Create 20 fixed benchmark scenarios for the Mars world.")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--out", type=pathlib.Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    scenarios = build_scenarios(seed=args.seed)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "world": "mars",
        "seed": args.seed,
        "count": len(scenarios),
        "scenarios": scenarios,
    }
    args.out.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"Wrote {len(scenarios)} scenarios to {args.out}:")
    for sc in scenarios:
        s, g = sc["start"], sc["goal"]
        print(
            f"  {sc['id']:8s} [{sc['group']:5s}] ({s['x']:+6.2f},{s['y']:+6.2f},yaw={math.degrees(s['yaw']):+6.1f}deg) "
            f"-> ({g['x']:+6.2f},{g['y']:+6.2f}) dist={sc['straight_dist']:5.2f}m meta={sc.get('meta', {})}"
        )


if __name__ == "__main__":
    main()
