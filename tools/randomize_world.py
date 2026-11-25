#!/usr/bin/env python3
"""Domain randomization generator for Mars Yard MuJoCo worlds (Phase 7).

Levels 0-3 sample physical parameters uniformly in +/-range with a fixed seed and
write a self-contained randomized copy of the world MJCF + rover MJCF plus params.json.
Level 0 returns the original world MJCF path unchanged (bit-for-bit nominal stack).

Usage:
  python3 tools/randomize_world.py --world mars --level 2 --seed 2 --out /tmp/rand_L2
"""

import argparse
import json
import pathlib
import sys
import xml.etree.ElementTree as ET
from typing import Dict, Optional, Tuple

import numpy as np


WHEEL_ACTUATORS = (
    "wheel_fl_joint",
    "wheel_fr_joint",
    "wheel_rl_joint",
    "wheel_rr_joint",
)

# Nominal rover physical constants (must match src/mars_yard_description/mjcf/rover.xml)
NOMINAL_CHASSIS_MASS_KG = 35.5
NOMINAL_CHASSIS_SIZE_M = (0.8, 0.5, 0.2)  # full box lengths (x, y, z)
NOMINAL_OTHER_MASS_KG = 4.5  # 4 wheels x 1.0 kg + front_axle_link 0.5 kg
NOMINAL_WHEEL_SLIDING_FRICTION = 0.8
NOMINAL_ACTUATOR_KV = 50.0
NOMINAL_ACTUATOR_FORCE_LIMIT = 15.0
NOMINAL_ACTUATOR_CTRL_LIMIT = 6.0

LEVEL_SPECS: Dict[int, Dict[str, float]] = {
    0: {
        "friction_range": 0.0,
        "mass_range": 0.0,
        "com_offset_m": 0.0,
        "motor_gain_range": 0.0,
        "lidar_noise_sigma_m": 0.0,
        "lidar_dropout": 0.0,
        "imu_gyro_sigma_rad_s": 0.0,
        "imu_gyro_z_bias_rad_s": 0.0,
        "cmd_latency_s": 0.0,
    },
    1: {
        "friction_range": 0.10,
        "mass_range": 0.05,
        "com_offset_m": 0.01,
        "motor_gain_range": 0.05,
        "lidar_noise_sigma_m": 0.01,
        "lidar_dropout": 0.01,
        "imu_gyro_sigma_rad_s": 0.002,
        "imu_gyro_z_bias_rad_s": 0.0,
        "cmd_latency_s": 0.02,
    },
    2: {
        "friction_range": 0.20,
        "mass_range": 0.10,
        "com_offset_m": 0.02,
        "motor_gain_range": 0.10,
        "lidar_noise_sigma_m": 0.02,
        "lidar_dropout": 0.05,
        "imu_gyro_sigma_rad_s": 0.005,
        "imu_gyro_z_bias_rad_s": 0.002,
        "cmd_latency_s": 0.05,
    },
    3: {
        "friction_range": 0.35,
        "mass_range": 0.20,
        "com_offset_m": 0.04,
        "motor_gain_range": 0.20,
        "lidar_noise_sigma_m": 0.04,
        "lidar_dropout": 0.15,
        "imu_gyro_sigma_rad_s": 0.01,
        "imu_gyro_z_bias_rad_s": 0.005,
        "cmd_latency_s": 0.10,
    },
}


def _default_repo_root() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1]


def _resolve_package_dirs(
    worlds_dir: Optional[pathlib.Path] = None,
    description_dir: Optional[pathlib.Path] = None,
) -> Tuple[pathlib.Path, pathlib.Path]:
    repo = _default_repo_root()
    if worlds_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory

            worlds_dir = pathlib.Path(get_package_share_directory("mars_yard_worlds"))
        except Exception:
            worlds_dir = repo / "src" / "mars_yard_worlds"
    if description_dir is None:
        try:
            from ament_index_python.packages import get_package_share_directory

            description_dir = pathlib.Path(get_package_share_directory("mars_yard_description"))
        except Exception:
            description_dir = repo / "src" / "mars_yard_description"
    return pathlib.Path(worlds_dir).resolve(), pathlib.Path(description_dir).resolve()


def sample_params(world: str, level: int, seed: int) -> Dict:
    """Deterministically sample domain randomization parameters for (level, seed)."""
    if level not in LEVEL_SPECS:
        raise ValueError(f"Unsupported randomization level {level}. Expected one of {sorted(LEVEL_SPECS)}.")
    if world not in ("flat", "mars"):
        raise ValueError(f"Unsupported world '{world}'. Expected 'flat' or 'mars'.")

    spec = LEVEL_SPECS[level]
    if level == 0:
        friction_scale = 1.0
        chassis_mass_scale = 1.0
        com_x = 0.0
        com_y = 0.0
        motor_gains = {w: 1.0 for w in WHEEL_ACTUATORS}
    else:
        rng = np.random.default_rng(int(seed))
        r_f = spec["friction_range"]
        r_m = spec["mass_range"]
        r_c = spec["com_offset_m"]
        r_g = spec["motor_gain_range"]

        friction_scale = float(rng.uniform(1.0 - r_f, 1.0 + r_f))
        chassis_mass_scale = float(rng.uniform(1.0 - r_m, 1.0 + r_m))
        com_x = float(rng.uniform(-r_c, r_c))
        com_y = float(rng.uniform(-r_c, r_c))
        motor_gains = {
            w: float(rng.uniform(1.0 - r_g, 1.0 + r_g)) for w in WHEEL_ACTUATORS
        }

    chassis_mass_kg = NOMINAL_CHASSIS_MASS_KG * chassis_mass_scale
    return {
        "world": world,
        "level": int(level),
        "seed": int(seed),
        "friction_scale": friction_scale,
        "wheel_sliding_friction": NOMINAL_WHEEL_SLIDING_FRICTION * friction_scale,
        "chassis_mass_scale": chassis_mass_scale,
        "chassis_mass_kg": chassis_mass_kg,
        "total_mass_kg": chassis_mass_kg + NOMINAL_OTHER_MASS_KG,
        "chassis_com_offset_x_m": com_x,
        "chassis_com_offset_y_m": com_y,
        "motor_gain_scale": motor_gains,
        "actuator_knob": "general_affine_gainprm_forcerange",
        "lidar_noise_sigma_m": float(spec["lidar_noise_sigma_m"]),
        "lidar_dropout": float(spec["lidar_dropout"]),
        "imu_gyro_sigma_rad_s": float(spec["imu_gyro_sigma_rad_s"]),
        "imu_gyro_z_bias_rad_s": float(spec["imu_gyro_z_bias_rad_s"]),
        "cmd_latency_s": float(spec["cmd_latency_s"]),
        "lidar_relay_seed": int(seed) * 1000 + int(level) * 10 + 1,
        "imu_relay_seed": int(seed) * 1000 + int(level) * 10 + 2,
    }


def _scale_friction_attr(fric_str: str, scale: float) -> str:
    parts = fric_str.split()
    if not parts:
        return fric_str
    parts[0] = f"{float(parts[0]) * scale:.6f}"
    return " ".join(parts)


def _write_randomized_rover(
    src_rover_xml: pathlib.Path,
    dst_rover_xml: pathlib.Path,
    params: Dict,
) -> None:
    tree = ET.parse(src_rover_xml)
    root = tree.getroot()

    friction_scale = params["friction_scale"]
    chassis_mass = params["chassis_mass_kg"]
    com_x = params["chassis_com_offset_x_m"]
    com_y = params["chassis_com_offset_y_m"]
    motor_gains = params["motor_gain_scale"]

    # 1. Scale sliding friction on wheel default class (and any explicit wheel geom friction).
    wheel_class_found = False
    for default_el in root.iter("default"):
        if default_el.attrib.get("class") == "wheel":
            for geom in default_el.findall("geom"):
                if "friction" in geom.attrib:
                    geom.attrib["friction"] = _scale_friction_attr(
                        geom.attrib["friction"], friction_scale
                    )
                    wheel_class_found = True
    if not wheel_class_found:
        raise RuntimeError(f"Could not find wheel default class friction in {src_rover_xml}")

    # 2. Scale chassis mass and shift chassis inertial COM (x, y) without moving collision box.
    base_body = None
    for body in root.iter("body"):
        if body.attrib.get("name") == "base_link":
            base_body = body
            break
    if base_body is None:
        raise RuntimeError(f"Could not find base_link body in {src_rover_xml}")

    chassis_geom = None
    for geom in base_body.findall("geom"):
        if geom.attrib.get("name") == "chassis":
            chassis_geom = geom
            break
    if chassis_geom is None:
        raise RuntimeError(f"Could not find chassis geom in {src_rover_xml}")
    chassis_geom.attrib["mass"] = f"{chassis_mass:.6f}"

    lx, ly, lz = NOMINAL_CHASSIS_SIZE_M
    ixx = chassis_mass * (ly * ly + lz * lz) / 12.0
    iyy = chassis_mass * (lx * lx + lz * lz) / 12.0
    izz = chassis_mass * (lx * lx + ly * ly) / 12.0

    # Remove any pre-existing <inertial> on base_link and insert our randomized one.
    for old_inertial in list(base_body.findall("inertial")):
        base_body.remove(old_inertial)
    inertial = ET.Element(
        "inertial",
        {
            "pos": f"{com_x:.6f} {com_y:.6f} 0",
            "mass": f"{chassis_mass:.6f}",
            "diaginertia": f"{ixx:.6f} {iyy:.6f} {izz:.6f}",
        },
    )
    # Place after <freejoint> for clean XML structure.
    insert_idx = 0
    for idx, child in enumerate(list(base_body)):
        if child.tag == "freejoint":
            insert_idx = idx + 1
            break
    base_body.insert(insert_idx, inertial)

    # 3. Replace each wheel <velocity> actuator with an affine <general> velocity servo
    #    tau = clip(kv * (g_i * ctrl - qvel), [-f_lim * g_i, +f_lim * g_i]).
    #    At g_i = 1.0 this compiles identically to <velocity kv="50" forcerange="-15 15"/>.
    actuator_el = root.find("actuator")
    if actuator_el is None:
        raise RuntimeError(f"Could not find <actuator> block in {src_rover_xml}")

    new_children = []
    seen_wheels = set()
    for child in list(actuator_el):
        name = child.attrib.get("name", "")
        if name in motor_gains:
            g = float(motor_gains[name])
            kv = float(child.attrib.get("kv", NOMINAL_ACTUATOR_KV))
            ctrl_lim = NOMINAL_ACTUATOR_CTRL_LIMIT
            force_lim = NOMINAL_ACTUATOR_FORCE_LIMIT * g
            gen = ET.Element(
                "general",
                {
                    "name": name,
                    "joint": child.attrib.get("joint", name),
                    "dyntype": "none",
                    "gaintype": "fixed",
                    "biastype": "affine",
                    "gainprm": f"{kv * g:.6f} 0 0",
                    "biasprm": f"0 0 {-kv:.6f}",
                    "ctrllimited": "true",
                    "ctrlrange": f"{-ctrl_lim:.6f} {ctrl_lim:.6f}",
                    "forcelimited": "true",
                    "forcerange": f"{-force_lim:.6f} {force_lim:.6f}",
                },
            )
            new_children.append(gen)
            seen_wheels.add(name)
        else:
            new_children.append(child)

    if seen_wheels != set(WHEEL_ACTUATORS):
        raise RuntimeError(
            f"Missing wheel actuators in {src_rover_xml}: expected {WHEEL_ACTUATORS}, found {sorted(seen_wheels)}"
        )

    for child in list(actuator_el):
        actuator_el.remove(child)
    for child in new_children:
        actuator_el.append(child)

    ET.indent(tree, space="  ")
    tree.write(dst_rover_xml, encoding="utf-8", xml_declaration=False)


def _write_randomized_world(
    src_world_xml: pathlib.Path,
    dst_world_xml: pathlib.Path,
    dst_rover_name: str,
    params: Dict,
) -> None:
    tree = ET.parse(src_world_xml)
    root = tree.getroot()
    src_dir = src_world_xml.parent.resolve()
    friction_scale = params["friction_scale"]

    # 1. Point <include> to our randomized rover.xml in the same output directory.
    include_updated = False
    for inc in root.findall("include"):
        inc_file = inc.attrib.get("file", "")
        if "rover.xml" in inc_file:
            inc.attrib["file"] = dst_rover_name
            include_updated = True
    if not include_updated:
        raise RuntimeError(f"Could not find rover.xml <include> in {src_world_xml}")

    # 2. Resolve relative asset paths (textures, heightfields) to absolute paths.
    asset_el = root.find("asset")
    if asset_el is not None:
        for child in asset_el:
            if "file" in child.attrib:
                rel = child.attrib["file"]
                abs_path = (src_dir / rel).resolve()
                child.attrib["file"] = str(abs_path)

    # 3. Scale sliding friction on terrain collision geoms (terrain_tile_*, sand_*, floor)
    #    by the same friction_scale. MuJoCo combines equal-priority geom friction via
    #    element-wise max(), so scaling only the wheel geom would mask any friction_scale < 1.0.
    worldbody = root.find("worldbody")
    if worldbody is not None:
        for geom in worldbody.iter("geom"):
            if "friction" in geom.attrib:
                geom.attrib["friction"] = _scale_friction_attr(
                    geom.attrib["friction"], friction_scale
                )

    ET.indent(tree, space="  ")
    tree.write(dst_world_xml, encoding="utf-8", xml_declaration=False)


def randomize_world(
    world: str,
    level: int,
    seed: int,
    out_dir: Optional[pathlib.Path] = None,
    worlds_dir: Optional[pathlib.Path] = None,
    description_dir: Optional[pathlib.Path] = None,
) -> Tuple[pathlib.Path, Dict]:
    """Generate randomized world MJCF + params.json.

    At level 0, returns the original world XML path untouched (and optionally writes
    params.json to out_dir if out_dir is provided).
    """
    worlds_root, desc_root = _resolve_package_dirs(worlds_dir, description_dir)
    src_world_xml = (worlds_root / "worlds" / f"{world}.xml").resolve()
    src_rover_xml = (desc_root / "mjcf" / "rover.xml").resolve()

    if not src_world_xml.exists():
        raise FileNotFoundError(f"Source world XML not found: {src_world_xml}")
    if not src_rover_xml.exists():
        raise FileNotFoundError(f"Source rover XML not found: {src_rover_xml}")

    params = sample_params(world=world, level=level, seed=seed)

    if level == 0:
        params["world_xml"] = str(src_world_xml)
        params["rover_xml"] = str(src_rover_xml)
        if out_dir is not None:
            out_path = pathlib.Path(out_dir).resolve()
            out_path.mkdir(parents=True, exist_ok=True)
            (out_path / "params.json").write_text(
                json.dumps(params, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
        return src_world_xml, params

    if out_dir is None:
        raise ValueError("out_dir is required when level > 0.")

    out_path = pathlib.Path(out_dir).resolve()
    out_path.mkdir(parents=True, exist_ok=True)

    dst_rover_xml = out_path / "rover.xml"
    dst_world_xml = out_path / f"{world}.xml"

    _write_randomized_rover(src_rover_xml, dst_rover_xml, params)
    _write_randomized_world(src_world_xml, dst_world_xml, dst_rover_xml.name, params)

    params["world_xml"] = str(dst_world_xml)
    params["rover_xml"] = str(dst_rover_xml)
    (out_path / "params.json").write_text(
        json.dumps(params, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return dst_world_xml, params


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Write a domain-randomized copy of a Mars Yard MuJoCo world + params.json."
    )
    parser.add_argument("--world", default="mars", choices=("flat", "mars"))
    parser.add_argument("--level", type=int, required=True, choices=(0, 1, 2, 3))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--out",
        type=pathlib.Path,
        default=None,
        help="Output directory for randomized XMLs and params.json (required for level > 0).",
    )
    parser.add_argument("--worlds-dir", type=pathlib.Path, default=None)
    parser.add_argument("--description-dir", type=pathlib.Path, default=None)
    args = parser.parse_args(argv)

    world_xml, _params = randomize_world(
        world=args.world,
        level=args.level,
        seed=args.seed,
        out_dir=args.out,
        worlds_dir=args.worlds_dir,
        description_dir=args.description_dir,
    )
    print(str(world_xml))
    return 0


if __name__ == "__main__":
    sys.exit(main())
