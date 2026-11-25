"""Unit tests for Phase 7 domain randomization, sensor noise math, and robustness analysis."""

import csv
import importlib.util
import json
import math
import pathlib
import sys

import mujoco
import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "src/mars_yard_benchmark"))

import randomize_world as rw
from mars_yard_benchmark.robustness_analysis import analyze_robustness


def _load_noise_relay_funcs():
    path = REPO / "src/mars_yard_bringup/scripts/noise_relay.py"
    spec = importlib.util.spec_from_file_location("noise_relay_mod", str(path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.apply_lidar_noise, mod.apply_gyro_noise


def test_randomize_world_level0_identity(tmp_path: pathlib.Path):
    worlds_dir = REPO / "src/mars_yard_worlds"
    desc_dir = REPO / "src/mars_yard_description"
    expected_mars = (worlds_dir / "worlds/mars.xml").resolve()

    out_path, params = rw.randomize_world(
        world="mars",
        level=0,
        seed=0,
        out_dir=tmp_path / "l0",
        worlds_dir=worlds_dir,
        description_dir=desc_dir,
    )
    assert out_path == expected_mars
    assert params["level"] == 0
    assert params["friction_scale"] == 1.0
    assert params["chassis_mass_scale"] == 1.0
    assert params["chassis_com_offset_x_m"] == 0.0
    assert params["chassis_com_offset_y_m"] == 0.0
    assert all(v == 1.0 for v in params["motor_gain_scale"].values())
    assert params["lidar_noise_sigma_m"] == 0.0
    assert params["lidar_dropout"] == 0.0
    assert params["imu_gyro_sigma_rad_s"] == 0.0
    assert params["imu_gyro_z_bias_rad_s"] == 0.0
    assert params["cmd_latency_s"] == 0.0
    # L0 must NOT write a modified world or rover XML into out_dir
    assert not (tmp_path / "l0" / "mars.xml").exists()
    assert not (tmp_path / "l0" / "rover.xml").exists()


def test_randomize_world_bounds_reproducibility_and_mujoco_compile(tmp_path: pathlib.Path):
    worlds_dir = REPO / "src/mars_yard_worlds"
    desc_dir = REPO / "src/mars_yard_description"

    for level in (1, 2, 3):
        spec = rw.LEVEL_SPECS[level]
        out_a = tmp_path / f"L{level}_a"
        out_b = tmp_path / f"L{level}_b"
        out_c = tmp_path / f"L{level}_diff_seed"

        xml_a, params_a = rw.randomize_world(
            "mars", level, seed=level, out_dir=out_a, worlds_dir=worlds_dir, description_dir=desc_dir
        )
        xml_b, params_b = rw.randomize_world(
            "mars", level, seed=level, out_dir=out_b, worlds_dir=worlds_dir, description_dir=desc_dir
        )
        _xml_c, params_c = rw.randomize_world(
            "mars", level, seed=level + 99, out_dir=out_c, worlds_dir=worlds_dir, description_dir=desc_dir
        )

        # Bounds check
        assert 1.0 - spec["friction_range"] <= params_a["friction_scale"] <= 1.0 + spec["friction_range"]
        assert 1.0 - spec["mass_range"] <= params_a["chassis_mass_scale"] <= 1.0 + spec["mass_range"]
        assert -spec["com_offset_m"] <= params_a["chassis_com_offset_x_m"] <= spec["com_offset_m"]
        assert -spec["com_offset_m"] <= params_a["chassis_com_offset_y_m"] <= spec["com_offset_m"]
        for w, g in params_a["motor_gain_scale"].items():
            assert 1.0 - spec["motor_gain_range"] <= g <= 1.0 + spec["motor_gain_range"], f"{w}: {g}"

        assert params_a["lidar_noise_sigma_m"] == spec["lidar_noise_sigma_m"]
        assert params_a["lidar_dropout"] == spec["lidar_dropout"]
        assert params_a["imu_gyro_sigma_rad_s"] == spec["imu_gyro_sigma_rad_s"]
        assert params_a["imu_gyro_z_bias_rad_s"] == spec["imu_gyro_z_bias_rad_s"]
        assert params_a["cmd_latency_s"] == spec["cmd_latency_s"]

        # Same seed -> identical generated MJCFs and sampled parameters (ignoring output dir path)
        assert (out_a / "mars.xml").read_text(encoding="utf-8") == (out_b / "mars.xml").read_text(encoding="utf-8")
        assert (out_a / "rover.xml").read_text(encoding="utf-8") == (out_b / "rover.xml").read_text(encoding="utf-8")
        for k in ("friction_scale", "chassis_mass_scale", "chassis_com_offset_x_m", "chassis_com_offset_y_m", "motor_gain_scale"):
            assert params_a[k] == params_b[k]
        # Different seed -> different physics
        assert params_a["friction_scale"] != params_c["friction_scale"]

        # params.json exists and matches
        disk_params = json.loads((out_a / "params.json").read_text(encoding="utf-8"))
        assert disk_params["friction_scale"] == params_a["friction_scale"]

        # Compile in MuJoCo 3.12 and verify every physical parameter took effect
        model = mujoco.MjModel.from_xml_path(str(xml_a))

        base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        assert math.isclose(model.body_mass[base_id], params_a["chassis_mass_kg"], rel_tol=1e-5)
        assert math.isclose(float(np.sum(model.body_mass)), params_a["total_mass_kg"], rel_tol=1e-5)
        assert math.isclose(model.body_ipos[base_id][0], params_a["chassis_com_offset_x_m"], abs_tol=1e-5)
        assert math.isclose(model.body_ipos[base_id][1], params_a["chassis_com_offset_y_m"], abs_tol=1e-5)

        for w_geom in ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"):
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, w_geom)
            assert math.isclose(model.geom_friction[gid][0], params_a["wheel_sliding_friction"], rel_tol=1e-5)

        tile_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "terrain_tile_5_5")
        assert math.isclose(model.geom_friction[tile_id][0], 0.8 * params_a["friction_scale"], rel_tol=1e-5)

        for act_name, gain in params_a["motor_gain_scale"].items():
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, act_name)
            assert math.isclose(model.actuator_gainprm[aid][0], 50.0 * gain, rel_tol=1e-5)
            assert math.isclose(model.actuator_biasprm[aid][2], -50.0, rel_tol=1e-5)
            assert math.isclose(model.actuator_forcerange[aid][1], 15.0 * gain, rel_tol=1e-5)


def test_noise_relay_math():
    apply_lidar_noise, apply_gyro_noise = _load_noise_relay_funcs()

    # Zero noise / dropout -> exact identity
    rng0 = np.random.default_rng(123)
    pts = np.array([[2.0, 0.0, 0.0], [0.0, 3.0, 4.0]], dtype=np.float32)
    out0 = apply_lidar_noise(pts, range_sigma_m=0.0, dropout_prob=0.0, rng=rng0)
    assert np.allclose(out0, pts)
    assert apply_gyro_noise(0.1, -0.2, 0.3, 0.0, 0.0, rng0) == (0.1, -0.2, 0.3)

    # Large sample statistical verification
    n = 40_000
    dirs = np.tile(np.array([[1.0, 0.0, 0.0]], dtype=np.float64), (n, 1))
    pts_big = (dirs * 4.0).astype(np.float32)
    rng1 = np.random.default_rng(7)
    noisy = apply_lidar_noise(pts_big, range_sigma_m=0.04, dropout_prob=0.15, rng=rng1)
    kept_frac = noisy.shape[0] / n
    assert abs((1.0 - kept_frac) - 0.15) < 0.01
    # Noise is purely radial (y and z remain 0)
    assert np.max(np.abs(noisy[:, 1:])) < 1e-6
    assert abs(float(np.std(noisy[:, 0] - 4.0)) - 0.04) < 0.002

    # Gyro noise + constant z bias
    rng2 = np.random.default_rng(42)
    zs = [apply_gyro_noise(0.0, 0.0, 0.0, 0.01, 0.005, rng2)[2] for _ in range(20_000)]
    assert abs(float(np.mean(zs)) - 0.005) < 0.0005
    assert abs(float(np.std(zs)) - 0.01) < 0.0005


def test_robustness_analysis_synthetic_csv(tmp_path: pathlib.Path):
    csv_path = tmp_path / "robustness.csv"
    header = [
        "combo", "scenario", "group", "seed", "result", "failure_type",
        "time_s", "path_len_gt", "straight_dist", "path_ratio", "gt_goal_err",
        "tracking_err", "slip_ratio", "lateral_slide", "ekf_drift", "energy_J",
        "min_obstacle_clearance", "recoveries", "rtf", "level",
    ]
    rows = [
        # L0: dwb 2/2 true success; mppi 1/2 (one goal_miss at 1.8m)
        ["smac_hybrid+dwb", "flat_01", "flat", 0, "SUCCESS", "none", 22.0, 8.1, 8.0, 1.01, 0.20, 0.05, 0.01, 0.05, 0.08, 300.0, 2.0, 0, 1.0, 0],
        ["smac_hybrid+dwb", "slope_04", "slope", 0, "SUCCESS", "none", 37.0, 13.6, 13.5, 1.01, 0.80, 0.30, 0.01, 0.60, 0.70, 450.0, 2.5, 0, 1.0, 0],
        ["smac_hybrid+mppi", "flat_01", "flat", 0, "SUCCESS", "none", 25.0, 8.3, 8.0, 1.04, 0.30, 0.09, 0.01, 0.08, 0.12, 350.0, 2.0, 0, 1.0, 0],
        ["smac_hybrid+mppi", "slope_04", "slope", 0, "SUCCESS", "none", 70.0, 15.0, 13.5, 1.11, 1.80, 1.20, 0.02, 2.50, 2.90, 1100.0, 2.5, 0, 1.0, 0],
        # L2: dwb 1/2 (one goal_miss 1.2m); mppi 0/2 (one timeout, one goal_miss)
        ["smac_hybrid+dwb", "flat_01", "flat", 0, "SUCCESS", "none", 24.0, 8.4, 8.0, 1.05, 0.40, 0.10, 0.02, 0.10, 0.25, 360.0, 2.0, 0, 1.0, 2],
        ["smac_hybrid+dwb", "slope_04", "slope", 0, "SUCCESS", "none", 42.0, 14.0, 13.5, 1.04, 1.20, 0.50, 0.03, 0.90, 1.10, 560.0, 2.5, 0, 1.0, 2],
        ["smac_hybrid+mppi", "flat_01", "flat", 0, "SUCCESS", "none", 30.0, 8.8, 8.0, 1.10, 1.50, 0.40, 0.03, 0.40, 1.40, 500.0, 2.0, 0, 1.0, 2],
        ["smac_hybrid+mppi", "slope_04", "slope", 0, "FAILURE", "timeout", 120.0, 20.0, 13.5, 1.48, 6.00, 3.00, 0.05, 4.00, 5.50, 1900.0, 2.5, 2, 1.0, 2],
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    out_md = tmp_path / "summary.md"
    plots_dir = tmp_path / "plots"
    summary, plot_paths = analyze_robustness(csv_path, out_md, plots_dir)

    assert summary["total_runs"] == 8
    assert summary["levels"] == [0, 2]
    assert summary["success_by_cl"][("smac_hybrid+dwb", 0)] == (2, 2)
    assert summary["success_by_cl"][("smac_hybrid+mppi", 0)] == (1, 2)
    assert summary["success_by_cl"][("smac_hybrid+dwb", 2)] == (1, 2)
    assert summary["success_by_cl"][("smac_hybrid+mppi", 2)] == (0, 2)
    assert summary["failures_by_cl"][("smac_hybrid+mppi", 0)]["goal_miss"] == 1
    assert summary["failures_by_cl"][("smac_hybrid+dwb", 2)]["goal_miss"] == 1
    assert summary["failures_by_cl"][("smac_hybrid+mppi", 2)]["timeout"] == 1
    assert summary["failures_by_cl"][("smac_hybrid+mppi", 2)]["goal_miss"] == 1

    assert math.isclose(summary["gt_err_all"][("smac_hybrid+dwb", 0)][0], 0.50, rel_tol=1e-6)
    assert math.isclose(summary["energy_all"][("smac_hybrid+dwb", 0)][0], 375.0, rel_tol=1e-6)

    md_text = out_md.read_text(encoding="utf-8")
    assert "smac_hybrid+dwb" in md_text
    assert "smac_hybrid+mppi" in md_text
    assert "100.0% (2/2)" in md_text
    assert "0.0% (0/2)" in md_text

    expected_names = {"success_vs_level.png", "gt_err_vs_level.png", "energy_vs_level.png"}
    assert {p.name for p in plot_paths} == expected_names
    for p in plot_paths:
        assert p.exists() and p.stat().st_size > 1000
