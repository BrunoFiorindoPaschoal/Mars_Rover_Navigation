"""Unit tests for mars_yard_benchmark analysis, geometry, and hazard map clear."""

import csv
import math
import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src/mars_yard_benchmark"))
sys.path.insert(0, str(REPO / "src/mars_yard_nav"))

from mars_yard_benchmark.analysis import analyze_benchmark
from mars_yard_benchmark.geometry import (
    build_hazard_rock_contours,
    point_to_polyline_dist,
    signed_footprint_clearance,
)
from mars_yard_nav.hazard_core import HazardConfig, RollingHazardMap


def test_analysis_synthetic_csv(tmp_path: pathlib.Path):
    csv_path = tmp_path / "synthetic.csv"
    header = [
        "combo",
        "scenario",
        "group",
        "seed",
        "result",
        "failure_type",
        "time_s",
        "path_len_gt",
        "straight_dist",
        "path_ratio",
        "gt_goal_err",
        "tracking_err",
        "slip_ratio",
        "lateral_slide",
        "ekf_drift",
        "energy_J",
        "min_obstacle_clearance",
        "recoveries",
        "rtf",
    ]
    rows = [
        ["smac_hybrid+dwb", "flat_01", "flat", 0, "SUCCESS", "none", 20.0, 8.2, 8.0, 1.025, 0.15, 0.08, 0.01, 0.05, 0.10, 200.0, 1.50, 0, 1.0],
        ["smac_hybrid+dwb", "slope_01", "slope", 0, "SUCCESS", "none", 30.0, 10.5, 10.0, 1.050, 0.25, 0.12, 0.03, 0.30, 0.35, 400.0, 1.20, 0, 1.0],
        ["smac_hybrid+dwb", "rocks_01", "rocks", 0, "FAILURE", "collision", 18.0, 5.0, 9.0, 0.556, 4.00, 0.20, 0.02, 0.20, 0.25, 310.0, -0.05, 1, 1.0],
        ["smac_hybrid+dwb", "sand_01", "sand", 0, "SUCCESS", "none", 28.0, 9.5, 9.0, 1.056, 0.30, 0.14, 0.08, 0.40, 0.45, 350.0, 1.80, 0, 1.0],
        ["navfn+rpp", "flat_01", "flat", 0, "SUCCESS", "none", 22.0, 8.4, 8.0, 1.050, 0.20, 0.10, 0.01, 0.06, 0.12, 220.0, 1.60, 0, 1.0],
        ["navfn+rpp", "slope_01", "slope", 0, "FAILURE", "timeout", 120.0, 25.0, 10.0, 2.500, 3.00, 0.50, 0.05, 1.50, 2.10, 1800.0, 1.10, 2, 1.0],
        ["navfn+rpp", "rocks_01", "rocks", 0, "FAILURE", "stuck", 45.0, 4.0, 9.0, 0.444, 5.00, 0.35, 0.04, 0.60, 0.70, 900.0, 0.40, 3, 1.0],
        ["navfn+rpp", "sand_01", "sand", 0, "FAILURE", "aborted", 15.0, 2.0, 9.0, 0.222, 7.00, 0.25, 0.06, 0.25, 0.30, 250.0, 1.70, 1, 1.0],
        # Nav2 says SUCCESS but ground truth is 4 m off (EKF drift) -> reclassified goal_miss
        ["navfn+rpp", "slope_02", "slope", 0, "SUCCESS", "none", 60.0, 12.0, 10.0, 1.200, 4.00, 2.00, 0.02, 3.50, 4.00, 900.0, 3.00, 0, 1.0],
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)

    out_md = tmp_path / "summary.md"
    plots_dir = tmp_path / "plots"
    summary, plot_paths = analyze_benchmark(csv_path, out_md, plots_dir)

    assert summary["total_runs"] == 9
    assert summary["combo_overall"]["smac_hybrid+dwb"] == (3, 4)
    assert summary["combo_overall"]["navfn+rpp"] == (1, 5)
    assert summary["failure_counts"]["navfn+rpp"]["goal_miss"] == 1
    assert summary["failure_counts"]["smac_hybrid+dwb"]["collision"] == 1
    assert summary["failure_counts"]["navfn+rpp"]["timeout"] == 1
    assert summary["failure_counts"]["navfn+rpp"]["stuck"] == 1
    assert summary["failure_counts"]["navfn+rpp"]["aborted"] == 1

    mean_t, _ = summary["metric_stats"]["smac_hybrid+dwb"]["time_s"]
    assert math.isclose(mean_t, (20.0 + 30.0 + 28.0) / 3.0, rel_tol=1e-6)

    md_text = out_md.read_text(encoding="utf-8")
    assert "smac_hybrid+dwb" in md_text
    assert "navfn+rpp" in md_text
    assert "75.0% (3/4)" in md_text
    assert "20.0% (1/5)" in md_text

    assert len(plot_paths) == 3
    for p in plot_paths:
        assert p.exists() and p.stat().st_size > 1000


def test_footprint_clearance_and_polyline_distance():
    rocks = [
        {"name": "boulder_a", "x": 2.0, "y": 0.0, "size": [0.5, 0.5, 0.6], "yaw": 0.0},
        {"name": "pebble_b", "x": 0.0, "y": 0.0, "size": [0.1, 0.1, 0.08], "yaw": 0.0},
    ]
    pts, meta = build_hazard_rock_contours(rocks, min_exposed_height=0.12)
    assert len(meta) == 1  # pebble ignored

    # Rover at (0,0,0) with half_len=0.40 -> front edge at x=0.40, boulder left edge at x=1.50 -> 1.10 m clear
    clr_far = signed_footprint_clearance(0.0, 0.0, 0.0, pts, meta)
    assert math.isclose(clr_far, 1.10, abs_tol=0.01)

    # Rover at (1.2, 0, 0) -> front edge at x=1.60 inside boulder (x=1.50) -> negative clearance (collision)
    clr_hit = signed_footprint_clearance(1.2, 0.0, 0.0, pts, meta)
    assert clr_hit < -0.05

    poly = np.array([[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]], dtype=np.float64)
    assert math.isclose(point_to_polyline_dist(2.5, 0.3, poly), 0.3, abs_tol=1e-9)
    assert math.isclose(point_to_polyline_dist(5.4, 3.0, poly), 0.4, abs_tol=1e-9)


def test_rolling_hazard_map_clear():
    cfg = HazardConfig()
    rmap = RollingHazardMap(cfg)
    hz = np.array([[10, 10], [11, 10]], dtype=np.int32)
    fr = np.array([[5, 5]], dtype=np.int32)
    rmap.update(0.0, 0.0, 1.0, hz, fr)
    assert np.any(rmap.state == 100)
    assert rmap.initialized

    rmap.clear()
    assert not rmap.initialized
    assert np.all(rmap.state == -1)
    assert np.all(rmap.score == 0)
