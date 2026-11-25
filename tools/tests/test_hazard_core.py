"""Pure-math unit tests for mars_yard_nav.hazard_core (no ROS required)."""

import math
import pathlib
import sys

import numpy as np
import pytest

ROOT = pathlib.Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/mars_yard_nav"))

from mars_yard_nav.hazard_core import (  # noqa: E402
    HazardConfig,
    RollingHazardMap,
    detect_hazards_scan,
    level_rotation,
    quat_to_rpy,
    transform_points_to_odom,
    yaw_rotation,
)


def _sample_plane(slope_deg: float, azimuth_deg: float = 0.0, size: float = 6.0, step: float = 0.05):
    xs = np.arange(-size / 2, size / 2 + 1e-9, step)
    ys = np.arange(-size / 2, size / 2 + 1e-9, step)
    xx, yy = np.meshgrid(xs, ys)
    ang = math.radians(azimuth_deg)
    t = math.tan(math.radians(slope_deg))
    zz = t * (xx * math.cos(ang) + yy * math.sin(ang))
    return np.column_stack((xx.ravel(), yy.ravel(), zz.ravel()))


def test_tilted_20deg_plane_with_noise_zero_hazards():
    """A 20 deg tilted plane with realistic sensor noise must produce zero hazards."""
    cfg = HazardConfig()
    rng = np.random.default_rng(123)
    for az in (0.0, 35.0, 90.0, -140.0):
        pts = _sample_plane(slope_deg=20.0, azimuth_deg=az, size=6.0, step=0.05)
        pts[:, 2] += rng.normal(0.0, 0.008, size=pts.shape[0])
        haz, free, _ = detect_hazards_scan(pts, cfg)
        assert haz.shape[0] == 0, f"False hazards on 20 deg plane (az={az}): {haz.shape[0]}"
        assert free.shape[0] > 500


@pytest.mark.parametrize("shape", ["box", "ellipsoid"])
def test_15cm_obstacle_on_15deg_slope_detected(shape):
    """A 0.15 m tall box or ellipsoid on a 15 deg slope must be detected as a hazard."""
    cfg = HazardConfig()
    pts = _sample_plane(slope_deg=15.0, azimuth_deg=20.0, size=6.0, step=0.04)
    ox, oy = 0.5, -0.3
    dx = pts[:, 0] - ox
    dy = pts[:, 1] - oy

    if shape == "box":
        on_obs = (np.abs(dx) <= 0.18) & (np.abs(dy) <= 0.18)
        pts[on_obs, 2] += 0.15
    else:
        rx, ry, h = 0.25, 0.20, 0.15
        r2 = (dx / rx) ** 2 + (dy / ry) ** 2
        on_obs = r2 < 1.0
        pts[on_obs, 2] += h * np.sqrt(1.0 - r2[on_obs])

    haz, free, steps = detect_hazards_scan(pts, cfg)
    assert haz.shape[0] >= 4, f"Expected hazard cells on 0.15 m {shape}, got {haz.shape[0]}"
    assert free.shape[0] > 500

    # All hazard cells must lie on/near the obstacle (no false hazards elsewhere on the 15 deg slope).
    hx = (haz[:, 0] + 0.5) * cfg.cell_size
    hy = (haz[:, 1] + 0.5) * cfg.cell_size
    dists = np.hypot(hx - ox, hy - oy)
    assert dists.min() < 0.15
    assert dists.max() < 0.35, f"Spurious hazard {dists.max():.2f} m from obstacle"
    assert float(steps.max()) >= 0.135


def test_8cm_bump_no_hazard():
    """A 0.08 m bump (below 0.12 m wheel-radius threshold) must produce zero hazards."""
    cfg = HazardConfig()
    pts = _sample_plane(slope_deg=10.0, azimuth_deg=-30.0, size=6.0, step=0.04)
    dx = pts[:, 0] - 0.2
    dy = pts[:, 1] + 0.4
    r2 = (dx / 0.25) ** 2 + (dy / 0.25) ** 2
    on_bump = r2 < 1.0
    pts[on_bump, 2] += 0.08 * np.sqrt(1.0 - r2[on_bump])

    haz, free, _ = detect_hazards_scan(pts, cfg)
    assert haz.shape[0] == 0, f"0.08 m bump should not trigger hazards (got {haz.shape[0]})"
    assert free.shape[0] > 500


def test_levelling_with_known_roll_and_pitch():
    """Levelling recovers a flat ground plane at constant z under known roll, pitch, and yaw."""
    roll = math.radians(12.0)
    pitch = math.radians(-18.0)
    yaw = math.radians(42.0)
    odom_x, odom_y, true_z = 3.5, -2.0, 0.275
    t_bl = np.array([0.30, 0.0, 0.40], dtype=np.float64)
    q_bl = (0.0, 0.0, 0.0, 1.0)

    # Generate flat world ground points at z_world = 0.0 around the rover.
    gx = np.linspace(odom_x - 4.0, odom_x + 4.0, 35)
    gy = np.linspace(odom_y - 4.0, odom_y + 4.0, 35)
    xx, yy = np.meshgrid(gx, gy)
    pts_world = np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size)))

    # Project world points into tilted base_link and then lidar_link.
    r_wb = yaw_rotation(yaw) @ level_rotation(roll, pitch)
    t_wb = np.array([odom_x, odom_y, true_z])
    pts_base = (pts_world - t_wb) @ r_wb
    pts_lidar = pts_base - t_bl

    # Recover odom points using transform_points_to_odom (without knowing true_z).
    pts_odom = transform_points_to_odom(
        pts_lidar, t_bl, q_bl, roll, pitch, odom_x, odom_y, yaw, min_range=0.65, max_range=7.5
    )
    assert pts_odom.shape[0] > 200
    # Leveled z must be constant (= -true_z) across the entire ground plane!
    np.testing.assert_allclose(pts_odom[:, 2], -true_z, atol=1e-9)

    # Check matching world x, y for points within range filter.
    r_base = np.hypot(pts_base[:, 0], pts_base[:, 1])
    keep = (r_base >= 0.65) & (r_base <= 7.5)
    np.testing.assert_allclose(pts_odom[:, :2], pts_world[keep, :2], atol=1e-9)

    # Verify quat_to_rpy round-trip on the rotation matrix via quaternion formula.
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    r_out, p_out, y_out = quat_to_rpy(qx, qy, qz, qw)
    assert r_out == pytest.approx(roll, abs=1e-9)
    assert p_out == pytest.approx(pitch, abs=1e-9)
    assert y_out == pytest.approx(yaw, abs=1e-9)


def test_rolling_map_memory_persistence_and_cloud_heights():
    """RollingHazardMap retains hazard labels across rover motion and exports z=0.3/z=0.0."""
    cfg = HazardConfig(cell_size=0.1, grid_size_m=10.0, memory_ttl=20.0, publish_radius_m=8.0)
    rmap = RollingHazardMap(cfg)

    haz = np.array([[20, 5], [21, 5]], dtype=np.int32)
    free = np.array([[10, 0], [11, 0]], dtype=np.int32)
    rmap.update(0.0, 0.0, 1.0, haz, free)

    pts = rmap.export_cloud_points(0.0, 0.0)
    assert pts.shape == (4, 3)
    assert int(np.sum(np.isclose(pts[:, 2], 0.3))) == 2
    assert int(np.sum(np.isclose(pts[:, 2], 0.0))) == 2

    # Rover moves 1.5 m forward; hazard labels must persist in odom without shifting world coords.
    rmap.update(1.5, 0.0, 5.0, np.empty((0, 2), dtype=np.int32), np.empty((0, 2), dtype=np.int32))
    pts2 = rmap.export_cloud_points(1.5, 0.0)
    haz_pts = pts2[np.isclose(pts2[:, 2], 0.3)]
    assert haz_pts.shape == (2, 3)
    np.testing.assert_allclose(sorted(haz_pts[:, 0]), [2.05, 2.15], atol=1e-5)

    # After memory_ttl expires, stale cells clear back to unknown.
    rmap.update(1.5, 0.0, 30.0, np.empty((0, 2), dtype=np.int32), np.empty((0, 2), dtype=np.int32))
    assert rmap.export_cloud_points(1.5, 0.0).shape[0] == 0
