"""Pure NumPy terrain levelling, slope-invariant step-height detection, and rolling hazard map.

Zero ROS imports so every piece of geometry and hazard math can be unit-tested directly
with pytest.

Why IMU levelling is done here (and not via tf `odom -> base_link`):
The EKF runs with `two_d_mode: true` (for 2D Nav2 costmaps and to prevent unobservable z-drift),
so tf `odom -> base_link` carries only `(x, y, yaw)` with `z = roll = pitch = 0`.
To project 3D lidar points into a gravity-leveled `odom` frame per scan, we chain:
  1. Static `base_link <- lidar_link` transform from tf (fixed mast offset).
  2. IMU roll and pitch (`Ry(pitch) @ Rx(roll)`), discarding IMU yaw (no Mars magnetometer).
  3. EKF planar pose `(x, y, yaw)` from tf `odom -> base_link`.
Because `base_link` z is unknown in 2D mode, raw point z is only consistent within a single scan;
step-height detection therefore runs per scan, and only discrete hazard/free LABELS are stored
in the rolling `odom` memory.
"""

from dataclasses import dataclass
import math
from typing import Tuple

import numpy as np


@dataclass(frozen=True)
class HazardConfig:
    cell_size: float = 0.1
    step_threshold: float = 0.12
    window_radius_m: float = 0.7
    ground_inlier_threshold: float = 0.06
    min_ground_cells: int = 8
    min_plane_eig: float = 0.015
    max_plane_slope_deg: float = 35.0
    free_margin_ratio: float = 0.7
    min_range: float = 0.65
    max_range: float = 7.5
    grid_size_m: float = 24.0
    memory_ttl: float = 30.0
    publish_radius_m: float = 10.0
    hazard_z: float = 0.3
    free_z: float = 0.0


def quat_to_rotmat(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert unit quaternion (x, y, z, w) to a 3x3 rotation matrix."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return np.eye(3, dtype=np.float64)
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Extract intrinsic ZYX Euler angles (roll, pitch, yaw) in radians from (x, y, z, w)."""
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        return 0.0, 0.0, 0.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def level_rotation(roll: float, pitch: float) -> np.ndarray:
    """Rotation matrix `Ry(pitch) @ Rx(roll)` mapping body vectors to gravity-leveled frame."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    return np.array(
        [
            [cp, sp * sr, sp * cr],
            [0.0, cr, -sr],
            [-sp, cp * sr, cp * cr],
        ],
        dtype=np.float64,
    )


def yaw_rotation(yaw: float) -> np.ndarray:
    """Rotation matrix `Rz(yaw)` about +z."""
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array(
        [
            [cy, -sy, 0.0],
            [sy, cy, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def transform_points_to_odom(
    pts_lidar: np.ndarray,
    t_base_lidar: np.ndarray,
    q_base_lidar: Tuple[float, float, float, float],
    imu_roll: float,
    imu_pitch: float,
    odom_x: float,
    odom_y: float,
    odom_yaw: float,
    min_range: float = 0.65,
    max_range: float = 7.5,
) -> np.ndarray:
    """Transform `lidar_link` points -> `base_link` -> gravity-leveled -> `odom` (2D EKF pose).

    Filters points outside `[min_range, max_range]` in horizontal `base_link` distance to reject
    rover-body returns and sparse far-range rings.
    """
    pts = np.asarray(pts_lidar, dtype=np.float64)
    if pts.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    pts = pts.reshape(-1, 3)

    finite = np.all(np.isfinite(pts), axis=1)
    pts = pts[finite]
    if pts.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)

    r_bl = quat_to_rotmat(*q_base_lidar)
    t_bl = np.asarray(t_base_lidar, dtype=np.float64).reshape(3)
    pts_base = pts @ r_bl.T + t_bl

    horiz_r2 = pts_base[:, 0] * pts_base[:, 0] + pts_base[:, 1] * pts_base[:, 1]
    keep = (horiz_r2 >= min_range * min_range) & (horiz_r2 <= max_range * max_range)
    pts_base = pts_base[keep]
    if pts_base.shape[0] == 0:
        return np.empty((0, 3), dtype=np.float64)

    r_total = yaw_rotation(odom_yaw) @ level_rotation(imu_roll, imu_pitch)
    pts_odom = pts_base @ r_total.T
    pts_odom[:, 0] += odom_x
    pts_odom[:, 1] += odom_y
    return pts_odom


def _box_sum_2d(arr: np.ndarray, radius: int) -> np.ndarray:
    """Exact 2D box sum over `(2*radius + 1) x (2*radius + 1)` window via integral image."""
    h, w = arr.shape
    padded = np.pad(arr, ((radius + 1, radius), (radius + 1, radius)), mode="constant", constant_values=0.0)
    integ = padded.cumsum(axis=0).cumsum(axis=1)
    k = 2 * radius + 1
    return (
        integ[k : k + h, k : k + w]
        - integ[0:h, k : k + w]
        - integ[k : k + h, 0:w]
        + integ[0:h, 0:w]
    )


def _fit_local_plane(
    mask: np.ndarray,
    gx: np.ndarray,
    gy: np.ndarray,
    gz: np.ndarray,
    radius: int,
    min_cells: int,
    min_eig: float,
    max_tan: float,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized 2D least-squares plane fit `z = cz + a*(x - cx) + b*(y - cy)` per cell."""
    m = mask.astype(np.float64)
    x = np.where(mask, gx, 0.0)
    y = np.where(mask, gy, 0.0)
    z = np.where(mask, gz, 0.0)

    n = _box_sum_2d(m, radius)
    sx = _box_sum_2d(x, radius)
    sy = _box_sum_2d(y, radius)
    sz = _box_sum_2d(z, radius)
    sxx = _box_sum_2d(x * x, radius)
    sxy = _box_sum_2d(x * y, radius)
    syy = _box_sum_2d(y * y, radius)
    sxz = _box_sum_2d(x * z, radius)
    syz = _box_sum_2d(y * z, radius)

    n_safe = np.maximum(n, 1.0)
    cx = sx / n_safe
    cy = sy / n_safe
    cz = sz / n_safe

    cxx = sxx / n_safe - cx * cx
    cxy = sxy / n_safe - cx * cy
    cyy = syy / n_safe - cy * cy
    cxz = sxz / n_safe - cx * cz
    cyz = syz / n_safe - cy * cz

    tr = cxx + cyy
    disc = np.sqrt(np.maximum(0.0, (cxx - cyy) ** 2 + 4.0 * cxy * cxy))
    eig_min = 0.5 * (tr - disc)
    det = cxx * cyy - cxy * cxy

    det_safe = np.where(det > 1e-12, det, 1.0)
    a = (cyy * cxz - cxy * cyz) / det_safe
    b = (cxx * cyz - cxy * cxz) / det_safe
    slope_tan = np.hypot(a, b)

    valid = (n >= min_cells) & (eig_min >= min_eig) & (det > 1e-12) & (slope_tan <= max_tan)
    return valid, a, b, cx, cy, cz


def detect_hazards_scan(
    pts_odom: np.ndarray,
    cfg: HazardConfig,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Detect per-scan hazard and free cells using a 2-pass robust local plane fit.

    Method:
    1. Bin leveled points into 0.1 m cells, retaining exact `(x, y, z)` of both the lowest
       and highest point per cell (eliminating cell-center slope discretization error).
    2. Pass 1: fit a local plane to cell minima `(x_min, y_min, z_min)` over a square window
       of radius `window_radius_m`. Reject elevated outlier cells (`z_min - z_plane > ground_inlier_threshold`).
    3. Pass 2: refit the local plane using only ground-inlier cells so rocks/boulders cannot
       pull the reference plane upward.
    4. Compute per-cell step height as `z_max - z_plane(x_max, y_max)` (plus negative step
       `z_plane(x_min, y_min) - z_min` for drop-offs). On a smooth tilted plane of any angle
       up to `max_plane_slope_deg`, the residual is identically zero.

    Returns:
        hazard_ij: int32 array (H, 2) of global integer cell indices `(ix, iy)` marked hazard.
        free_ij:   int32 array (F, 2) of global integer cell indices `(ix, iy)` marked free.
        step_vals: float64 array (H,) of step heights for `hazard_ij` (for diagnostics/tests).
    """
    pts = np.asarray(pts_odom, dtype=np.float64)
    empty_ij = np.empty((0, 2), dtype=np.int32)
    empty_f = np.empty((0,), dtype=np.float64)
    if pts.size == 0:
        return empty_ij, empty_ij, empty_f
    pts = pts.reshape(-1, 3)

    cs = cfg.cell_size
    ix = np.floor(pts[:, 0] / cs).astype(np.int32)
    iy = np.floor(pts[:, 1] / cs).astype(np.int32)

    ix_min, ix_max = int(ix.min()), int(ix.max())
    iy_min, iy_max = int(iy.min()), int(iy.max())
    w = ix_max - ix_min + 1
    h = iy_max - iy_min + 1

    # Guard against degenerate huge bounding boxes from corrupt input
    if w <= 0 or h <= 0 or w * h > 2_000_000:
        return empty_ij, empty_ij, empty_f

    # Shift coordinates to local origin for numerical conditioning of second moments
    x_ref = float(pts[:, 0].mean())
    y_ref = float(pts[:, 1].mean())
    z_ref = float(pts[:, 2].mean())
    px = pts[:, 0] - x_ref
    py = pts[:, 1] - y_ref
    pz = pts[:, 2] - z_ref

    r_idx = iy - iy_min
    c_idx = ix - ix_min
    flat = r_idx * w + c_idx

    occupied = np.zeros(h * w, dtype=bool)
    occupied[flat] = True

    order = np.argsort(pz, kind="mergesort")
    x_max = np.zeros(h * w, dtype=np.float64)
    y_max = np.zeros(h * w, dtype=np.float64)
    z_max = np.zeros(h * w, dtype=np.float64)
    x_max[flat[order]] = px[order]
    y_max[flat[order]] = py[order]
    z_max[flat[order]] = pz[order]

    rev = order[::-1]
    x_min = np.zeros(h * w, dtype=np.float64)
    y_min = np.zeros(h * w, dtype=np.float64)
    z_min = np.zeros(h * w, dtype=np.float64)
    x_min[flat[rev]] = px[rev]
    y_min[flat[rev]] = py[rev]
    z_min[flat[rev]] = pz[rev]

    occupied = occupied.reshape(h, w)
    x_max, y_max, z_max = x_max.reshape(h, w), y_max.reshape(h, w), z_max.reshape(h, w)
    x_min, y_min, z_min = x_min.reshape(h, w), y_min.reshape(h, w), z_min.reshape(h, w)

    radius = max(1, int(round(cfg.window_radius_m / cs)))
    max_tan = math.tan(math.radians(cfg.max_plane_slope_deg))

    # Pass 1: initial local plane fit over all occupied cell minima
    valid1, a1, b1, cx1, cy1, cz1 = _fit_local_plane(
        occupied, x_min, y_min, z_min, radius, cfg.min_ground_cells, cfg.min_plane_eig, max_tan
    )
    z_hat1_min = cz1 + a1 * (x_min - cx1) + b1 * (y_min - cy1)
    z_hat1_max = cz1 + a1 * (x_max - cx1) + b1 * (y_max - cy1)
    res1_min = z_min - z_hat1_min
    step1 = np.maximum(z_max - z_hat1_max, z_hat1_min - z_min)

    # Pass 2: ground-only refit excluding cells protruding above Pass-1 plane
    ground_mask = occupied & (~valid1 | (res1_min <= cfg.ground_inlier_threshold))
    valid2, a2, b2, cx2, cy2, cz2 = _fit_local_plane(
        ground_mask, x_min, y_min, z_min, radius, cfg.min_ground_cells, cfg.min_plane_eig, max_tan
    )
    z_hat2_max = cz2 + a2 * (x_max - cx2) + b2 * (y_max - cy2)
    z_hat2_min = cz2 + a2 * (x_min - cx2) + b2 * (y_min - cy2)
    step2 = np.maximum(z_max - z_hat2_max, z_hat2_min - z_min)

    # Hazard if Pass-2 step exceeds threshold, or Pass-2 lacks ground neighbors (e.g. huge boulder top)
    # while Pass-1 already detected a clear step.
    is_hazard = occupied & (
        (valid2 & (step2 > cfg.step_threshold))
        | (~valid2 & valid1 & (step1 > cfg.step_threshold))
    )
    is_free = occupied & valid2 & (~is_hazard) & (step2 <= cfg.step_threshold * cfg.free_margin_ratio)

    step_eff = np.where(valid2, step2, step1)

    hr, hc = np.nonzero(is_hazard)
    fr, fc = np.nonzero(is_free)
    hazard_ij = np.column_stack((hc + ix_min, hr + iy_min)).astype(np.int32)
    free_ij = np.column_stack((fc + ix_min, fr + iy_min)).astype(np.int32)
    step_vals = step_eff[hr, hc]
    return hazard_ij, free_ij, step_vals


class RollingHazardMap:
    """Rolling 2D grid of hazard/free LABELS in `odom` centered on the rover.

    Stores only discrete labels (`-1` unknown, `0` free, `100` hazard), hysteresis scores,
    and timestamps—never raw z elevations—so 2D-EKF height drift cannot corrupt the map.
    """

    def __init__(self, cfg: HazardConfig) -> None:
        self.cfg = cfg
        self.cell_size = cfg.cell_size
        self.dim = max(10, int(round(cfg.grid_size_m / cfg.cell_size)))
        self.origin_ix = -(self.dim // 2)
        self.origin_iy = -(self.dim // 2)
        self.initialized = False
        self.state = np.full((self.dim, self.dim), -1, dtype=np.int8)
        self.score = np.zeros((self.dim, self.dim), dtype=np.int8)
        self.stamp = np.full((self.dim, self.dim), -math.inf, dtype=np.float64)

    def _recenter(self, odom_x: float, odom_y: float) -> None:
        center_ix = int(math.floor(odom_x / self.cell_size))
        center_iy = int(math.floor(odom_y / self.cell_size))
        new_ox = center_ix - self.dim // 2
        new_oy = center_iy - self.dim // 2

        if not self.initialized:
            self.origin_ix = new_ox
            self.origin_iy = new_oy
            self.initialized = True
            return

        dx = new_ox - self.origin_ix
        dy = new_oy - self.origin_iy
        if dx == 0 and dy == 0:
            return

        new_state = np.full((self.dim, self.dim), -1, dtype=np.int8)
        new_score = np.zeros((self.dim, self.dim), dtype=np.int8)
        new_stamp = np.full((self.dim, self.dim), -math.inf, dtype=np.float64)

        if abs(dx) < self.dim and abs(dy) < self.dim:
            src_x0 = max(0, dx)
            src_x1 = min(self.dim, self.dim + dx)
            dst_x0 = max(0, -dx)
            dst_x1 = dst_x0 + (src_x1 - src_x0)

            src_y0 = max(0, dy)
            src_y1 = min(self.dim, self.dim + dy)
            dst_y0 = max(0, -dy)
            dst_y1 = dst_y0 + (src_y1 - src_y0)

            new_state[dst_y0:dst_y1, dst_x0:dst_x1] = self.state[src_y0:src_y1, src_x0:src_x1]
            new_score[dst_y0:dst_y1, dst_x0:dst_x1] = self.score[src_y0:src_y1, src_x0:src_x1]
            new_stamp[dst_y0:dst_y1, dst_x0:dst_x1] = self.stamp[src_y0:src_y1, src_x0:src_x1]

        self.origin_ix = new_ox
        self.origin_iy = new_oy
        self.state = new_state
        self.score = new_score
        self.stamp = new_stamp

    def update(
        self,
        odom_x: float,
        odom_y: float,
        sim_time: float,
        hazard_ij: np.ndarray,
        free_ij: np.ndarray,
    ) -> None:
        """Update rolling labels with one scan's detected hazard and free cell indices."""
        self._recenter(odom_x, odom_y)

        # Expire stale cells
        if self.cfg.memory_ttl > 0.0:
            expired = (self.state != -1) & ((sim_time - self.stamp) > self.cfg.memory_ttl)
            if np.any(expired):
                self.state[expired] = -1
                self.score[expired] = 0
                self.stamp[expired] = -math.inf

        # Apply free updates first (decrement hazard hysteresis score before clearing)
        if free_ij.shape[0] > 0:
            lx = free_ij[:, 0] - self.origin_ix
            ly = free_ij[:, 1] - self.origin_iy
            inb = (lx >= 0) & (lx < self.dim) & (ly >= 0) & (ly < self.dim)
            lx, ly = lx[inb], ly[inb]
            if lx.size > 0:
                new_sc = np.maximum(self.score[ly, lx] - 1, 0)
                self.score[ly, lx] = new_sc
                cleared = new_sc == 0
                self.state[ly[cleared], lx[cleared]] = 0
                self.stamp[ly, lx] = sim_time

        # Apply hazard updates (wins over free in same scan)
        if hazard_ij.shape[0] > 0:
            lx = hazard_ij[:, 0] - self.origin_ix
            ly = hazard_ij[:, 1] - self.origin_iy
            inb = (lx >= 0) & (lx < self.dim) & (ly >= 0) & (ly < self.dim)
            lx, ly = lx[inb], ly[inb]
            if lx.size > 0:
                self.score[ly, lx] = np.minimum(self.score[ly, lx] + 2, 4)
                self.state[ly, lx] = 100
                self.stamp[ly, lx] = sim_time

    def export_cloud_points(self, odom_x: float, odom_y: float) -> np.ndarray:
        """Return (K, 3) float32 points in `odom`: hazard cells at z=hazard_z, free at z=free_z."""
        r2_max = self.cfg.publish_radius_m ** 2
        hy, hx = np.nonzero(self.state == 100)
        fy, fx = np.nonzero(self.state == 0)

        parts = []
        if hx.size > 0:
            wx = (hx + self.origin_ix + 0.5) * self.cell_size
            wy = (hy + self.origin_iy + 0.5) * self.cell_size
            keep = (wx - odom_x) ** 2 + (wy - odom_y) ** 2 <= r2_max
            if np.any(keep):
                hz = np.full(int(np.sum(keep)), self.cfg.hazard_z, dtype=np.float32)
                parts.append(np.column_stack((wx[keep].astype(np.float32), wy[keep].astype(np.float32), hz)))

        if fx.size > 0:
            wx = (fx + self.origin_ix + 0.5) * self.cell_size
            wy = (fy + self.origin_iy + 0.5) * self.cell_size
            keep = (wx - odom_x) ** 2 + (wy - odom_y) ** 2 <= r2_max
            if np.any(keep):
                fz = np.full(int(np.sum(keep)), self.cfg.free_z, dtype=np.float32)
                parts.append(np.column_stack((wx[keep].astype(np.float32), wy[keep].astype(np.float32), fz)))

        if not parts:
            return np.empty((0, 3), dtype=np.float32)
        return np.vstack(parts)

    def origin_world(self) -> Tuple[float, float]:
        """World (odom) position of the bottom-left corner of cell (0, 0)."""
        return self.origin_ix * self.cell_size, self.origin_iy * self.cell_size
