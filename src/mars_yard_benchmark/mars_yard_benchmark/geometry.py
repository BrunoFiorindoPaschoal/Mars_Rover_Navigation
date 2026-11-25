"""Geometry utilities for benchmark clearance, collision, and path tracking error."""

import math
from typing import Sequence, Tuple

import numpy as np


# Rover physical footprint half-extents (m): chassis 0.80 m long, outer wheel span 0.74 m wide.
ROVER_HALF_LENGTH = 0.40
ROVER_HALF_WIDTH = 0.37


def build_hazard_rock_contours(
    rocks: Sequence[dict],
    min_exposed_height: float = 0.12,
    n_arc_pts: int = 36,
) -> Tuple[np.ndarray, list]:
    """Precompute world-frame ellipse perimeter points for hazard rocks/boulders.

    Only rocks with exposed height `0.7 * size[2] >= min_exposed_height` are included
    (matching hazard_mapper's 0.12 m step threshold; smaller pebbles are traversable).

    Returns:
      - all_pts: (M, 2) float64 world coordinates of all hazard-rock perimeter samples
      - meta: list of (rx, ry, a, b, cos_y, sin_y, name) for exact corner-inside checks
    """
    angles = np.linspace(0.0, 2.0 * math.pi, n_arc_pts, endpoint=False)
    ca = np.cos(angles)
    sa = np.sin(angles)

    pts_list = []
    meta = []
    for idx, r in enumerate(rocks):
        exp_h = 0.7 * float(r["size"][2])
        if exp_h < min_exposed_height:
            continue
        rx, ry = float(r["x"]), float(r["y"])
        a, b = float(r["size"][0]), float(r["size"][1])
        yaw = float(r.get("yaw", 0.0))
        cy, sy = math.cos(yaw), math.sin(yaw)

        lx = a * ca
        ly = b * sa
        wx = rx + cy * lx - sy * ly
        wy = ry + sy * lx + cy * ly
        pts_list.append(np.column_stack((wx, wy)))
        meta.append((rx, ry, a, b, cy, sy, r.get("name", f"rock_{idx}")))

    if not pts_list:
        return np.empty((0, 2), dtype=np.float64), []
    return np.vstack(pts_list), meta


def signed_footprint_clearance(
    x: float,
    y: float,
    yaw: float,
    rock_pts: np.ndarray,
    rock_meta: list,
    half_len: float = ROVER_HALF_LENGTH,
    half_wid: float = ROVER_HALF_WIDTH,
) -> float:
    """Minimum signed distance (m) from the rover's oriented footprint to hazard rocks.

    Returns a positive value when clear, 0.0 when touching, and negative when overlapping.
    Returns +inf when there are no hazard rocks.
    """
    if rock_pts.shape[0] == 0:
        return math.inf

    # Transform rock perimeter points into rover body frame.
    dx_w = rock_pts[:, 0] - x
    dy_w = rock_pts[:, 1] - y
    c = math.cos(yaw)
    s = math.sin(yaw)
    bx = c * dx_w + s * dy_w
    by = -s * dx_w + c * dy_w

    qx = np.abs(bx) - half_len
    qy = np.abs(by) - half_wid
    outside = np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
    inside = np.minimum(np.maximum(qx, qy), 0.0)
    min_dist = float(np.min(outside + inside))

    # Also check whether any of the 4 rover corners lies strictly inside a rock ellipse.
    corners_body = (
        (half_len, half_wid),
        (half_len, -half_wid),
        (-half_len, -half_wid),
        (-half_len, half_wid),
    )
    for cx_b, cy_b in corners_body:
        wx = x + c * cx_b - s * cy_b
        wy = y + s * cx_b + c * cy_b
        for rx, ry, a, b, rcy, rsy, _ in rock_meta:
            ux = wx - rx
            uy = wy - ry
            if ux * ux + uy * uy > (max(a, b) + 0.05) ** 2:
                continue
            lu = rcy * ux + rsy * uy
            lv = -rsy * ux + rcy * uy
            rn = math.hypot(lu / a, lv / b)
            if rn < 1.0:
                pen = (rn - 1.0) * min(a, b)
                if pen < min_dist:
                    min_dist = pen

    return min_dist


def point_to_polyline_dist(px: float, py: float, polyline: np.ndarray) -> float:
    """Minimum Euclidean distance from (px, py) to a 2D polyline (N, 2)."""
    n = polyline.shape[0]
    if n == 0:
        return float("nan")
    if n == 1:
        return float(math.hypot(px - polyline[0, 0], py - polyline[0, 1]))

    a = polyline[:-1]
    b = polyline[1:]
    v = b - a
    len2 = v[:, 0] * v[:, 0] + v[:, 1] * v[:, 1]
    wx = px - a[:, 0]
    wy = py - a[:, 1]
    dot = wx * v[:, 0] + wy * v[:, 1]
    t = np.where(len2 > 1e-12, np.clip(dot / np.where(len2 > 1e-12, len2, 1.0), 0.0, 1.0), 0.0)
    proj_x = a[:, 0] + t * v[:, 0]
    proj_y = a[:, 1] + t * v[:, 1]
    dists = np.hypot(px - proj_x, py - proj_y)
    return float(np.min(dists))
