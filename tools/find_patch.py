"""Search a HiRISE DTM for a rover-friendly terrain patch with obstacle relief."""

import argparse

import numpy as np
import rasterio
from rasterio.windows import from_bounds


def _is_valid_mask(data, nodata):
    """Return a boolean mask where True means valid finite elevation data."""
    valid = np.isfinite(data)
    if nodata is not None and np.isfinite(nodata):
        valid &= ~np.isclose(data, nodata, rtol=1e-5, atol=1e-5)
    return valid


def _ortho_patch_valid(ortho_src, cx, cy, size_m):
    """Check that the matching orthoimage patch lies in bounds and has non-zero pixels."""
    half = size_m / 2.0
    west, east = cx - half, cx + half
    south, north = cy - half, cy + half
    b = ortho_src.bounds
    if west < b.left or east > b.right or south < b.bottom or north > b.top:
        return False
    win = from_bounds(west, south, east, north, transform=ortho_src.transform)
    win = win.round_offsets().round_lengths()
    arr = ortho_src.read(1, window=win)
    # Reject patches with empty/black border strips in the orthoimage.
    return float(np.mean(arr > 0)) > 0.99


def score_candidate(stats):
    """Score a candidate patch (lower is better) for rover navigation variety."""
    # Prefer median slope ~5.5 deg, ~8% lethal cells (>20 deg) including an
    # interior obstacle (so Nav2 must route around it), and max slope ~26 deg.
    return (
        abs(stats["p50_deg"] - 5.5) * 1.5
        + abs(stats["pct_above_20"] - 8.0) * 0.6
        + abs(stats["interior_gt20"] - 6.0) * 0.8
        + abs(stats["max_deg"] - 26.0) * 0.25
        + abs(stats["pct_15_30"] - 12.0) * 0.3
    )


def search_patches(
    dtm_path,
    size_m=50.0,
    stride_m=50.0,
    ortho_path=None,
    top_k=10,
):
    """Scan the DTM on a regular grid and return the top_k candidate patches."""
    with rasterio.open(dtm_path) as src:
        dtm = src.read(1).astype(np.float32)
        valid = _is_valid_mask(dtm, src.nodata)
        dx = abs(src.transform.a)
        dy = abs(src.transform.e)
        transform = src.transform
        height, width = dtm.shape

    patch_px_x = int(round(size_m / dx))
    patch_px_y = int(round(size_m / dy))
    stride_px_x = max(1, int(round(stride_m / dx)))
    stride_px_y = max(1, int(round(stride_m / dy)))
    margin = 5

    # Precompute cumulative invalid counts so O(1) window checks skip nodata fast.
    invalid_prefix = np.zeros((height + 1, width + 1), dtype=np.int64)
    invalid_prefix[1:, 1:] = np.cumsum(np.cumsum(~valid, axis=0), axis=1)

    candidates = []
    for r0 in range(margin, height - patch_px_y - margin, stride_px_y):
        r1 = r0 + patch_px_y
        # Include a 4-px bicubic safety margin around the patch.
        mr0, mr1 = r0 - 4, r1 + 4
        for c0 in range(margin, width - patch_px_x - margin, stride_px_x):
            c1 = c0 + patch_px_x
            mc0, mc1 = c0 - 4, c1 + 4

            bad = (
                invalid_prefix[mr1, mc1]
                - invalid_prefix[mr0, mc1]
                - invalid_prefix[mr1, mc0]
                + invalid_prefix[mr0, mc0]
            )
            if bad > 0:
                continue

            patch = dtm[r0:r1, c0:c1]
            dz_dy, dz_dx = np.gradient(patch, dy, dx)
            slope = np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))

            p50 = float(np.percentile(slope, 50))
            if not (3.0 <= p50 <= 8.0):
                continue

            max_s = float(np.max(slope))
            pct_15_30 = float(np.mean((slope >= 15.0) & (slope <= 30.0)) * 100.0)
            pct_gt20 = float(np.mean(slope > 20.0) * 100.0)
            # Require steep terrain inside the central 60% so Nav2 routes around it,
            # while keeping the outer band mostly drivable for spawn/goal points.
            h, w = slope.shape
            r_lo, r_hi = int(0.2 * h), int(0.8 * h)
            c_lo, c_hi = int(0.2 * w), int(0.8 * w)
            interior = slope[r_lo:r_hi, c_lo:c_hi]
            interior_gt20 = float(np.mean(interior > 20.0) * 100.0)
            border_mask = np.ones_like(slope, dtype=bool)
            border_mask[r_lo:r_hi, c_lo:c_hi] = False
            border_free = float(np.mean(slope[border_mask] <= 10.0) * 100.0)

            if not (
                20.0 <= max_s <= 30.0
                and 3.0 <= pct_gt20 <= 16.0
                and pct_15_30 >= 5.0
                and interior_gt20 >= 3.0
                and border_free >= 65.0
            ):
                continue

            center_col = (c0 + c1) / 2.0
            center_row = (r0 + r1) / 2.0
            cx, cy = transform * (center_col, center_row)

            stats = {
                "center_x": round(float(cx), 2),
                "center_y": round(float(cy), 2),
                "mean_deg": round(float(np.mean(slope)), 2),
                "p50_deg": round(p50, 2),
                "p90_deg": round(float(np.percentile(slope, 90)), 2),
                "max_deg": round(max_s, 2),
                "pct_15_30": round(pct_15_30, 2),
                "pct_above_20": round(pct_gt20, 2),
                "interior_gt20": round(interior_gt20, 2),
                "z_range_m": round(float(np.max(patch) - np.min(patch)), 2),
            }
            stats["score"] = round(score_candidate(stats), 3)
            candidates.append(stats)

    candidates.sort(key=lambda item: item["score"])

    if ortho_path is None:
        return candidates[:top_k]

    verified = []
    with rasterio.open(ortho_path) as ortho_src:
        for cand in candidates:
            if _ortho_patch_valid(ortho_src, cand["center_x"], cand["center_y"], size_m):
                verified.append(cand)
                if len(verified) >= top_k:
                    break
    return verified


def parse_args(argv=None):
    """Parse CLI arguments for find_patch."""
    parser = argparse.ArgumentParser(
        description="Find candidate rover navigation patches in a HiRISE DTM."
    )
    parser.add_argument("--dtm", required=True, help="Path to DTM GeoTIFF or PDS .IMG")
    parser.add_argument(
        "--ortho", default=None, help="Optional orthoimage path to verify coverage"
    )
    parser.add_argument("--size", type=float, default=50.0, help="Patch size in metres")
    parser.add_argument(
        "--stride", type=float, default=50.0, help="Grid search stride in metres"
    )
    parser.add_argument("--top", type=int, default=10, help="Number of candidates to print")
    return parser.parse_args(argv)


def main(argv=None):
    """CLI entry point."""
    args = parse_args(argv)
    results = search_patches(
        dtm_path=args.dtm,
        size_m=args.size,
        stride_m=args.stride,
        ortho_path=args.ortho,
        top_k=args.top,
    )
    if not results:
        print("No matching patches found.")
        return

    header = (
        f"{'#':>2}  {'center_x':>11}  {'center_y':>11}  {'mean':>5}  {'p50':>5}  "
        f"{'p90':>5}  {'max':>5}  {'15-30%':>6}  {'>20%':>5}  {'in>20%':>6}  {'z_rng':>5}  {'score':>6}"
    )
    print(header)
    print("-" * len(header))
    for i, c in enumerate(results, start=1):
        print(
            f"{i:>2}  {c['center_x']:>11.2f}  {c['center_y']:>11.2f}  "
            f"{c['mean_deg']:>5.2f}  {c['p50_deg']:>5.2f}  {c['p90_deg']:>5.2f}  "
            f"{c['max_deg']:>5.2f}  {c['pct_15_30']:>6.2f}  {c['pct_above_20']:>5.2f}  "
            f"{c['interior_gt20']:>6.2f}  {c['z_range_m']:>5.2f}  {c['score']:>6.3f}"
        )


if __name__ == "__main__":
    main()
