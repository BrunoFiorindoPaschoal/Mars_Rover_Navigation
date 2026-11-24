"""Convert a HiRISE DTM and orthoimage patch into MuJoCo and Nav2 assets."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image
import rasterio
from rasterio.enums import Resampling
from rasterio.transform import from_origin
from rasterio.warp import reproject
from rasterio.windows import from_bounds


def _is_nodata_mask(data, nodata):
    """Return a boolean mask of nodata or non-finite pixels."""
    mask = ~np.isfinite(data)
    if nodata is not None and np.isfinite(nodata):
        if np.issubdtype(data.dtype, np.floating):
            mask |= np.isclose(data, nodata, rtol=1e-5, atol=1e-5)
        else:
            mask |= data == nodata
    return mask


def _padded_window(src, west, south, east, north, pad_px=4):
    """Build a source pixel window covering bounds with a clamped margin."""
    win = from_bounds(west, south, east, north, transform=src.transform)
    col_off = max(0, int(np.floor(win.col_off)) - pad_px)
    row_off = max(0, int(np.floor(win.row_off)) - pad_px)
    col_end = min(src.width, int(np.ceil(win.col_off + win.width)) + pad_px)
    row_end = min(src.height, int(np.ceil(win.row_off + win.height)) + pad_px)
    return rasterio.windows.Window(
        col_off=col_off,
        row_off=row_off,
        width=col_end - col_off,
        height=row_end - row_off,
    )


def _check_within_bounds(src, west, south, east, north, label):
    """Raise ValueError if the requested crop lies outside raster bounds."""
    b = src.bounds
    if west < b.left or east > b.right or south < b.bottom or north > b.top:
        raise ValueError(
            f"{label} crop ({west:.2f}, {south:.2f}, {east:.2f}, {north:.2f}) "
            f"exceeds raster bounds ({b.left:.2f}, {b.bottom:.2f}, "
            f"{b.right:.2f}, {b.top:.2f})"
        )


def crop_and_resample_dem(dtm_path, center_x, center_y, size_m, res_m):
    """Crop a square DTM patch and bicubically resample it to res_m."""
    n_px = int(round(size_m / res_m))
    if n_px < 2:
        raise ValueError("Patch size must be at least 2 pixels wide.")

    actual_size = n_px * res_m
    half = actual_size / 2.0
    west, east = center_x - half, center_x + half
    south, north = center_y - half, center_y + half

    with rasterio.open(dtm_path) as src:
        _check_within_bounds(src, west, south, east, north, "DTM")

        # Check exact source crop for nodata before resampling.
        exact_win = from_bounds(west, south, east, north, transform=src.transform)
        exact_win = exact_win.round_offsets().round_lengths()
        exact_raw = src.read(1, window=exact_win).astype(np.float64)
        if np.any(_is_nodata_mask(exact_raw, src.nodata)):
            raise ValueError("Cropped DTM patch contains nodata pixels.")

        # Read a small margin around the crop so bicubic kernels have full support.
        pad_win = _padded_window(src, west, south, east, north, pad_px=4)
        src_arr = src.read(1, window=pad_win).astype(np.float32)
        src_arr[_is_nodata_mask(src_arr, src.nodata)] = np.nan
        src_transform = src.window_transform(pad_win)
        crs_str = src.crs.to_string() if src.crs is not None else "UNKNOWN"
        crs_obj = src.crs

    dst_transform = from_origin(west, north, res_m, res_m)
    dem = np.full((n_px, n_px), np.nan, dtype=np.float32)

    # HiRISE DTMs are ~1 m/px, so resampling to 0.1 m/px is bicubic upsampling.
    reproject(
        source=src_arr,
        destination=dem,
        src_transform=src_transform,
        src_crs=crs_obj,
        dst_transform=dst_transform,
        dst_crs=crs_obj,
        src_nodata=np.nan,
        dst_nodata=np.nan,
        resampling=Resampling.cubic,
    )

    if not np.all(np.isfinite(dem)):
        raise ValueError("Resampled DTM patch contains non-finite values.")

    return dem.astype(np.float64), crs_str, Path(dtm_path).stem


def crop_ortho_texture(ortho_path, center_x, center_y, size_m, max_px=2048):
    """Crop the matching orthoimage region as an 8-bit array capped at max_px."""
    half = size_m / 2.0
    west, east = center_x - half, center_x + half
    south, north = center_y - half, center_y + half

    with rasterio.open(ortho_path) as src:
        _check_within_bounds(src, west, south, east, north, "Orthoimage")

        native_res = min(abs(src.transform.a), abs(src.transform.e))
        native_px = max(1, int(round(size_m / native_res)))
        out_px = min(native_px, max_px)
        out_res = size_m / out_px

        pad_win = _padded_window(src, west, south, east, north, pad_px=4)
        bands = 3 if src.count >= 3 else 1
        src_arr = src.read(list(range(1, bands + 1)), window=pad_win)
        src_transform = src.window_transform(pad_win)
        crs_obj = src.crs
        src_nodata = src.nodata

    dst_transform = from_origin(west, north, out_res, out_res)
    resampling = Resampling.average if out_px < native_px else Resampling.cubic
    dst_arr = np.zeros((bands, out_px, out_px), dtype=np.float32)

    for idx in range(bands):
        reproject(
            source=src_arr[idx].astype(np.float32),
            destination=dst_arr[idx],
            src_transform=src_transform,
            src_crs=crs_obj,
            dst_transform=dst_transform,
            dst_crs=crs_obj,
            src_nodata=src_nodata,
            dst_nodata=0.0,
            resampling=resampling,
        )

    if src_arr.dtype == np.uint8:
        tex = np.clip(np.round(dst_arr), 0, 255).astype(np.uint8)
    else:
        lo, hi = float(np.nanmin(dst_arr)), float(np.nanmax(dst_arr))
        if hi > lo:
            tex = np.clip(np.round((dst_arr - lo) / (hi - lo) * 255.0), 0, 255).astype(
                np.uint8
            )
        else:
            tex = np.zeros_like(dst_arr, dtype=np.uint8)

    if bands == 1:
        tex = tex[0]
    else:
        tex = np.transpose(tex, (1, 2, 0))

    return tex, Path(ortho_path).stem


def write_heightmap_png(dem, out_path):
    """Subtract minimum elevation and write a 16-bit grayscale PNG heightmap."""
    z_min = float(np.min(dem))
    z_max = float(np.max(dem))
    z_range = z_max - z_min

    if z_range > 0.0:
        scaled = np.round((dem - z_min) / z_range * 65535.0)
        height_u16 = np.clip(scaled, 0, 65535).astype(np.uint16)
    else:
        height_u16 = np.zeros(dem.shape, dtype=np.uint16)

    Image.fromarray(height_u16).save(out_path)
    return z_min, z_max, z_range


def write_texture_png(texture, out_path):
    """Write an 8-bit grayscale or RGB texture PNG."""
    Image.fromarray(texture).save(out_path)


def compute_slope_deg(dem, res_m):
    """Compute terrain slope in degrees using numpy gradient and real spacing."""
    dz_dy, dz_dx = np.gradient(dem, res_m, res_m)
    return np.degrees(np.arctan(np.hypot(dz_dx, dz_dy)))


def slope_to_nav2_pgm(slope_deg, free_slope_deg=10.0, lethal_slope_deg=20.0):
    """Map slope in degrees to Nav2 PGM values (254 = free, 0 = lethal)."""
    if lethal_slope_deg <= free_slope_deg:
        raise ValueError("lethal_slope_deg must be greater than free_slope_deg.")

    frac = (slope_deg - free_slope_deg) / (lethal_slope_deg - free_slope_deg)
    frac = np.clip(frac, 0.0, 1.0)
    return np.round((1.0 - frac) * 254.0).astype(np.uint8)


def write_nav2_map(pgm_array, out_dir, size_x_m, size_y_m, res_m):
    """Write Nav2 slope.pgm (row 0 = north) and slope.yaml with mode: scale."""
    out_dir = Path(out_dir)
    pgm_path = out_dir / "slope.pgm"
    yaml_path = out_dir / "slope.yaml"

    height, width = pgm_array.shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with open(pgm_path, "wb") as f:
        f.write(header)
        f.write(pgm_array.tobytes())

    origin = [round(-size_x_m / 2.0, 6), round(-size_y_m / 2.0, 6), 0.0]
    yaml_lines = [
        "image: slope.pgm",
        "mode: scale",
        f"resolution: {float(res_m)}",
        f"origin: [{origin[0]}, {origin[1]}, {origin[2]}]",
        "negate: 0",
        "occupied_thresh: 0.65",
        "free_thresh: 0.25",
        "",
    ]
    yaml_path.write_text("\n".join(yaml_lines), encoding="utf-8")

    return pgm_path, yaml_path


def compute_slope_stats(slope_deg):
    """Compute summary slope statistics for terrain.json."""
    return {
        "mean_deg": round(float(np.mean(slope_deg)), 4),
        "p50_deg": round(float(np.percentile(slope_deg, 50)), 4),
        "p90_deg": round(float(np.percentile(slope_deg, 90)), 4),
        "max_deg": round(float(np.max(slope_deg)), 4),
        "pct_above_20_deg": round(float(np.mean(slope_deg > 20.0) * 100.0), 4),
    }


def write_terrain_json(out_path, metadata):
    """Write terrain metadata dictionary to a formatted JSON file."""
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(metadata, f, indent=2)
        f.write("\n")


def save_preview_figure(
    dem, texture, slope_deg, size_m, free_slope_deg, lethal_slope_deg, out_path
):
    """Save a 3-panel matplotlib preview: heightmap, ortho texture, and slope."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    half = size_m / 2.0
    extent = [-half, half, -half, half]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8), constrained_layout=True)

    rel_dem = dem - float(np.min(dem))
    im0 = axes[0].imshow(rel_dem, cmap="terrain", extent=extent, origin="upper")
    axes[0].set_title(f"Heightmap (z range: {rel_dem.max():.2f} m)")
    axes[0].set_xlabel("X (m)")
    axes[0].set_ylabel("Y (m)")
    fig.colorbar(im0, ax=axes[0], shrink=0.82, label="Relative height (m)")

    if texture is not None:
        cmap = "gray" if texture.ndim == 2 else None
        axes[1].imshow(texture, cmap=cmap, extent=extent, origin="upper")
        axes[1].set_title(f"Orthoimage Texture ({texture.shape[1]}x{texture.shape[0]} px)")
    else:
        axes[1].text(0.5, 0.5, "No orthoimage", ha="center", va="center")
        axes[1].set_title("Orthoimage Texture")
    axes[1].set_xlabel("X (m)")
    axes[1].set_ylabel("Y (m)")

    vmax = max(lethal_slope_deg + 10.0, float(np.percentile(slope_deg, 99)))
    im2 = axes[2].imshow(
        slope_deg, cmap="inferno", vmin=0.0, vmax=vmax, extent=extent, origin="upper"
    )
    axes[2].contour(
        slope_deg,
        levels=[free_slope_deg, lethal_slope_deg],
        colors=["cyan", "lime"],
        linewidths=0.9,
        extent=extent,
        origin="upper",
    )
    stats = compute_slope_stats(slope_deg)
    axes[2].set_title(
        f"Slope (p50={stats['p50_deg']:.1f}°, p90={stats['p90_deg']:.1f}°, "
        f">20°: {stats['pct_above_20_deg']:.1f}%)"
    )
    axes[2].set_xlabel("X (m)")
    axes[2].set_ylabel("Y (m)")
    fig.colorbar(im2, ax=axes[2], shrink=0.82, label="Slope (deg)")

    fig.savefig(out_path, dpi=150)
    plt.close(fig)


def run_pipeline(
    dtm_path,
    center_x,
    center_y,
    size_m=50.0,
    res_m=0.1,
    ortho_path=None,
    free_slope_deg=10.0,
    lethal_slope_deg=20.0,
    out_dir="src/mars_yard_worlds/terrain",
    preview_path=None,
):
    """Run the full DTM-to-world conversion pipeline and return metadata."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    dem, crs_str, dtm_id = crop_and_resample_dem(
        dtm_path, center_x, center_y, size_m, res_m
    )
    actual_size = dem.shape[0] * res_m

    z_min, z_max, z_range = write_heightmap_png(dem, out_dir / "heightmap.png")

    texture = None
    ortho_id = None
    if ortho_path is not None:
        texture, ortho_id = crop_ortho_texture(
            ortho_path, center_x, center_y, actual_size, max_px=2048
        )
        write_texture_png(texture, out_dir / "texture.png")

    slope_deg = compute_slope_deg(dem, res_m)
    pgm_array = slope_to_nav2_pgm(slope_deg, free_slope_deg, lethal_slope_deg)
    write_nav2_map(pgm_array, out_dir, actual_size, actual_size, res_m)

    slope_stats = compute_slope_stats(slope_deg)
    metadata = {
        "size_x_m": round(float(actual_size), 6),
        "size_y_m": round(float(actual_size), 6),
        "z_min_m": round(z_min, 4),
        "z_max_m": round(z_max, 4),
        "z_range_m": round(z_range, 4),
        "res_m": float(res_m),
        "center_x_m": float(center_x),
        "center_y_m": float(center_y),
        "crs": crs_str,
        "source_products": {
            "dtm": dtm_id,
            "ortho": ortho_id,
        },
        "slope_thresholds_deg": {
            "free": float(free_slope_deg),
            "lethal": float(lethal_slope_deg),
        },
        "slope_stats": slope_stats,
    }
    write_terrain_json(out_dir / "terrain.json", metadata)

    if preview_path is not None:
        save_preview_figure(
            dem,
            texture,
            slope_deg,
            actual_size,
            free_slope_deg,
            lethal_slope_deg,
            preview_path,
        )

    return metadata


def parse_args(argv=None):
    """Parse CLI arguments for dem_to_world."""
    parser = argparse.ArgumentParser(
        description="Crop a HiRISE DTM + orthoimage into MuJoCo heightmap and Nav2 slope map."
    )
    parser.add_argument("--dtm", required=True, help="Path to DTM GeoTIFF or PDS .IMG")
    parser.add_argument("--ortho", default=None, help="Path to matching orthoimage")
    parser.add_argument(
        "--center-x", type=float, required=True, help="Crop center X in raster CRS (m)"
    )
    parser.add_argument(
        "--center-y", type=float, required=True, help="Crop center Y in raster CRS (m)"
    )
    parser.add_argument(
        "--size", type=float, default=50.0, help="Square patch side length in metres"
    )
    parser.add_argument(
        "--res", type=float, default=0.1, help="Output DEM/map resolution in m/px"
    )
    parser.add_argument(
        "--free-slope",
        type=float,
        default=10.0,
        help="Slope <= this (deg) maps to Nav2 free (254)",
    )
    parser.add_argument(
        "--lethal-slope",
        type=float,
        default=20.0,
        help="Slope >= this (deg) maps to Nav2 lethal (0)",
    )
    parser.add_argument(
        "--out",
        default="src/mars_yard_worlds/terrain",
        help="Output folder for terrain assets",
    )
    parser.add_argument(
        "--preview",
        default=None,
        help="Optional output path for a 3-panel preview PNG",
    )
    return parser.parse_args(argv)


def main(argv=None):
    """CLI entry point."""
    args = parse_args(argv)
    meta = run_pipeline(
        dtm_path=args.dtm,
        center_x=args.center_x,
        center_y=args.center_y,
        size_m=args.size,
        res_m=args.res,
        ortho_path=args.ortho,
        free_slope_deg=args.free_slope,
        lethal_slope_deg=args.lethal_slope,
        out_dir=args.out,
        preview_path=args.preview,
    )
    print(json.dumps(meta, indent=2))


if __name__ == "__main__":
    main()
