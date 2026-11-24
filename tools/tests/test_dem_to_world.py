"""Unit tests for tools/dem_to_world.py using synthetic GeoTIFFs."""

import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image
import pytest
import rasterio
from rasterio.crs import CRS
from rasterio.transform import from_origin
import yaml

TOOLS_DIR = Path(__file__).resolve().parents[1]
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))

import dem_to_world


def _write_synthetic_geotiff(path, elevation, origin_x, origin_y, pixel_size=1.0, nodata=-9999.0):
    """Write a float32 GeoTIFF with north-up transform in a metric CRS."""
    height, width = elevation.shape
    transform = from_origin(origin_x, origin_y, pixel_size, pixel_size)
    with rasterio.open(
        path,
        "w",
        driver="GTiff",
        height=height,
        width=width,
        count=1,
        dtype="float32",
        crs=CRS.from_epsg(32633),
        transform=transform,
        nodata=nodata,
    ) as dst:
        dst.write(elevation.astype(np.float32), 1)


def _read_pgm(path):
    """Read a binary P5 PGM file and return a uint8 2D numpy array."""
    with open(path, "rb") as f:
        magic = f.readline().strip()
        assert magic == b"P5"
        line = f.readline().strip()
        while line.startswith(b"#"):
            line = f.readline().strip()
        width, height = map(int, line.split())
        maxval = int(f.readline().strip())
        assert maxval == 255
        raw = f.read()
    return np.frombuffer(raw, dtype=np.uint8).reshape((height, width))


def test_tilted_plane_slope_heightmap_and_z_range(tmp_path):
    """Verify slope, 16-bit PNG span, z_range_m, and metadata on a 10-deg plane."""
    # Build a 100x100 m GeoTIFF at 1 m/px tilted 10 deg upward toward +Y (north).
    width, height = 100, 100
    origin_x, origin_y = 1000.0, 2100.0
    pixel_size = 1.0
    tilt_deg = 10.0

    rows = np.arange(height, dtype=np.float64)
    y_coords = origin_y - (rows + 0.5) * pixel_size
    elevation = np.repeat((y_coords * math.tan(math.radians(tilt_deg)))[:, None], width, axis=1)

    dtm_path = tmp_path / "tilted_10deg.tif"
    _write_synthetic_geotiff(dtm_path, elevation, origin_x, origin_y, pixel_size=pixel_size)

    # Matching synthetic orthoimage at 0.5 m/px (200x200).
    ortho_path = tmp_path / "ortho.tif"
    ortho_data = np.tile(np.linspace(0, 255, 200, dtype=np.uint8)[:, None], (1, 200))
    with rasterio.open(
        ortho_path,
        "w",
        driver="GTiff",
        height=200,
        width=200,
        count=1,
        dtype="uint8",
        crs=CRS.from_epsg(32633),
        transform=from_origin(origin_x, origin_y, 0.5, 0.5),
    ) as dst:
        dst.write(ortho_data, 1)

    out_dir = tmp_path / "out"
    size_m = 50.0
    res_m = 0.5
    meta = dem_to_world.run_pipeline(
        dtm_path=dtm_path,
        ortho_path=ortho_path,
        center_x=1050.0,
        center_y=2050.0,
        size_m=size_m,
        res_m=res_m,
        free_slope_deg=10.0,
        lethal_slope_deg=20.0,
        out_dir=out_dir,
    )

    # Slope should match 10 +/- 0.5 deg across the patch.
    stats = meta["slope_stats"]
    assert abs(stats["mean_deg"] - tilt_deg) <= 0.5
    assert abs(stats["p50_deg"] - tilt_deg) <= 0.5
    assert abs(stats["max_deg"] - tilt_deg) <= 0.5

    # Expected z_range between northernmost and southernmost pixel centres.
    expected_z_range = (size_m - res_m) * math.tan(math.radians(tilt_deg))
    assert abs(meta["z_range_m"] - expected_z_range) <= 0.05

    # Heightmap PNG must be 16-bit grayscale spanning 0..65535 with north at row 0.
    hmap = np.array(Image.open(out_dir / "heightmap.png"))
    assert hmap.dtype == np.uint16 or hmap.max() == 65535
    assert int(hmap.min()) == 0
    assert int(hmap.max()) == 65535
    assert int(hmap[0, 0]) == 65535
    assert int(hmap[-1, 0]) == 0

    # Texture PNG must be 8-bit and <= 2048 px per side.
    tex = np.array(Image.open(out_dir / "texture.png"))
    assert tex.dtype == np.uint8
    assert max(tex.shape[:2]) <= 2048

    # terrain.json and slope.yaml must match Nav2 scale convention.
    loaded_json = json.loads((out_dir / "terrain.json").read_text(encoding="utf-8"))
    assert loaded_json["size_x_m"] == size_m
    assert loaded_json["size_y_m"] == size_m
    assert loaded_json["res_m"] == res_m

    loaded_yaml = yaml.safe_load((out_dir / "slope.yaml").read_text(encoding="utf-8"))
    assert loaded_yaml["mode"] == "scale"
    assert loaded_yaml["image"] == "slope.pgm"
    assert loaded_yaml["origin"] == [-25.0, -25.0, 0.0]
    assert loaded_yaml["negate"] == 0
    assert loaded_yaml["occupied_thresh"] == 0.65
    assert loaded_yaml["free_thresh"] == 0.25


def test_pgm_north_edge_in_row_zero_on_y_tilted_surface(tmp_path):
    """Verify PGM row 0 corresponds to the north (max-y) edge on a y-varying ramp."""
    # Elevation is quadratic in y so slope increases smoothly toward +Y (north):
    # south half has slope < 5 deg (free -> 254), north half has slope > 25 deg (lethal -> 0).
    width, height = 100, 100
    origin_x, origin_y = 0.0, 100.0
    pixel_size = 1.0

    rows = np.arange(height, dtype=np.float64)
    y_coords = origin_y - (rows + 0.5) * pixel_size  # 99.5 (north) down to 0.5 (south)
    # dz/dy = 0.008 * y -> at y=25: 0.20 (11.3 deg); at y=75: 0.60 (31.0 deg).
    # Shift y so y=25..35 is nearly flat (<5 deg) and y=65..75 is steep (>25 deg).
    y_rel = np.maximum(y_coords - 30.0, 0.0)
    z_profile = 0.007 * (y_rel ** 2)
    elevation = np.repeat(z_profile[:, None], width, axis=1)

    dtm_path = tmp_path / "y_ramp.tif"
    _write_synthetic_geotiff(dtm_path, elevation, origin_x, origin_y, pixel_size=pixel_size)

    out_dir = tmp_path / "out_ramp"
    dem_to_world.run_pipeline(
        dtm_path=dtm_path,
        center_x=50.0,
        center_y=50.0,
        size_m=50.0,
        res_m=0.5,
        free_slope_deg=10.0,
        lethal_slope_deg=20.0,
        out_dir=out_dir,
    )

    pgm = _read_pgm(out_dir / "slope.pgm")
    assert pgm.shape == (100, 100)
    # North edge (row 0, y ~ 75 m) has slope ~32 deg -> lethal (pixel 0).
    assert int(np.median(pgm[0, :])) == 0
    # South edge (row -1, y ~ 25 m) is flat (0 deg) -> free (pixel 254).
    assert int(np.median(pgm[-1, :])) == 254


def test_nodata_crop_raises_value_error(tmp_path):
    """Ensure a crop containing nodata pixels fails cleanly."""
    elevation = np.zeros((60, 60), dtype=np.float32)
    elevation[30, 30] = -9999.0
    dtm_path = tmp_path / "with_nodata.tif"
    _write_synthetic_geotiff(dtm_path, elevation, 0.0, 60.0, pixel_size=1.0, nodata=-9999.0)

    with pytest.raises(ValueError, match="nodata"):
        dem_to_world.run_pipeline(
            dtm_path=dtm_path,
            center_x=30.0,
            center_y=30.0,
            size_m=20.0,
            res_m=0.5,
            out_dir=tmp_path / "out_err",
        )
