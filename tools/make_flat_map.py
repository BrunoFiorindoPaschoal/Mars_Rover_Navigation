#!/usr/bin/env python3
"""Generate an all-free 50x50 m Nav2 map matching the slope.pgm/slope.yaml layout."""

import argparse
from pathlib import Path
import numpy as np


def write_flat_map(out_dir, size_m=50.0, res_m=0.1, stem="flat"):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    n_px = int(round(size_m / res_m))
    actual_size = n_px * res_m

    pgm_path = out_dir / f"{stem}.pgm"
    yaml_path = out_dir / f"{stem}.yaml"

    pgm_array = np.full((n_px, n_px), 254, dtype=np.uint8)
    header = f"P5\n{n_px} {n_px}\n255\n".encode("ascii")
    with open(pgm_path, "wb") as f:
        f.write(header)
        f.write(pgm_array.tobytes())

    origin = [round(-actual_size / 2.0, 6), round(-actual_size / 2.0, 6), 0.0]
    yaml_lines = [
        f"image: {stem}.pgm",
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


def main():
    parser = argparse.ArgumentParser(description="Generate all-free flat Nav2 map.")
    parser.add_argument("--size", type=float, default=50.0)
    parser.add_argument("--res", type=float, default=0.1)
    parser.add_argument("--out", default="src/mars_yard_worlds/terrain")
    args = parser.parse_args()
    pgm, yml = write_flat_map(args.out, size_m=args.size, res_m=args.res)
    print(f"Wrote {pgm} and {yml}")


if __name__ == "__main__":
    main()
