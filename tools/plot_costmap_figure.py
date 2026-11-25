"""README figure next to the hero GIF: what the lidar put into Nav2's global costmap, and how the plan reacted.

    .venv/bin/python tools/plot_costmap_figure.py media/hero/costmap.npz --out media/costmap_rocks_01.png

Input is tools/bag_to_costmap.py output. Left: the costmap at the first update after the goal, the first plan
(made before the boulders were marked) and the replan around them. Right: the costmap at the goal and the true
driven path. True rock outlines from obstacles.json are drawn on both, to show the detections line up.
"""
import argparse
import json
import pathlib

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

ROOT = pathlib.Path(__file__).resolve().parents[1]
OBSTACLES = ROOT / "src/mars_yard_worlds/terrain/obstacles.json"
PAD = 2.0  # metres of map around the route
BG, FG = "#141414", "#e8e8e8"


def plan_at(d, k):
    ends = np.cumsum(d["plan_len"])
    return d["plan_xy"][ends[k] - d["plan_len"][k]:ends[k]]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("costmap", type=pathlib.Path)
    parser.add_argument("--out", type=pathlib.Path, required=True)
    args = parser.parse_args()
    d = np.load(args.costmap)
    t0 = d["gt_t"][0]
    ox, oy = d["cm_origin"]
    res = float(d["cm_res"])
    h, w = d["cm"].shape[1:]
    extent = (ox, ox + w * res, oy, oy + h * res)

    first_plan = plan_at(d, 0)
    goal = first_plan[-1]
    after = np.flatnonzero(d["cm_t"] > d["plan_t"][0])
    k_early, k_end = after[0], len(d["cm_t"]) - 1
    j = np.searchsorted(d["plan_t"], d["cm_t"][k_early], "right") - 1  # plan in force at that costmap
    assert j > 0, "no replan before the first costmap update; pick a later costmap"
    replan = plan_at(d, j)

    route = np.vstack([d["gt_pos"], first_plan])
    lo, hi = route.min(0) - PAD, route.max(0) + PAD
    rocks = [r for r in json.loads(OBSTACLES.read_text())["rocks"]
             if lo[0] - 1 < r["x"] < hi[0] + 1 and lo[1] - 1 < r["y"] < hi[1] + 1]

    plt.rcParams.update({"text.color": FG, "axes.labelcolor": FG, "xtick.color": FG, "ytick.color": FG,
                         "axes.edgecolor": "#555555", "font.size": 11})
    fig, axes = plt.subplots(1, 2, figsize=(11, 7.6), facecolor=BG, constrained_layout=True)
    gt_end = d["gt_t"] <= d["cm_t"][k_end]
    panels = (
        (axes[0], k_early, f"t = {d['cm_t'][k_early] - t0:.1f} s: lidar marks the boulders, Nav2 replans"),
        (axes[1], k_end, f"t = {d['cm_t'][k_end] - t0:.0f} s: at the goal"),
    )
    for ax, k, title in panels:
        ax.set_facecolor(BG)
        im = ax.imshow(np.ma.masked_less(d["cm"][k], 0), origin="lower", extent=extent, cmap="inferno",
                       vmin=0, vmax=100, interpolation="nearest")
        for r in rocks:
            ax.add_patch(Circle((r["x"], r["y"]), max(r["size"][:2]), fill=False, ec="#7CFC00", lw=1.2, ls="--"))
        ax.plot(*goal, marker="*", ms=16, color="#ff5a1f", mec="white", zorder=5)
        ax.set_xlim(lo[0], hi[0])
        ax.set_ylim(lo[1], hi[1])
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=12)
        ax.set_xlabel("x (m)")
    ax0, ax1 = axes
    ax0.plot(*first_plan.T, color="#00e5ff", lw=1.6, ls=(0, (4, 3)), alpha=0.8, label="first plan (t = 0 s)")
    ax0.plot(*replan.T, color="#00e5ff", lw=2.6, label="replan")
    ax0.plot(*d["gt_pos"][0], "o", color="white", ms=7, label="rover start")
    ax0.set_ylabel("y (m)")
    ax1.plot(*d["gt_pos"][gt_end].T, color="white", lw=2.2, label="true driven path")
    ax1.plot([], [], ls="--", color="#7CFC00", label="true rock outline")
    ax1.plot([], [], "*", ms=12, color="#ff5a1f", label="goal")
    for ax in axes:
        ax.legend(loc="upper right", fontsize=9, facecolor=BG, edgecolor="#555555", labelcolor=FG)
    cb = fig.colorbar(im, ax=axes, shrink=0.8, pad=0.01)
    cb.set_label("Nav2 global cost (lethal = 100; halo = inflation)")
    fig.suptitle("What the lidar put into Nav2's costmap  (rocks_01, Smac Hybrid-A* + DWB)", fontsize=14)
    fig.savefig(args.out, dpi=110, facecolor=BG)
    print(f"wrote {args.out}: costmaps at {d['cm_t'][k_early] - t0:.1f} s and {d['cm_t'][k_end] - t0:.1f} s, "
          f"{len(rocks)} rocks in view")


if __name__ == "__main__":
    main()
