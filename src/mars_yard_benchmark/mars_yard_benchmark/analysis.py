#!/usr/bin/env python3
"""Analyze Mars Yard benchmark CSV results into markdown summary tables and PNG plots.

Uses only stdlib csv + NumPy + Matplotlib so it runs identically in both system Python
(ROS 2 environment) and the project's .venv.
"""

import argparse
from collections import defaultdict
import csv
import math
import pathlib
from typing import Dict, List, Tuple

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


GROUPS_ORDER = ("flat", "slope", "rocks", "sand")
FAILURE_TYPES = ("timeout", "stuck", "collision", "tipover", "aborted", "goal_miss")
# Nav2 judges arrival on the EKF pose, which drifts on slopes; a run only counts as a true success
# if the ground-truth rover ends within this radius of the goal (~1 rover length, 4x the 0.25 m tolerance).
GOAL_MISS_M = 1.0
METRIC_KEYS = (
    ("time_s", "Time (s)"),
    ("path_ratio", "Path ratio"),
    ("tracking_err", "Tracking err (m)"),
    ("slip_ratio", "Slip ratio"),
    ("lateral_slide", "Lat slide (m)"),
    ("ekf_drift", "EKF drift (m)"),
    ("energy_J", "Energy (J)"),
    ("min_obstacle_clearance", "Min clr (m)"),
)


def _to_float(val: str) -> float:
    if val is None or str(val).strip() == "":
        return float("nan")
    try:
        return float(val)
    except ValueError:
        return float("nan")


def load_rows(csv_path: pathlib.Path) -> List[Dict]:
    rows: List[Dict] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            row = dict(raw)
            row["seed"] = int(raw["seed"])
            row["recoveries"] = int(float(raw.get("recoveries", 0) or 0))
            for k in (
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
                "rtf",
            ):
                row[k] = _to_float(raw.get(k, ""))
            row["nav2_result"] = row["result"]
            if row["result"] == "SUCCESS" and not row["gt_goal_err"] <= GOAL_MISS_M:
                row["result"], row["failure_type"] = "FAILURE", "goal_miss"
            rows.append(row)
    return rows


def compute_summary(rows: List[Dict]) -> Dict:
    combos = sorted({r["combo"] for r in rows})
    groups_present = [g for g in GROUPS_ORDER if any(r["group"] == g for r in rows)]
    for g in sorted({r["group"] for r in rows}):
        if g not in groups_present:
            groups_present.append(g)

    # Success rate per combo x group
    success_grid: Dict[Tuple[str, str], Tuple[int, int]] = {}
    combo_overall: Dict[str, Tuple[int, int]] = {}
    for c in combos:
        c_rows = [r for r in rows if r["combo"] == c]
        combo_overall[c] = (sum(1 for r in c_rows if r["result"] == "SUCCESS"), len(c_rows))
        for g in groups_present:
            cg = [r for r in c_rows if r["group"] == g]
            success_grid[(c, g)] = (sum(1 for r in cg if r["result"] == "SUCCESS"), len(cg))

    # Mean +/- std over successful runs per combo
    metric_stats: Dict[str, Dict[str, Tuple[float, float]]] = {}
    success_samples: Dict[str, Dict[str, List[float]]] = {}
    for c in combos:
        succ = [r for r in rows if r["combo"] == c and r["result"] == "SUCCESS"]
        metric_stats[c] = {}
        success_samples[c] = {}
        for key, _ in METRIC_KEYS:
            vals = [r[key] for r in succ if math.isfinite(r[key])]
            success_samples[c][key] = vals
            if vals:
                arr = np.asarray(vals, dtype=np.float64)
                mean_v = float(np.mean(arr))
                std_v = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
                metric_stats[c][key] = (mean_v, std_v)
            else:
                metric_stats[c][key] = (float("nan"), float("nan"))

    # Failure counts per combo
    failure_counts: Dict[str, Dict[str, int]] = {}
    for c in combos:
        counts = defaultdict(int)
        for r in rows:
            if r["combo"] == c and r["result"] != "SUCCESS":
                ft = r.get("failure_type", "aborted") or "aborted"
                counts[ft] += 1
        failure_counts[c] = {ft: int(counts[ft]) for ft in FAILURE_TYPES}
        other = sum(v for k, v in counts.items() if k not in FAILURE_TYPES)
        failure_counts[c]["other"] = int(other)
        failure_counts[c]["total"] = sum(counts.values())

    return {
        "total_runs": len(rows),
        "combos": combos,
        "groups": groups_present,
        "success_grid": success_grid,
        "combo_overall": combo_overall,
        "metric_stats": metric_stats,
        "success_samples": success_samples,
        "failure_counts": failure_counts,
    }


def _fmt_rate(ok: int, total: int) -> str:
    if total == 0:
        return "—"
    return f"{100.0 * ok / total:.1f}% ({ok}/{total})"


def _fmt_mean_std(mean_v: float, std_v: float, decimals: int = 2) -> str:
    if not math.isfinite(mean_v):
        return "—"
    return f"{mean_v:.{decimals}f} ± {std_v:.{decimals}f}"


def write_summary_markdown(summary: Dict, out_md: pathlib.Path) -> str:
    combos = summary["combos"]
    groups = summary["groups"]

    lines: List[str] = [
        "# Mars Yard Benchmark Summary",
        "",
        f"Total runs: **{summary['total_runs']}** across **{len(combos)}** planner × controller combo(s) "
        f"and **{len(groups)}** terrain group(s).",
        "",
        "## 1. Success rate (combo × terrain group)",
        "",
    ]

    header = ["Combo"] + [g.capitalize() for g in groups] + ["Overall"]
    lines.append("| " + " | ".join(header) + " |")
    lines.append("|" + "|".join(["---"] + ["---:"] * (len(groups) + 1)) + "|")
    for c in combos:
        cells = [f"`{c}`"]
        for g in groups:
            ok, tot = summary["success_grid"][(c, g)]
            cells.append(_fmt_rate(ok, tot))
        ok_all, tot_all = summary["combo_overall"][c]
        cells.append(f"**{_fmt_rate(ok_all, tot_all)}**")
        lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 2. Navigation metrics over successful runs (mean ± std)",
        "",
    ]
    m_header = ["Combo", "N ok"] + [label for _, label in METRIC_KEYS]
    lines.append("| " + " | ".join(m_header) + " |")
    lines.append("|" + "|".join(["---", "---:"] + ["---:"] * len(METRIC_KEYS)) + "|")
    decimals_map = {
        "time_s": 2,
        "path_ratio": 3,
        "tracking_err": 3,
        "slip_ratio": 3,
        "lateral_slide": 3,
        "ekf_drift": 3,
        "energy_J": 1,
        "min_obstacle_clearance": 2,
    }
    for c in combos:
        ok_all, _ = summary["combo_overall"][c]
        cells = [f"`{c}`", str(ok_all)]
        for key, _ in METRIC_KEYS:
            mv, sv = summary["metric_stats"][c][key]
            cells.append(_fmt_mean_std(mv, sv, decimals=decimals_map.get(key, 2)))
        lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 3. Failure counts by type",
        "",
    ]
    f_header = ["Combo"] + list(FAILURE_TYPES) + ["Total fail"]
    lines.append("| " + " | ".join(f_header) + " |")
    lines.append("|" + "|".join(["---"] + ["---:"] * (len(FAILURE_TYPES) + 1)) + "|")
    for c in combos:
        fc = summary["failure_counts"][c]
        cells = [f"`{c}`"] + [str(fc[ft]) for ft in FAILURE_TYPES] + [str(fc["total"])]
        lines.append("| " + " | ".join(cells) + " |")

    lines.append("")
    text = "\n".join(lines)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text(text, encoding="utf-8")
    return text


def build_plots(summary: Dict, plots_dir: pathlib.Path) -> List[pathlib.Path]:
    plots_dir.mkdir(parents=True, exist_ok=True)
    combos = summary["combos"]
    groups = summary["groups"]
    created: List[pathlib.Path] = []

    # 1. Success-rate heatmap (combo x group)
    mat = np.full((len(combos), len(groups)), np.nan, dtype=np.float64)
    annots = []
    for i, c in enumerate(combos):
        row_ann = []
        for j, g in enumerate(groups):
            ok, tot = summary["success_grid"][(c, g)]
            if tot > 0:
                mat[i, j] = 100.0 * ok / tot
                row_ann.append(f"{mat[i, j]:.0f}%\n({ok}/{tot})")
            else:
                row_ann.append("—")
        annots.append(row_ann)

    fig, ax = plt.subplots(figsize=(max(5.5, 1.4 * len(groups) + 2.0), max(3.5, 0.75 * len(combos) + 1.5)))
    im = ax.imshow(mat, cmap="RdYlGn", vmin=0.0, vmax=100.0, aspect="auto")
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels([g.capitalize() for g in groups])
    ax.set_yticks(range(len(combos)))
    ax.set_yticklabels(combos)
    ax.set_title("Success Rate (%) by Planner × Controller and Terrain Group")
    for i in range(len(combos)):
        for j in range(len(groups)):
            val = mat[i, j]
            color = "black" if (not math.isfinite(val) or 25.0 <= val <= 85.0) else "white"
            ax.text(j, i, annots[i][j], ha="center", va="center", color=color, fontsize=9, fontweight="bold")
    fig.colorbar(im, ax=ax, label="Success rate (%)")
    fig.tight_layout()
    heatmap_path = plots_dir / "success_heatmap.png"
    fig.savefig(heatmap_path, dpi=150)
    plt.close(fig)
    created.append(heatmap_path)

    # 2 & 3. Box plots of completion time and tracking error per combo (over successful runs)
    for key, ylabel, title, filename in (
        ("time_s", "Completion time (s)", "Completion Time per Combo (Successful Runs)", "time_boxplot.png"),
        (
            "tracking_err",
            "Mean tracking error to /plan (m)",
            "Path Tracking Error per Combo (Successful Runs)",
            "tracking_error_boxplot.png",
        ),
    ):
        data = []
        labels = []
        for c in combos:
            vals = summary["success_samples"][c][key]
            data.append(vals if vals else [np.nan])
            labels.append(f"{c}\n(n={len(vals)})")

        fig, ax = plt.subplots(figsize=(max(6.0, 1.35 * len(combos) + 1.5), 4.5))
        ax.boxplot(data, patch_artist=True)
        ax.set_xticks(range(1, len(labels) + 1))
        ax.set_xticklabels(labels, rotation=18, ha="right")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.grid(axis="y", linestyle="--", alpha=0.4)
        fig.tight_layout()
        out_path = plots_dir / filename
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        created.append(out_path)

    return created


def analyze_benchmark(
    csv_path: pathlib.Path,
    out_md: pathlib.Path,
    plots_dir: pathlib.Path,
) -> Tuple[Dict, List[pathlib.Path]]:
    rows = load_rows(csv_path)
    summary = compute_summary(rows)
    write_summary_markdown(summary, out_md)
    plot_paths = build_plots(summary, plots_dir)
    return summary, plot_paths


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Analyze Mars Yard benchmark CSV into tables and plots.")
    parser.add_argument("--csv", type=pathlib.Path, default=pathlib.Path("results/benchmark/benchmark.csv"))
    parser.add_argument("--out-md", type=pathlib.Path, default=pathlib.Path("results/benchmark/summary.md"))
    parser.add_argument("--plots-dir", type=pathlib.Path, default=pathlib.Path("results/benchmark/plots"))
    args = parser.parse_args(argv)

    summary, plot_paths = analyze_benchmark(args.csv, args.out_md, args.plots_dir)
    print(f"Analyzed {summary['total_runs']} rows from {args.csv}")
    print(f"Wrote summary: {args.out_md}")
    for p in plot_paths:
        print(f"Wrote plot   : {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
