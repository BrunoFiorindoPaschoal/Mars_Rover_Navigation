#!/usr/bin/env python3
"""Analyze Mars Yard Phase 7 robustness CSV (levels 0-3) into summary tables and PNG plots.

Applies the Phase 6 ground-truth success rule: a run counts as SUCCESS only if Nav2
reported SUCCEEDED AND gt_goal_err <= GOAL_MISS_M (1.0 m); otherwise it is reclassified
as FAILURE / goal_miss.

Outputs:
  - summary.md
  - success_vs_level.png
  - gt_err_vs_level.png
  - energy_vs_level.png
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

from mars_yard_benchmark.analysis import FAILURE_TYPES, GOAL_MISS_M, GROUPS_ORDER, _to_float


def load_robustness_rows(csv_path: pathlib.Path) -> List[Dict]:
    rows: List[Dict] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            row = dict(raw)
            row["level"] = int(raw["level"])
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
            if row["result"] == "SUCCESS" and not (row["gt_goal_err"] <= GOAL_MISS_M):
                row["result"] = "FAILURE"
                row["failure_type"] = "goal_miss"
            rows.append(row)
    return rows


def _mean_std(vals: List[float]) -> Tuple[float, float]:
    clean = [v for v in vals if math.isfinite(v)]
    if not clean:
        return float("nan"), float("nan")
    arr = np.asarray(clean, dtype=np.float64)
    mean_v = float(np.mean(arr))
    std_v = float(np.std(arr, ddof=1)) if arr.size > 1 else 0.0
    return mean_v, std_v


def compute_robustness_summary(rows: List[Dict]) -> Dict:
    combos = sorted({r["combo"] for r in rows})
    levels = sorted({r["level"] for r in rows})
    groups = [g for g in GROUPS_ORDER if any(r["group"] == g for r in rows)]
    for g in sorted({r["group"] for r in rows}):
        if g not in groups:
            groups.append(g)

    success_by_cl: Dict[Tuple[str, int], Tuple[int, int]] = {}
    nav2_success_by_cl: Dict[Tuple[str, int], Tuple[int, int]] = {}
    gt_err_all: Dict[Tuple[str, int], Tuple[float, float]] = {}
    gt_err_ok: Dict[Tuple[str, int], Tuple[float, float]] = {}
    energy_all: Dict[Tuple[str, int], Tuple[float, float]] = {}
    energy_ok: Dict[Tuple[str, int], Tuple[float, float]] = {}
    ekf_drift_all: Dict[Tuple[str, int], Tuple[float, float]] = {}
    time_all: Dict[Tuple[str, int], Tuple[float, float]] = {}
    failures_by_cl: Dict[Tuple[str, int], Dict[str, int]] = {}
    group_success: Dict[Tuple[str, int, str], Tuple[int, int]] = {}

    for c in combos:
        for lv in levels:
            cl_rows = [r for r in rows if r["combo"] == c and r["level"] == lv]
            ok_rows = [r for r in cl_rows if r["result"] == "SUCCESS"]
            nav2_ok = sum(1 for r in cl_rows if r["nav2_result"] == "SUCCESS")

            success_by_cl[(c, lv)] = (len(ok_rows), len(cl_rows))
            nav2_success_by_cl[(c, lv)] = (nav2_ok, len(cl_rows))

            gt_err_all[(c, lv)] = _mean_std([r["gt_goal_err"] for r in cl_rows])
            gt_err_ok[(c, lv)] = _mean_std([r["gt_goal_err"] for r in ok_rows])
            energy_all[(c, lv)] = _mean_std([r["energy_J"] for r in cl_rows])
            energy_ok[(c, lv)] = _mean_std([r["energy_J"] for r in ok_rows])
            ekf_drift_all[(c, lv)] = _mean_std([r["ekf_drift"] for r in cl_rows])
            time_all[(c, lv)] = _mean_std([r["time_s"] for r in cl_rows])

            counts = defaultdict(int)
            for r in cl_rows:
                if r["result"] != "SUCCESS":
                    ft = r.get("failure_type", "aborted") or "aborted"
                    counts[ft] += 1
            fc = {ft: int(counts[ft]) for ft in FAILURE_TYPES}
            fc["other"] = int(sum(v for k, v in counts.items() if k not in FAILURE_TYPES))
            fc["total"] = int(sum(counts.values()))
            failures_by_cl[(c, lv)] = fc

            for g in groups:
                g_rows = [r for r in cl_rows if r["group"] == g]
                g_ok = sum(1 for r in g_rows if r["result"] == "SUCCESS")
                group_success[(c, lv, g)] = (g_ok, len(g_rows))

    return {
        "total_runs": len(rows),
        "combos": combos,
        "levels": levels,
        "groups": groups,
        "goal_miss_m": GOAL_MISS_M,
        "success_by_cl": success_by_cl,
        "nav2_success_by_cl": nav2_success_by_cl,
        "gt_err_all": gt_err_all,
        "gt_err_ok": gt_err_ok,
        "energy_all": energy_all,
        "energy_ok": energy_ok,
        "ekf_drift_all": ekf_drift_all,
        "time_all": time_all,
        "failures_by_cl": failures_by_cl,
        "group_success": group_success,
    }


def _fmt_rate(ok: int, tot: int) -> str:
    if tot == 0:
        return "—"
    return f"{100.0 * ok / tot:.1f}% ({ok}/{tot})"


def _fmt_ms(mean_v: float, std_v: float, dec: int = 2) -> str:
    if not math.isfinite(mean_v):
        return "—"
    return f"{mean_v:.{dec}f} ± {std_v:.{dec}f}"


def write_robustness_markdown(summary: Dict, out_md: pathlib.Path) -> str:
    combos = summary["combos"]
    levels = summary["levels"]
    groups = summary["groups"]

    lines: List[str] = [
        "# Mars Yard Robustness Study Summary (Domain Randomization Levels 0–3)",
        "",
        f"Total runs: **{summary['total_runs']}** across **{len(combos)}** combo(s), "
        f"**{len(levels)}** randomization level(s) ({', '.join(f'L{lv}' for lv in levels)}), "
        f"and **{len(groups)}** terrain group(s).",
        f"Success criterion (Phase 6 rule): Nav2 `SUCCEEDED` **and** ground-truth goal error `<= {summary['goal_miss_m']:.1f} m` (else `goal_miss`).",
        "",
        "## 1. True success rate vs randomization level",
        "",
    ]

    hdr = ["Combo"] + [f"L{lv}" for lv in levels]
    lines.append("| " + " | ".join(hdr) + " |")
    lines.append("|" + "|".join(["---"] + ["---:"] * len(levels)) + "|")
    for c in combos:
        cells = [f"`{c}`"] + [_fmt_rate(*summary["success_by_cl"][(c, lv)]) for lv in levels]
        lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 2. Mean ground-truth goal error (m) vs level (all runs, mean ± std)",
        "",
    ]
    lines.append("| " + " | ".join(hdr) + " |")
    lines.append("|" + "|".join(["---"] + ["---:"] * len(levels)) + "|")
    for c in combos:
        cells = [f"`{c}`"] + [_fmt_ms(*summary["gt_err_all"][(c, lv)], dec=3) for lv in levels]
        lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 3. Mean energy (J) vs level (all runs, mean ± std)",
        "",
    ]
    lines.append("| " + " | ".join(hdr) + " |")
    lines.append("|" + "|".join(["---"] + ["---:"] * len(levels)) + "|")
    for c in combos:
        cells = [f"`{c}`"] + [_fmt_ms(*summary["energy_all"][(c, lv)], dec=1) for lv in levels]
        lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 4. Failure counts by combo and level",
        "",
    ]
    fhdr = ["Combo", "Level", "Nav2 ok"] + list(FAILURE_TYPES) + ["Total fail"]
    lines.append("| " + " | ".join(fhdr) + " |")
    lines.append("|" + "|".join(["---", "---:", "---:"] + ["---:"] * (len(FAILURE_TYPES) + 1)) + "|")
    for c in combos:
        for lv in levels:
            fc = summary["failures_by_cl"][(c, lv)]
            nav_ok, tot = summary["nav2_success_by_cl"][(c, lv)]
            cells = (
                [f"`{c}`", f"L{lv}", f"{nav_ok}/{tot}"]
                + [str(fc[ft]) for ft in FAILURE_TYPES]
                + [str(fc["total"])]
            )
            lines.append("| " + " | ".join(cells) + " |")

    lines += [
        "",
        "## 5. True success rate by terrain group",
        "",
    ]
    ghdr = ["Combo", "Level"] + [g.capitalize() for g in groups]
    lines.append("| " + " | ".join(ghdr) + " |")
    lines.append("|" + "|".join(["---", "---:"] + ["---:"] * len(groups)) + "|")
    for c in combos:
        for lv in levels:
            cells = [f"`{c}`", f"L{lv}"] + [
                _fmt_rate(*summary["group_success"][(c, lv, g)]) for g in groups
            ]
            lines.append("| " + " | ".join(cells) + " |")

    lines.append("")
    text = "\n".join(lines)
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text(text, encoding="utf-8")
    return text


def build_robustness_plots(summary: Dict, plots_dir: pathlib.Path) -> List[pathlib.Path]:
    plots_dir.mkdir(parents=True, exist_ok=True)
    combos = summary["combos"]
    levels = summary["levels"]
    x_pos = np.arange(len(levels), dtype=np.float64)
    x_labels = [f"L{lv}" for lv in levels]
    markers = ["o", "s", "^", "D", "v", "P"]
    created: List[pathlib.Path] = []

    # 1. Success rate vs level
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    for i, c in enumerate(combos):
        ys = []
        for lv in levels:
            ok, tot = summary["success_by_cl"][(c, lv)]
            ys.append(100.0 * ok / tot if tot > 0 else np.nan)
        ax.plot(
            x_pos,
            ys,
            marker=markers[i % len(markers)],
            linewidth=2.0,
            markersize=7,
            label=c,
        )
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_ylim(-5.0, 108.0)
    ax.set_xlabel("Domain randomization level")
    ax.set_ylabel("True success rate (%)")
    ax.set_title(f"True Success Rate vs Randomization Level (GT err <= {summary['goal_miss_m']:.1f} m)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", frameon=True)
    fig.tight_layout()
    p_succ = plots_dir / "success_vs_level.png"
    fig.savefig(p_succ, dpi=150)
    plt.close(fig)
    created.append(p_succ)

    # 2. Mean GT goal error vs level (all runs)
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    for i, c in enumerate(combos):
        means = [summary["gt_err_all"][(c, lv)][0] for lv in levels]
        stds = [summary["gt_err_all"][(c, lv)][1] for lv in levels]
        offset = (i - 0.5 * (len(combos) - 1)) * 0.04
        ax.errorbar(
            x_pos + offset,
            means,
            yerr=stds,
            marker=markers[i % len(markers)],
            linewidth=2.0,
            markersize=7,
            capsize=4,
            label=c,
        )
    ax.axhline(
        summary["goal_miss_m"],
        color="red",
        linestyle=":",
        linewidth=1.5,
        label=f"goal_miss threshold ({summary['goal_miss_m']:.1f} m)",
    )
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel("Domain randomization level")
    ax.set_ylabel("Ground-truth goal error (m)")
    ax.set_title("Mean Ground-Truth Goal Error vs Randomization Level (all runs, ±1σ)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", frameon=True)
    fig.tight_layout()
    p_err = plots_dir / "gt_err_vs_level.png"
    fig.savefig(p_err, dpi=150)
    plt.close(fig)
    created.append(p_err)

    # 3. Mean energy vs level (all runs)
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    for i, c in enumerate(combos):
        means = [summary["energy_all"][(c, lv)][0] for lv in levels]
        stds = [summary["energy_all"][(c, lv)][1] for lv in levels]
        offset = (i - 0.5 * (len(combos) - 1)) * 0.04
        ax.errorbar(
            x_pos + offset,
            means,
            yerr=stds,
            marker=markers[i % len(markers)],
            linewidth=2.0,
            markersize=7,
            capsize=4,
            label=c,
        )
    ax.set_xticks(x_pos)
    ax.set_xticklabels(x_labels)
    ax.set_ylim(bottom=0.0)
    ax.set_xlabel("Domain randomization level")
    ax.set_ylabel("Mechanical energy (J)")
    ax.set_title("Mean Energy vs Randomization Level (all runs, ±1σ)")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.legend(loc="best", frameon=True)
    fig.tight_layout()
    p_eng = plots_dir / "energy_vs_level.png"
    fig.savefig(p_eng, dpi=150)
    plt.close(fig)
    created.append(p_eng)

    return created


def analyze_robustness(
    csv_path: pathlib.Path,
    out_md: pathlib.Path,
    plots_dir: pathlib.Path,
) -> Tuple[Dict, List[pathlib.Path]]:
    rows = load_robustness_rows(csv_path)
    summary = compute_robustness_summary(rows)
    write_robustness_markdown(summary, out_md)
    plot_paths = build_robustness_plots(summary, plots_dir)
    return summary, plot_paths


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Analyze Mars Yard robustness CSV (levels 0-3) into tables and plots."
    )
    parser.add_argument(
        "--csv",
        type=pathlib.Path,
        default=pathlib.Path("results/robustness/robustness.csv"),
    )
    parser.add_argument(
        "--out-md",
        type=pathlib.Path,
        default=pathlib.Path("results/robustness/summary.md"),
    )
    parser.add_argument(
        "--plots-dir",
        type=pathlib.Path,
        default=pathlib.Path("results/robustness/plots"),
    )
    args = parser.parse_args(argv)

    summary, plot_paths = analyze_robustness(args.csv, args.out_md, args.plots_dir)
    print(f"Analyzed {summary['total_runs']} rows from {args.csv}")
    print(f"Wrote summary: {args.out_md}")
    for p in plot_paths:
        print(f"Wrote plot   : {p}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
