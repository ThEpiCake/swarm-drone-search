#!/usr/bin/env python3
"""Generate final speed-sweep plots from search experiment folders.

Usage:
  scripts/plot_experiment_series.py <series_dir | experiment_dir ...> [--out DIR]

Accepts one or more paths. A series folder (containing series_runs.csv) is
expanded to its experiment folders; experiment folders are used directly.
For each (drone count, v_max) cell the best run is kept (completed status
preferred, then the latest attempt). Also prints ready-to-paste LaTeX rows
for the final results table.
"""

import argparse
import csv
import sys
from dataclasses import dataclass, field
from pathlib import Path

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_OUT = REPO_ROOT / "report/drone_report_latex/images/results"
COVERAGE_GOAL_PCT = 70.0

VMAX_COLORS = {
    "0.70": "#4CAF50",
    "1.30": "#2196F3",
    "1.80": "#FF9800",
    "2.00": "#9C27B0",
}
DEFAULT_COLOR = "#607D8B"
DRONE_COUNT_COLORS = {1: "#2196F3", 2: "#FF9800"}


def vmax_color(vmax):
    return VMAX_COLORS.get(f"{vmax:.2f}", DEFAULT_COLOR)


@dataclass
class Run:
    exp_dir: Path
    drones: int
    vmax: float
    status: str
    attempt: int = 1
    duration: float = float("nan")
    final_cov: float = float("nan")
    ttf_detect: float = float("nan")
    ttf_confirm: float = float("nan")
    hovers: float = float("nan")
    scans: pd.DataFrame = field(default_factory=pd.DataFrame)

    @property
    def completed(self):
        return self.status.startswith("completed")


def load_metrics(exp_dir):
    metrics = {}
    path = exp_dir / "data/metrics.csv"
    if not path.is_file():
        return metrics
    with open(path, newline="") as fh:
        for row in csv.reader(fh):
            if len(row) >= 2 and row[0] != "metric":
                metrics[row[0]] = row[1]
    return metrics


def metric_float(metrics, key):
    try:
        return float(metrics.get(key, ""))
    except ValueError:
        return float("nan")


def load_run(exp_dir, attempt=1, status_hint=""):
    exp_dir = Path(exp_dir)
    metrics = load_metrics(exp_dir)
    if not metrics:
        print(f"WARNING: no metrics.csv in {exp_dir}, skipping", file=sys.stderr)
        return None
    scan_csv = exp_dir / "data/scan_samples.csv"
    scans = pd.read_csv(scan_csv) if scan_csv.is_file() else pd.DataFrame()
    return Run(
        exp_dir=exp_dir,
        drones=int(metric_float(metrics, "drone_count")),
        vmax=metric_float(metrics, "v_max_mps"),
        status=metrics.get("mission_status", status_hint or "unknown"),
        attempt=attempt,
        duration=metric_float(metrics, "last_logged_scan_time"),
        final_cov=metric_float(metrics, "final_logged_coverage"),
        ttf_detect=metric_float(metrics, "time_to_first_detection"),
        ttf_confirm=metric_float(metrics, "time_to_first_confirmed_target"),
        hovers=metric_float(metrics, "emergency_hovers"),
        scans=scans,
    )


def collect_runs(paths, include_incomplete):
    candidates = []
    for path in paths:
        path = Path(path)
        series_csv = path / "series_runs.csv"
        if series_csv.is_file():
            with open(series_csv, newline="") as fh:
                for row in csv.DictReader(fh):
                    exp_dir = Path(row["experiment_dir"])
                    if not exp_dir.is_absolute():
                        exp_dir = REPO_ROOT / exp_dir
                    run = load_run(exp_dir, attempt=int(row.get("attempt", 1) or 1),
                                   status_hint=row.get("mission_status", ""))
                    if run:
                        candidates.append(run)
        elif (path / "data/metrics.csv").is_file():
            run = load_run(path)
            if run:
                candidates.append(run)
        else:
            print(f"WARNING: {path} is neither a series dir nor an experiment dir",
                  file=sys.stderr)

    # keep the best run per (drones, vmax): completed preferred, then latest attempt
    best = {}
    for run in candidates:
        key = (run.drones, f"{run.vmax:.2f}")
        cur = best.get(key)
        if cur is None or (run.completed, run.attempt) > (cur.completed, cur.attempt):
            best[key] = run

    runs = sorted(best.values(), key=lambda r: (r.drones, r.vmax))
    if not include_incomplete:
        kept = [r for r in runs if r.completed]
        for r in runs:
            if not r.completed:
                print(f"NOTE: excluding {r.exp_dir.name} (status={r.status}); "
                      f"use --include-incomplete to keep it", file=sys.stderr)
        runs = kept
    candidates = sorted(candidates, key=lambda r: (r.drones, r.vmax, r.attempt))
    return runs, candidates


def team_coverage(scans):
    """Shared-map coverage over time; max across drones per timestamp."""
    df = scans.dropna(subset=["coverage_pct"])
    return df.groupby("scan_t_s")["coverage_pct"].max()


def plot_duration_vs_vmax(runs, out_dir):
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for drones in sorted({r.drones for r in runs}):
        group = [r for r in runs if r.drones == drones]
        ax.plot([r.vmax for r in group], [r.duration for r in group],
                "o-", color=DRONE_COUNT_COLORS.get(drones, DEFAULT_COLOR),
                linewidth=1.8, markersize=7, label=f"{drones} drone(s)")
        for r in group:
            if not r.completed:
                ax.plot(r.vmax, r.duration, "o", markersize=11, mfc="none",
                        mec="red", mew=1.5, zorder=5)
    ax.set_xlabel(r"$v_{max}$ [m/s]", fontsize=12)
    ax.set_ylabel("Mission duration [s]", fontsize=12)
    ax.set_title("Mission Duration vs Maximum Velocity", fontsize=13)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_dir / "duration_vs_vmax.png", dpi=150)
    plt.close(fig)
    print("Saved duration_vs_vmax.png")


def plot_coverage_over_time(runs, out_dir):
    for drones in sorted({r.drones for r in runs}):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for r in [r for r in runs if r.drones == drones]:
            if r.scans.empty:
                continue
            cov = team_coverage(r.scans)
            style = "-" if r.completed else "--"
            ax.plot(cov.index, cov.values, style, color=vmax_color(r.vmax),
                    linewidth=1.8,
                    label=f"$v_{{max}}$={r.vmax:.2f} m/s" +
                          ("" if r.completed else " (incomplete)"))
        ax.set_xlabel("Mission time [s]", fontsize=12)
        ax.set_ylabel("Reachable coverage [%]", fontsize=12)
        ax.set_title(f"Coverage over Time — {drones} Drone(s)", fontsize=13)
        ax.legend(fontsize=9, loc="lower right")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        name = f"coverage_vs_time_d{drones}.png"
        fig.savefig(out_dir / name, dpi=150)
        plt.close(fig)
        print(f"Saved {name}")


def plot_layer_progression(runs, out_dir):
    for drones in sorted({r.drones for r in runs}):
        fig, ax = plt.subplots(figsize=(8, 4.5))
        for r in [r for r in runs if r.drones == drones]:
            if r.scans.empty:
                continue
            for i, (ns, seg) in enumerate(sorted(r.scans.groupby("drone_ns"))):
                seg = seg.dropna(subset=["layer_index"]).sort_values("scan_t_s")
                style = "-" if i == 0 else "--"
                label = f"$v_{{max}}$={r.vmax:.2f} m/s"
                if drones > 1:
                    label += f" ({ns})"
                ax.step(seg["scan_t_s"], seg["layer_index"], style,
                        where="post", color=vmax_color(r.vmax),
                        linewidth=1.6, label=label)
        ax.set_xlabel("Mission time [s]", fontsize=12)
        ax.set_ylabel("Scan layer index", fontsize=12)
        ax.set_title(f"Scan Layer Progression — {drones} Drone(s)", fontsize=13)
        ax.yaxis.get_major_locator().set_params(integer=True)
        ax.legend(fontsize=8, loc="upper left")
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        name = f"layer_progression_d{drones}.png"
        fig.savefig(out_dir / name, dpi=150)
        plt.close(fig)
        print(f"Saved {name}")


def plot_1v2_comparison(runs, out_dir):
    vmaxes = sorted({f"{r.vmax:.2f}" for r in runs})
    drone_counts = sorted({r.drones for r in runs})
    if len(drone_counts) < 2:
        print("NOTE: only one drone count present, skipping one_vs_two_drones.png",
              file=sys.stderr)
        return
    by_cell = {(r.drones, f"{r.vmax:.2f}"): r for r in runs}
    x = range(len(vmaxes))
    width = 0.35

    fig, (ax_dur, ax_cov) = plt.subplots(1, 2, figsize=(10, 4.5))
    for i, drones in enumerate(drone_counts):
        offs = [xi + (i - (len(drone_counts) - 1) / 2) * width for xi in x]
        durs = [by_cell.get((drones, v)).duration if (drones, v) in by_cell
                else float("nan") for v in vmaxes]
        covs = [by_cell.get((drones, v)).final_cov if (drones, v) in by_cell
                else float("nan") for v in vmaxes]
        color = DRONE_COUNT_COLORS.get(drones, DEFAULT_COLOR)
        ax_dur.bar(offs, durs, width, color=color, label=f"{drones} drone(s)")
        ax_cov.bar(offs, covs, width, color=color, label=f"{drones} drone(s)")

    for ax, ylabel, title in (
        (ax_dur, "Mission duration [s]", "Duration"),
        (ax_cov, "Final coverage [%]", "Final Coverage"),
    ):
        ax.set_xticks(list(x))
        ax.set_xticklabels(vmaxes)
        ax.set_xlabel(r"$v_{max}$ [m/s]", fontsize=11)
        ax.set_ylabel(ylabel, fontsize=11)
        ax.set_title(title, fontsize=12)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3, axis="y")
    fig.suptitle("Single vs Dual Drone Comparison", fontsize=13)
    fig.tight_layout()
    fig.savefig(out_dir / "one_vs_two_drones.png", dpi=150)
    plt.close(fig)
    print("Saved one_vs_two_drones.png")


def plot_xy_trajectories(runs, out_dir):
    """One showcase XY trajectory per drone count (fastest completed run)."""
    for drones in sorted({r.drones for r in runs}):
        group = [r for r in runs if r.drones == drones and r.completed
                 and not r.scans.empty]
        if not group:
            continue
        r = min(group, key=lambda r: r.duration)
        fig, ax = plt.subplots(figsize=(7, 6))
        for i, (ns, seg) in enumerate(sorted(r.scans.groupby("drone_ns"))):
            seg = seg.dropna(subset=["north_m", "east_m"]).sort_values("scan_t_s")
            color = list(VMAX_COLORS.values())[i % len(VMAX_COLORS)]
            ax.plot(seg["east_m"], seg["north_m"], "-", color=color,
                    linewidth=1.2, alpha=0.85, label=ns)
            ax.plot(seg["east_m"].iloc[0], seg["north_m"].iloc[0], "o",
                    color=color, markersize=9, mec="black", zorder=5)
            ax.plot(seg["east_m"].iloc[-1], seg["north_m"].iloc[-1], "s",
                    color=color, markersize=9, mec="black", zorder=5)
        ax.set_xlabel("East [m]", fontsize=12)
        ax.set_ylabel("North [m]", fontsize=12)
        ax.set_title(f"Scan Trajectory (XY) — {drones} Drone(s), "
                     f"$v_{{max}}$={r.vmax:.2f} m/s", fontsize=13)
        ax.legend(fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal")
        fig.tight_layout()
        name = f"trajectory_d{drones}_v{r.vmax:.2f}.png"
        fig.savefig(out_dir / name, dpi=150)
        plt.close(fig)
        print(f"Saved {name}")


def plot_navigation_stress(candidates, out_dir):
    """Emergency hovers per attempt, split by outcome — navigation stress vs speed."""
    cells = sorted({(r.drones, f"{r.vmax:.2f}") for r in candidates})
    if not cells:
        return
    fig, ax = plt.subplots(figsize=(9, 4.5))
    xticks, xlabels = [], []
    for idx, (drones, vmax) in enumerate(cells):
        attempts = [r for r in candidates
                    if r.drones == drones and f"{r.vmax:.2f}" == vmax]
        for j, r in enumerate(attempts):
            hovers = r.hovers if r.hovers == r.hovers else 0.0
            color = "#4CAF50" if r.completed else "#F44336"
            ax.bar(idx + (j - (len(attempts) - 1) / 2) * 0.25, hovers, 0.22,
                   color=color, edgecolor="black", linewidth=0.4)
        xticks.append(idx)
        xlabels.append(f"{drones}d\n{vmax}")
    ax.set_xticks(xticks)
    ax.set_xticklabels(xlabels, fontsize=9)
    ax.set_xlabel("Configuration (drones, $v_{max}$ [m/s])", fontsize=11)
    ax.set_ylabel("Emergency hover events per attempt", fontsize=11)
    ax.set_title("Navigation Stress vs Speed — Emergency Hovers per Attempt", fontsize=13)
    handles = [plt.Rectangle((0, 0), 1, 1, color="#4CAF50"),
               plt.Rectangle((0, 0), 1, 1, color="#F44336")]
    ax.legend(handles, ["completed attempt", "failed attempt"], fontsize=9)
    ax.grid(True, alpha=0.3, axis="y")
    fig.tight_layout()
    fig.savefig(out_dir / "navigation_stress.png", dpi=150)
    plt.close(fig)
    print("Saved navigation_stress.png")


def latex_escape(text):
    return text.replace("_", r"\_")


def print_latex_table(runs):
    print("\n% --- LaTeX rows for tab:final-results-placeholder ---")
    for r in runs:
        dur = f"{r.duration:.0f}" if r.duration == r.duration else "--"
        cov = f"{r.final_cov:.1f}" if r.final_cov == r.final_cov else "--"
        ttf = f"{r.ttf_confirm:.0f}" if r.ttf_confirm == r.ttf_confirm else "--"
        print(f"{r.drones} & {r.vmax:.2f} & {latex_escape(r.status)} & "
              f"{dur} & {cov} & {ttf} \\\\")
    print("% --- end LaTeX rows ---\n")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="+",
                        help="series dir(s) and/or experiment dir(s)")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help=f"output dir for PNGs (default: {DEFAULT_OUT})")
    parser.add_argument("--include-incomplete", action="store_true",
                        help="also plot runs whose mission_status is not completed*")
    args = parser.parse_args()

    runs, candidates = collect_runs(args.paths, args.include_incomplete)
    if not runs:
        print("ERROR: no usable runs found", file=sys.stderr)
        return 1

    print(f"Using {len(runs)} run(s):")
    for r in runs:
        print(f"  {r.drones}d v={r.vmax:.2f} status={r.status} "
              f"dur={r.duration:.0f}s cov={r.final_cov:.1f}% -> {r.exp_dir.name}")

    args.out.mkdir(parents=True, exist_ok=True)
    plot_duration_vs_vmax(runs, args.out)
    plot_coverage_over_time(runs, args.out)
    plot_layer_progression(runs, args.out)
    plot_1v2_comparison(runs, args.out)
    plot_xy_trajectories(runs, args.out)
    plot_navigation_stress(candidates, args.out)
    print_latex_table(runs)
    return 0


if __name__ == "__main__":
    sys.exit(main())
