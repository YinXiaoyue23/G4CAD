#!/usr/bin/env python3
"""
plot_scaling.py  —  Runtime and parallel efficiency figures

Usage:
    python plot_scaling.py scaling_summary.csv --output figures/
    python plot_scaling.py scaling_summary.csv --geometry box_hole --output figures/
"""
import argparse
import os
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DPI = 300

def plot_scaling(df, out_dir, geometry_filter=None):
    if geometry_filter:
        df = df[df["geometry"] == geometry_filter]
    if df.empty:
        print("No data to plot.")
        return

    geoms = df["geometry"].unique()
    modes = df["mode"].unique()
    colors = plt.cm.tab10(np.linspace(0, 1, len(geoms) * len(modes)))

    fig1, ax1 = plt.subplots(figsize=(7, 5))   # wall time
    fig2, ax2 = plt.subplots(figsize=(7, 5))   # speedup
    fig3, ax3 = plt.subplots(figsize=(7, 5))   # efficiency

    ci = 0
    all_threads = sorted(df["n_threads"].unique())

    for g in geoms:
        for m in modes:
            sub = df[(df["geometry"] == g) & (df["mode"] == m)].sort_values("n_threads")
            if sub.empty:
                continue
            label = f"{g}/{m}"
            c = colors[ci]; ci += 1
            ax1.plot(sub["n_threads"], sub["wall_time_s"],    "o-", label=label, color=c)
            ax2.plot(sub["n_threads"], sub["speedup"],         "o-", label=label, color=c)
            ax3.plot(sub["n_threads"], sub["efficiency_pct"],  "o-", label=label, color=c)

    # Ideal linear reference
    t_ref = np.array(all_threads, dtype=float)
    ax2.plot(t_ref, t_ref, "k--", lw=0.8, label="Ideal linear")
    ax3.axhline(100, ls="--", color="k", lw=0.8, label="100% efficiency")

    ax1.set_xlabel("Threads"); ax1.set_ylabel("Wall time (s)")
    ax1.set_title("Wall time vs thread count")
    ax1.set_xscale("log", base=2); ax1.set_yscale("log")
    ax1.legend(fontsize=8)

    ax2.set_xlabel("Threads"); ax2.set_ylabel("Speedup")
    ax2.set_title("Speedup vs thread count")
    ax2.set_xscale("log", base=2)
    ax2.legend(fontsize=8)

    ax3.set_xlabel("Threads"); ax3.set_ylabel("Parallel efficiency (%)")
    ax3.set_title("Parallel efficiency vs thread count")
    ax3.set_xscale("log", base=2)
    ax3.legend(fontsize=8)

    os.makedirs(out_dir, exist_ok=True)
    suffix = f"_{geometry_filter}" if geometry_filter else ""
    for fig, name in [(fig1, "scaling_walltime"), (fig2, "scaling_speedup"),
                      (fig3, "scaling_efficiency")]:
        path = os.path.join(out_dir, f"{name}{suffix}.pdf")
        fig.tight_layout()
        fig.savefig(path, dpi=DPI, bbox_inches="tight")
        print(f"Saved: {path}")
        plt.close(fig)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", help="Path to scaling_summary.csv")
    ap.add_argument("--geometry", default=None, help="Filter to specific geometry")
    ap.add_argument("--output", default="figures", help="Output directory")
    args = ap.parse_args()

    df = pd.read_csv(args.csv)
    plot_scaling(df, args.output, args.geometry)

if __name__ == "__main__":
    main()
