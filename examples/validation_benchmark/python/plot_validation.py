#!/usr/bin/env python3
"""
plot_validation.py  —  Native vs G4CAD validation figures

Usage:
    python plot_validation.py --data-dir results/ --output figures/
"""
import argparse
import os
import glob
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DPI = 300

def find_csv(data_dir, suffix):
    files = glob.glob(os.path.join(data_dir, f"*{suffix}"))
    return files[0] if files else None

def plot_volume_comparison(df_geom, out_dir):
    geoms = df_geom["geometry"].unique()
    native = df_geom[df_geom["mode"] == "native"].set_index("geometry")
    step   = df_geom[df_geom["mode"] == "step"].set_index("geometry")

    x = np.arange(len(geoms))
    w = 0.3
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Volume bar chart
    v_nat = [native.loc[g, "volume_mm3"]  if g in native.index else np.nan for g in geoms]
    v_stp = [step.loc[g,   "volume_mm3"]  if g in step.index   else np.nan for g in geoms]
    ax1.bar(x - w/2, v_nat, w, label="Native", color="steelblue")
    ax1.bar(x + w/2, v_stp, w, label="G4CAD STEP", color="tomato", alpha=0.8)
    ax1.set_xticks(x); ax1.set_xticklabels(geoms, rotation=15, ha="right")
    ax1.set_ylabel("Volume (mm³)"); ax1.set_title("Volume comparison")
    ax1.legend()

    # Relative error
    rel_errs = []
    for g in geoms:
        if g in native.index and g in step.index:
            vn = native.loc[g, "volume_mm3"]
            vs = step.loc[g, "volume_mm3"]
            rel_errs.append(abs(vn - vs) / vn * 100 if vn > 0 else np.nan)
        else:
            rel_errs.append(np.nan)
    ax2.bar(x, rel_errs, 0.5, color="mediumpurple")
    ax2.set_xticks(x); ax2.set_xticklabels(geoms, rotation=15, ha="right")
    ax2.set_ylabel("Relative volume error (%)"); ax2.set_title("Native vs STEP volume error")

    fig.tight_layout()
    path = os.path.join(out_dir, "volume_comparison.pdf")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close(fig)

def plot_nav_deviation(df_nav, out_dir):
    if df_nav is None or df_nav.empty:
        return
    geoms = df_nav["geometry"].unique()
    # CDF of max_dev_in across geometries (summary, not per-ray)
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    metrics = [("max_dev_in_mm", "Max DistToIn deviation (mm)"),
               ("max_dev_out_mm", "Max DistToOut deviation (mm)")]
    for ax, (col, label) in zip(axes, metrics):
        vals = [df_nav[df_nav["geometry"] == g][col].values for g in geoms]
        ax.bar(range(len(geoms)), [v[0] if len(v) else 0 for v in vals], color="coral")
        ax.set_xticks(range(len(geoms))); ax.set_xticklabels(geoms, rotation=15, ha="right")
        ax.set_ylabel(label); ax.set_yscale("log"); ax.set_title(label)
        ax.axhline(1e-3, ls="--", color="k", lw=0.8, label="1 μm threshold")
        ax.legend()
    fig.suptitle("Navigation distance deviation — native vs STEP")
    fig.tight_layout()
    path = os.path.join(out_dir, "nav_deviation.pdf")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close(fig)

def plot_energy_deposition(df_event, out_dir):
    if df_event is None or df_event.empty:
        return
    geoms = df_event["geometry"].unique()
    nmodes = df_event["mode"].unique()
    ncols = min(len(geoms), 3)
    nrows = (len(geoms) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(5*ncols, 4*nrows), squeeze=False)
    colors = {"native": "steelblue", "step": "tomato"}
    for idx, g in enumerate(geoms):
        ax = axes[idx // ncols][idx % ncols]
        for mode in nmodes:
            sub = df_event[(df_event["geometry"] == g) & (df_event["mode"] == mode)]
            if sub.empty: continue
            ax.hist(sub["edep_MeV"], bins=50, histtype="step", density=True,
                    label=mode, color=colors.get(mode, None), linewidth=1.5)
        ax.set_title(g); ax.set_xlabel("Edep (MeV)"); ax.set_ylabel("Density")
        ax.legend(fontsize=8)
    # Hide unused subplots
    for idx in range(len(geoms), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Energy deposition distribution — native vs G4CAD STEP")
    fig.tight_layout()
    path = os.path.join(out_dir, "energy_deposition.pdf")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close(fig)

def plot_step_length(df_event, out_dir):
    if df_event is None or df_event.empty:
        return
    geoms = df_event["geometry"].unique()
    ncols = min(len(geoms), 3)
    nrows = (len(geoms) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(5*ncols, 4*nrows), squeeze=False)
    colors = {"native": "steelblue", "step": "tomato"}
    for idx, g in enumerate(geoms):
        ax = axes[idx // ncols][idx % ncols]
        for mode in df_event["mode"].unique():
            sub = df_event[(df_event["geometry"] == g) & (df_event["mode"] == mode)]
            if sub.empty or "n_steps" not in sub.columns: continue
            # step length ≈ track_length / n_steps per event
            sl = sub["track_length_mm"] / sub["n_steps"].replace(0, np.nan)
            ax.hist(sl.dropna(), bins=50, histtype="step", density=True,
                    label=mode, color=colors.get(mode, None), linewidth=1.5)
        ax.set_title(g); ax.set_xlabel("Mean step length (mm)"); ax.set_ylabel("Density")
        ax.legend(fontsize=8)
    for idx in range(len(geoms), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Mean step-length distribution — native vs G4CAD STEP")
    fig.tight_layout()
    path = os.path.join(out_dir, "step_length.pdf")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close(fig)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data-dir", default=".", help="Directory containing CSV files")
    ap.add_argument("--output",   default="figures", help="Output directory for figures")
    args = ap.parse_args()

    os.makedirs(args.output, exist_ok=True)

    f_geom  = find_csv(args.data_dir, "_geometry_summary.csv")
    f_nav   = find_csv(args.data_dir, "_navigation_test.csv")
    f_event = find_csv(args.data_dir, "_event_summary.csv")

    df_geom  = pd.read_csv(f_geom)  if f_geom  else pd.DataFrame()
    df_nav   = pd.read_csv(f_nav)   if f_nav   else pd.DataFrame()
    df_event = pd.read_csv(f_event) if f_event else pd.DataFrame()

    if not df_geom.empty:
        plot_volume_comparison(df_geom, args.output)
    if not df_nav.empty:
        plot_nav_deviation(df_nav, args.output)
    if not df_event.empty:
        plot_energy_deposition(df_event, args.output)
        plot_step_length(df_event, args.output)

if __name__ == "__main__":
    main()
