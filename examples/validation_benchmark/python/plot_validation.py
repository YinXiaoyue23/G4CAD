#!/usr/bin/env python3
"""
plot_validation.py  —  Native vs G4CAD validation figures

Usage:
    python plot_validation.py --data-dir results/ --output figures/
"""
import argparse
import os
import glob
import logging
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# PS backend issues transparency warnings via logging, not warnings module
logging.getLogger("matplotlib.backends.backend_ps").setLevel(logging.ERROR)

DPI = 300

def find_csv(data_dir, suffix):
    files = sorted(glob.glob(os.path.join(data_dir, f"*{suffix}")))
    return files[0] if files else None

def savefig(fig, path_pdf):
    fig.savefig(path_pdf, dpi=DPI, bbox_inches="tight")
    path_eps = path_pdf.replace(".pdf", ".eps")
    fig.savefig(path_eps, format="eps", bbox_inches="tight")
    print(f"Saved: {path_pdf}  +  {path_eps}")


def plot_volume_comparison(df_geom, out_dir):
    geoms = df_geom["geometry"].unique()
    native = df_geom[df_geom["mode"] == "native"].drop_duplicates("geometry").set_index("geometry")
    step   = df_geom[df_geom["mode"] == "step"].drop_duplicates("geometry").set_index("geometry")

    x = np.arange(len(geoms))
    w = 0.3
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    ax1.bar(x - w/2, [native.loc[g, "volume_mm3"] if g in native.index else np.nan for g in geoms],
            w, label="Native", color="steelblue")
    ax1.bar(x + w/2, [step.loc[g, "volume_mm3"] if g in step.index else np.nan for g in geoms],
            w, label="G4CAD STEP", color="tomato", alpha=0.85)
    ax1.set_xticks(x); ax1.set_xticklabels(geoms, rotation=15, ha="right")
    ax1.set_ylabel("Volume (mm³)"); ax1.set_title("Volume comparison")
    ax1.legend()

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
    savefig(fig, os.path.join(out_dir, "volume_comparison.pdf"))
    plt.close(fig)


def plot_nav_deviation(df_nav, out_dir):
    if df_nav is None or df_nav.empty:
        return
    geoms = df_nav["geometry"].unique()
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    metrics = [("max_dev_in_mm",  "Max DistToIn deviation (mm)"),
               ("max_dev_out_mm", "Max DistToOut deviation (mm)")]
    for ax, (col, label) in zip(axes, metrics):
        vals = [df_nav[df_nav["geometry"] == g][col].values for g in geoms]
        ax.bar(range(len(geoms)), [v[0] if len(v) else 0 for v in vals], color="coral")
        ax.set_xticks(range(len(geoms))); ax.set_xticklabels(geoms, rotation=15, ha="right")
        ax.set_ylabel(label)
        max_val = max(v[0] if len(v) else 0 for v in vals)
        if max_val > 0:
            ax.set_yscale("log")
        ax.set_title(label)
        ax.axhline(1e-3, ls="--", color="k", lw=0.8, label="1 μm threshold")
        ax.legend()
    fig.suptitle("Navigation distance deviation — native vs STEP")
    fig.tight_layout()
    savefig(fig, os.path.join(out_dir, "nav_deviation.pdf"))
    plt.close(fig)


def plot_energy_deposition(df_event, out_dir):
    if df_event is None or df_event.empty:
        return
    geoms = sorted(df_event["geometry"].unique())
    ncols = min(len(geoms), 3)
    nrows = (len(geoms) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(5*ncols, 4*nrows), squeeze=False)

    for idx, g in enumerate(geoms):
        ax = axes[idx // ncols][idx % ncols]
        nat = df_event[(df_event["geometry"] == g) & (df_event["mode"] == "native")]["edep_MeV"]
        stp = df_event[(df_event["geometry"] == g) & (df_event["mode"] == "step")]["edep_MeV"]
        bins = np.linspace(
            min(nat.min() if len(nat) else 0, stp.min() if len(stp) else 0),
            max(nat.max() if len(nat) else 1, stp.max() if len(stp) else 1),
            51)
        if len(nat):
            ax.hist(nat, bins=bins, density=True, histtype="stepfilled",
                    color="steelblue", alpha=0.45, label="native")
        if len(stp):
            ax.hist(stp, bins=bins, density=True, histtype="step",
                    color="tomato", linewidth=2.0, label="G4CAD STEP")
        ax.set_title(g)
        ax.set_xlabel("Edep (MeV)")
        ax.set_ylabel("Density")
        ax.legend(fontsize=8)

    for idx in range(len(geoms), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Energy deposition distribution — native vs G4CAD STEP")
    fig.tight_layout()
    savefig(fig, os.path.join(out_dir, "energy_deposition.pdf"))
    plt.close(fig)


def plot_step_length(df_event, out_dir):
    if df_event is None or df_event.empty:
        return
    geoms = sorted(df_event["geometry"].unique())
    ncols = min(len(geoms), 3)
    nrows = (len(geoms) + ncols - 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(5*ncols, 4*nrows), squeeze=False)

    for idx, g in enumerate(geoms):
        ax = axes[idx // ncols][idx % ncols]
        nat_df = df_event[(df_event["geometry"] == g) & (df_event["mode"] == "native")]
        stp_df = df_event[(df_event["geometry"] == g) & (df_event["mode"] == "step")]
        nat_sl = (nat_df["track_length_mm"] / nat_df["n_steps"].replace(0, np.nan)).dropna() \
                 if "n_steps" in nat_df.columns else pd.Series([], dtype=float)
        stp_sl = (stp_df["track_length_mm"] / stp_df["n_steps"].replace(0, np.nan)).dropna() \
                 if "n_steps" in stp_df.columns else pd.Series([], dtype=float)

        all_vals = pd.concat([nat_sl, stp_sl])
        if all_vals.empty:
            continue
        bins = np.linspace(all_vals.min(), all_vals.max(), 51)
        if len(nat_sl):
            ax.hist(nat_sl, bins=bins, density=True, histtype="stepfilled",
                    color="steelblue", alpha=0.45, label="native")
        if len(stp_sl):
            ax.hist(stp_sl, bins=bins, density=True, histtype="step",
                    color="tomato", linewidth=2.0, label="G4CAD STEP")
        ax.set_title(g)
        ax.set_xlabel("Mean step length (mm)")
        ax.set_ylabel("Density")
        ax.legend(fontsize=8)

    for idx in range(len(geoms), nrows * ncols):
        axes[idx // ncols][idx % ncols].set_visible(False)
    fig.suptitle("Mean step-length distribution — native vs G4CAD STEP")
    fig.tight_layout()
    savefig(fig, os.path.join(out_dir, "step_length.pdf"))
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
