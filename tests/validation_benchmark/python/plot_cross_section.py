#!/usr/bin/env python3
"""
plot_cross_section.py  —  Native vs G4CAD cross-section comparison figures

Single mode:
    python plot_cross_section.py \
        --native results/output_cross_section_box_native_xy.csv \
        --step   results/output_cross_section_box_step_xy.csv \
        --output figures/

Batch mode (all geometries and planes):
    python plot_cross_section.py --all --data-dir results/ --output figures/
"""
import argparse
import os
import glob
import re
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

DPI = 300

# Classification encoding: 0=Outside, 1=Surface, 2=Inside
CMAP_CLASS = mcolors.ListedColormap(["#f0f0f0", "#e74c3c", "#3498db"])
NORM_CLASS  = mcolors.BoundaryNorm([0, 1, 2, 3], CMAP_CLASS.N)

# Difference map colors
# 0=both same, 1=native_in/step_out (orange), 2=native_out/step_in (purple), 3=surface mismatch (yellow)
CMAP_DIFF = mcolors.ListedColormap(["white", "#e67e22", "#8e44ad", "#f1c40f"])
NORM_DIFF  = mcolors.BoundaryNorm([0, 1, 2, 3, 4], CMAP_DIFF.N)

def load_csv(path, resolution=None):
    df = pd.read_csv(path)
    cls = df["classification"].values.astype(np.int8)
    # Infer resolution from sqrt(N)
    n = len(cls)
    res = resolution or int(round(np.sqrt(n)))
    img = cls.reshape(res, res)
    x = df["x_mm"].values.reshape(res, res)
    y = df["y_mm"].values.reshape(res, res)
    return img, x, y

def compute_diff(img_native, img_step):
    diff = np.zeros_like(img_native, dtype=np.int8)
    both_same = (img_native == img_step)
    diff[both_same] = 0
    # native=Inside(2), step=Outside(0)
    mask1 = (img_native == 2) & (img_step == 0)
    diff[mask1] = 1
    # native=Outside(0), step=Inside(2)
    mask2 = (img_native == 0) & (img_step == 2)
    diff[mask2] = 2
    # any other mismatch (surface involved)
    mask3 = ~both_same & ~mask1 & ~mask2
    diff[mask3] = 3
    return diff

def make_figure(img_native, img_step, x, y, geometry, plane, out_dir):
    diff = compute_diff(img_native, img_step)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    extent = [x.min(), x.max(), y.min(), y.max()]

    # Axis labels by plane
    ax_labels = {"xy": ("x (mm)", "y (mm)"),
                 "xz": ("x (mm)", "z (mm)"),
                 "yz": ("y (mm)", "z (mm)")}
    xl, yl = ax_labels.get(plane, ("u (mm)", "v (mm)"))

    for ax, img, title, cmap, norm in [
        (axes[0], img_native, "Native",      CMAP_CLASS, NORM_CLASS),
        (axes[1], img_step,   "G4CAD STEP",  CMAP_CLASS, NORM_CLASS),
        (axes[2], diff,       "Difference",  CMAP_DIFF,  NORM_DIFF),
    ]:
        im = ax.imshow(img.T, origin="lower", extent=extent,
                       cmap=cmap, norm=norm, aspect="equal", interpolation="nearest")
        ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.set_title(title)
        plt.colorbar(im, ax=ax, fraction=0.046, pad=0.04)

    # Legend for classification
    from matplotlib.patches import Patch
    legend_cls = [Patch(facecolor="#f0f0f0", label="Outside"),
                  Patch(facecolor="#e74c3c", label="Surface"),
                  Patch(facecolor="#3498db", label="Inside")]
    axes[0].legend(handles=legend_cls, loc="lower right", fontsize=7)
    axes[1].legend(handles=legend_cls, loc="lower right", fontsize=7)

    legend_diff = [Patch(facecolor="white",   edgecolor="k", label="Match"),
                   Patch(facecolor="#e67e22", label="Native∈ / STEP∉"),
                   Patch(facecolor="#8e44ad", label="Native∉ / STEP∈"),
                   Patch(facecolor="#f1c40f", label="Surface mismatch")]
    axes[2].legend(handles=legend_diff, loc="lower right", fontsize=7)

    n_mismatch = int((diff > 0).sum())
    n_total    = diff.size
    rate = n_mismatch / n_total * 100
    fig.suptitle(f"{geometry}  —  {plane.upper()} slice at fixed axis = 0"
                 f"   |   mismatch: {n_mismatch}/{n_total} = {rate:.3f}%",
                 fontsize=11)
    fig.tight_layout()

    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"cross_section_{geometry}_{plane}_comparison.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight")
    print(f"Saved: {path}")
    plt.close(fig)

def run_single(native_csv, step_csv, geometry, plane, out_dir):
    img_n, x, y = load_csv(native_csv)
    img_s, _,  _ = load_csv(step_csv)
    make_figure(img_n, img_s, x, y, geometry, plane, out_dir)

def run_batch(data_dir, out_dir):
    # Find all native CSVs and match with step CSVs
    # Pattern: *_cross_section_<geometry>_native_<plane>.csv
    pattern = os.path.join(data_dir, "*_cross_section_*_native_*.csv")
    native_files = glob.glob(pattern)
    if not native_files:
        print(f"No cross-section CSVs found in {data_dir}")
        return
    for nf in sorted(native_files):
        m = re.search(r"cross_section_(.+)_native_(xy|xz|yz)\.csv$", nf)
        if not m:
            continue
        geometry, plane = m.group(1), m.group(2)
        step_f = nf.replace("_native_", "_step_")
        if not os.path.exists(step_f):
            print(f"  Missing STEP file for {geometry}/{plane}: {step_f}")
            continue
        run_single(nf, step_f, geometry, plane, out_dir)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--native",   help="Native CSV (single mode)")
    ap.add_argument("--step",     help="STEP CSV (single mode)")
    ap.add_argument("--geometry", help="Geometry name (single mode)")
    ap.add_argument("--plane",    help="Plane name: xy|xz|yz (single mode)")
    ap.add_argument("--all",      action="store_true", help="Batch mode")
    ap.add_argument("--data-dir", default=".", help="Data directory (batch mode)")
    ap.add_argument("--output",   default="figures", help="Output directory")
    args = ap.parse_args()

    if args.all:
        run_batch(args.data_dir, args.output)
    elif args.native and args.step:
        geom  = args.geometry or "unknown"
        plane = args.plane or "xy"
        run_single(args.native, args.step, geom, plane, args.output)
    else:
        ap.print_help()

if __name__ == "__main__":
    main()
