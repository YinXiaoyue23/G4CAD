"""
Plot cross-section classification maps (native, step, mismatch).
Reads:  {prefix}_cross_section_{geom}_{kind}_{plane}.csv
Produces: {outdir}/cross_section_{geom}_{plane}.pdf  (3-panel figure)
"""
import sys
import pathlib
import glob
import re
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

INSIDE_COLOR  = "#2166ac"   # blue  — inside solid
OUTSIDE_COLOR = "#f4a582"   # salmon — outside
MISMATCH_COLOR = "#d73027"  # red   — mismatch

INSIDE_CMAP   = ListedColormap([OUTSIDE_COLOR, INSIDE_COLOR])
MISMATCH_CMAP = ListedColormap(["#ffffff", MISMATCH_COLOR])


def load_grid(path):
    """Return (x_vals, y_vals, grid_2d) from a cross-section CSV."""
    df = pd.read_csv(path)
    # Find the two columns with varying values (ignore constant coordinate)
    # Columns typically: x_mm, y_mm [, z_mm], classification
    coord_cols = [c for c in df.columns if c != "classification"]
    varying = [c for c in coord_cols if df[c].nunique() > 1]
    if len(varying) < 2:
        varying = coord_cols[:2]  # fallback
    ax0, ax1 = varying[0], varying[1]
    cl = "classification"
    x_vals = np.sort(df[ax0].unique())
    y_vals = np.sort(df[ax1].unique())
    grid = df.pivot_table(index=ax1, columns=ax0, values=cl, aggfunc="first").values
    return x_vals, y_vals, grid, ax0, ax1


def plot_triplet(native_path, step_path, mismatch_path, out_pdf):
    x, y, nat_grid, ax0, ax1   = load_grid(native_path)
    _, _, step_grid, _, _       = load_grid(step_path)
    _, _, mm_grid, _, _         = load_grid(mismatch_path)

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    extent = [x.min(), x.max(), y.min(), y.max()]

    axes[0].imshow(nat_grid,  origin="lower", extent=extent, aspect="auto",
                   cmap=INSIDE_CMAP, vmin=0, vmax=1, interpolation="nearest")
    axes[0].set_title("Native", fontsize=11)
    axes[0].set_xlabel(f"{ax0} (mm)")
    axes[0].set_ylabel(f"{ax1} (mm)")

    axes[1].imshow(step_grid, origin="lower", extent=extent, aspect="auto",
                   cmap=INSIDE_CMAP, vmin=0, vmax=1, interpolation="nearest")
    axes[1].set_title("G4CAD (STEP)", fontsize=11)
    axes[1].set_xlabel(f"{ax0} (mm)")

    im = axes[2].imshow(mm_grid, origin="lower", extent=extent, aspect="auto",
                        cmap=MISMATCH_CMAP, vmin=0, vmax=1, interpolation="nearest")
    n_mm = int(mm_grid.sum())
    n_tot = mm_grid.size
    axes[2].set_title(f"Mismatch  ({n_mm}/{n_tot} = {100*n_mm/n_tot:.2f}%)", fontsize=11)
    axes[2].set_xlabel(f"{ax0} (mm)")

    geom_name = pathlib.Path(native_path).stem
    fig.suptitle(f"Cross-section: {geom_name}", fontsize=12)
    plt.tight_layout()
    fig.savefig(out_pdf, bbox_inches="tight", dpi=150)
    out_eps = out_pdf.replace(".pdf", ".eps")
    fig.savefig(out_eps, format="eps", bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {out_pdf}  +  {out_eps}")


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "output/paper/cs_boxhole"
    outdir = pathlib.Path(prefix).parent

    # Discover all native files matching pattern *_cross_section_*_native_*.csv
    pat = f"{prefix}_cross_section_*_native_*.csv"
    native_files = sorted(glob.glob(pat))
    if not native_files:
        print(f"No cross-section files found for prefix '{prefix}'")
        return

    for nat_path in native_files:
        # Extract geometry and plane from filename
        stem = pathlib.Path(nat_path).stem
        # stem looks like: <prefix_base>_cross_section_<geom>_native_<plane>
        prefix_base = pathlib.Path(prefix).name
        inner = stem.replace(prefix_base + "_cross_section_", "")
        # inner = <geom>_native_<plane>
        m = re.match(r"^(.+)_native_(\w+)$", inner)
        if not m:
            print(f"  skipping (unexpected name): {stem}")
            continue
        geom, plane = m.group(1), m.group(2)

        step_path     = nat_path.replace("_native_", "_step_")
        mismatch_path = nat_path.replace("_native_", "_mismatch_")
        if not pathlib.Path(step_path).exists() or not pathlib.Path(mismatch_path).exists():
            print(f"  missing step or mismatch CSV for {geom}/{plane}, skipping")
            continue

        out_pdf = outdir / f"cross_section_{geom}_{plane}.pdf"
        plot_triplet(nat_path, step_path, mismatch_path, out_pdf)

        # Also print mismatch fraction
        _, _, mm_grid, _, _ = load_grid(mismatch_path)
        n_mm = int(mm_grid.sum())
        print(f"  {geom} {plane}: {n_mm}/{mm_grid.size} mismatched pixels "
              f"({100*n_mm/mm_grid.size:.2f}%)")


if __name__ == "__main__":
    main()
