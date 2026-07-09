"""
Plot safety distance ratio (step / native) diagnostics.
Reads:  {prefix}_safety_diagnostic_{geom}_{mode}.csv
        {prefix}_safety_summary.csv
"""
import sys
import pathlib
import glob
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "output/paper"
    outdir = pathlib.Path(prefix).parent
    outdir.mkdir(parents=True, exist_ok=True)

    # ---- per-geometry ratio distributions ----
    diag_files = sorted(glob.glob(f"{prefix}_safety_diagnostic_*.csv"))
    if not diag_files:
        print(f"No safety_diagnostic files found for prefix '{prefix}'")
        return

    ncols = 2
    nrows = (len(diag_files) + 1) // ncols
    fig, axes = plt.subplots(nrows, ncols, figsize=(12, 4 * nrows))
    axes = np.array(axes).flatten()

    for i, fpath in enumerate(diag_files):
        ax = axes[i]
        df = pd.read_csv(fpath)
        name = pathlib.Path(fpath).stem.replace(
            pathlib.Path(prefix).name + "_safety_diagnostic_", "")

        ratios = df["ratio"].dropna()
        ratios_clipped = ratios.clip(0, 5)

        random_r = df[df["source"] == 0]["ratio"].clip(0, 5)
        path_r   = df[df["source"] == 1]["ratio"].clip(0, 5)

        bins = np.linspace(0, 5, 100)
        ax.hist(random_r, bins=bins, alpha=0.6, label="random",  color="steelblue",  density=True)
        ax.hist(path_r,   bins=bins, alpha=0.6, label="on path", color="darkorange", density=True)
        ax.axvline(1.0, color="black", lw=1.2, ls="--", label="ratio=1")

        median = ratios.median()
        p05    = ratios.quantile(0.05)
        ax.set_title(f"{name}  median={median:.3f}  p5={p05:.3f}", fontsize=9)
        ax.set_xlabel("safety ratio (step/native)")
        ax.set_ylabel("density")
        ax.legend(fontsize=7)

    for j in range(len(diag_files), len(axes)):
        axes[j].set_visible(False)

    fig.suptitle("Safety distance ratio: G4CAD / native", fontsize=12)
    plt.tight_layout()
    prefix_base = pathlib.Path(prefix).name
    out = outdir / f"{prefix_base}_safety_diagnostic.pdf"
    fig.savefig(out, bbox_inches="tight")
    print(f"Saved {out}")

    # ---- summary table ----
    summary_path = pathlib.Path(f"{prefix}_safety_summary.csv")
    if summary_path.exists():
        df_sum = pd.read_csv(summary_path)
        cols = ["geometry", "mode", "n_points_used",
                "ratio_median", "ratio_p05", "ratio_p95",
                "frac_lt_0p1", "frac_lt_0p01",
                "native_safety_mean_mm", "step_safety_mean_mm"]
        avail = [c for c in cols if c in df_sum.columns]
        print("\nSafety summary:")
        print(df_sum[avail].to_string(index=False, float_format=lambda x: f"{x:.4g}"))

if __name__ == "__main__":
    main()
