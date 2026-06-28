"""
Plot physics summary statistics (mean ± SE) for all geometries and modes.
Reads: {prefix}_physics_summary.csv
"""
import sys
import pathlib
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

def load(prefix):
    path = pathlib.Path(f"{prefix}_physics_summary.csv")
    if not path.exists():
        sys.exit(f"File not found: {path}")
    return pd.read_csv(path)

def grouped_bar(ax, df, col_mean, col_se, label, color):
    geoms = df["geometry"].unique()
    x = np.arange(len(geoms))
    width = 0.35
    native = df[df["mode"] == "native"].set_index("geometry")
    step   = df[df["mode"] == "step"].set_index("geometry")

    n_vals = native.reindex(geoms)[col_mean].values
    n_err  = native.reindex(geoms)[col_se].values
    s_vals = step.reindex(geoms)[col_mean].values
    s_err  = step.reindex(geoms)[col_se].values

    ax.bar(x - width/2, n_vals, width, yerr=n_err, label="native",
           color="steelblue", capsize=4)
    ax.bar(x + width/2, s_vals, width, yerr=s_err, label="step",
           color="coral",     capsize=4)
    ax.set_xticks(x)
    ax.set_xticklabels(geoms, rotation=15, ha="right")
    ax.set_ylabel(label)
    ax.legend(fontsize=8)

def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else "output/paper"
    outdir = pathlib.Path(prefix).parent
    outdir.mkdir(parents=True, exist_ok=True)

    df = load(prefix)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("Physics observables: mean ± SE (1σ/√N)", fontsize=12)

    grouped_bar(axes[0,0], df, "edep_mean_MeV",     "edep_se_MeV",     "Energy deposit (MeV)", "tab:blue")
    grouped_bar(axes[0,1], df, "track_len_mean_mm", "track_len_se_mm", "Track length (mm)",    "tab:orange")
    grouped_bar(axes[1,0], df, "steps_mean",         "steps_se",        "Steps per event",      "tab:green")
    grouped_bar(axes[1,1], df, "bcx_mean",           "bcx_se",          "Boundary crossings",   "tab:red")

    plt.tight_layout()
    out = outdir / "physics_statistics.pdf"
    fig.savefig(out, bbox_inches="tight")
    print(f"Saved {out}")

    # Print compact table
    cols = ["geometry", "mode", "n_events",
            "edep_mean_MeV", "edep_std_MeV", "edep_se_MeV",
            "steps_mean",    "steps_std",    "steps_se",
            "bcx_mean",      "bcx_std",      "bcx_se"]
    print(df[cols].to_string(index=False, float_format=lambda x: f"{x:.4g}"))

if __name__ == "__main__":
    main()
