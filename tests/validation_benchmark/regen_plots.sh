#!/usr/bin/env bash
# regen_plots.sh — re-run all simple geometry benchmarks (dev library) and regenerate figures
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BENCH="${SCRIPT_DIR}/build/g4cad_bench"
STEP_DIR="${SCRIPT_DIR}/step_files"
OUT="${SCRIPT_DIR}/output/regen"
FIGS="${SCRIPT_DIR}/figures"
GEOMS=(box sphere cylinder box_hole touching_boxes)
N=2000

_G4DATA=/home/yinxy/software/miniconda3/envs/IR3/share/Geant4/data
export G4ENSDFSTATEDATA=${_G4DATA}/ENSDFSTATE2.3
export G4LEDATA=${_G4DATA}/EMLOW8.0
export G4LEVELGAMMADATA=${_G4DATA}/PhotonEvaporation5.7
export G4RADIOACTIVEDATA=${_G4DATA}/RadioactiveDecay5.6
export G4PARTICLEXSDATA=${_G4DATA}/PARTICLEXS4.0
export G4NEUTRONHPDATA=${_G4DATA}/NDL4.6

mkdir -p "${OUT}" "${FIGS}"

echo "=== Re-running simple geometry benchmarks (dev library, CT-TRIM-2) ==="

# --- Physics runs ---
for geom in "${GEOMS[@]}"; do
    echo "  [native] ${geom}"
    "${BENCH}" --geometry "${geom}" --mode native \
        --events "${N}" --output "${OUT}/${geom}_native" 2>/dev/null

    echo "  [step  ] ${geom}"
    "${BENCH}" --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events "${N}" --output "${OUT}/${geom}_step" 2>/dev/null
done

# --- Navigation tests ---
for geom in "${GEOMS[@]}"; do
    echo "  [navtest] ${geom}"
    "${BENCH}" --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events 0 --navigation-test \
        --nav-points 50000 --nav-rays 20000 \
        --output "${OUT}/${geom}_navtest" 2>/dev/null
done

# --- Merge CSVs ---
echo "  Merging CSVs..."
python3 - "${OUT}" <<'PYEOF'
import sys, pathlib, pandas as pd

outdir = pathlib.Path(sys.argv[1])

def merge(suffix, out_name):
    files = sorted(outdir.glob(f"*{suffix}"))
    if not files:
        print(f"  WARNING: no files matching *{suffix}")
        return
    dfs = [pd.read_csv(f) for f in files]
    combined = pd.concat(dfs, ignore_index=True)
    combined.to_csv(outdir / out_name, index=False)
    print(f"  {out_name}: {len(combined)} rows from {len(files)} files")

merge("_geometry_summary.csv", "all_geometry_summary.csv")
merge("_navigation_test.csv",  "all_navigation_test.csv")
merge("_event_summary.csv",    "all_event_summary.csv")
merge("_physics_summary.csv",  "all_physics_summary.csv")
PYEOF

# --- Generate figures ---
echo "  Generating figures..."
python3 "${SCRIPT_DIR}/python/plot_validation.py" \
    --data-dir "${OUT}" \
    --output "${FIGS}"

python3 "${SCRIPT_DIR}/python/plot_statistics.py" \
    "${OUT}/all" 2>/dev/null || true

echo "=== Done. Figures written to ${FIGS}/ ==="
ls -lh "${FIGS}"/*.pdf "${FIGS}"/*.png 2>/dev/null || true
