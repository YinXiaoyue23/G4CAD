#!/usr/bin/env bash
# run_paper_benchmarks.sh — orchestrate all paper benchmark runs for G4CAD
# Usage: ./run_paper_benchmarks.sh [build_dir] [output_dir]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${1:-${SCRIPT_DIR}/build}"
OUT_DIR="${2:-${SCRIPT_DIR}/output/paper}"
STEP_DIR="${SCRIPT_DIR}/step_files"
BENCH="${BUILD_DIR}/g4cad_bench"
RP_STEP="${SCRIPT_DIR}/../../share/example/RP.step"

mkdir -p "${OUT_DIR}"

# Geant4 data directories (needed by FTFP_BERT physics list)
_G4DATA=/home/yinxy/software/miniconda3/envs/IR3/share/Geant4/data
export G4ENSDFSTATEDATA=${_G4DATA}/ENSDFSTATE2.3
export G4LEDATA=${_G4DATA}/EMLOW8.0
export G4LEVELGAMMADATA=${_G4DATA}/PhotonEvaporation5.7
export G4RADIOACTIVEDATA=${_G4DATA}/RadioactiveDecay5.6
export G4PARTICLEXSDATA=${_G4DATA}/PARTICLEXS4.0
export G4PIIDATA=${_G4DATA}/PII1.3
export G4REALSURFACEDATA=${_G4DATA}/RealSurface2.2
export G4SAIDXSDATA=${_G4DATA}/SAIDDATA2.0
export G4ABLADATA=${_G4DATA}/ABLA3.1
export G4INCLDATA=${_G4DATA}/INCL1.0
export G4NEUTRONHPDATA=${_G4DATA}/NDL4.6
unset _G4DATA

if [[ ! -x "${BENCH}" ]]; then
    echo "ERROR: ${BENCH} not found or not executable. Build first." >&2
    exit 1
fi

GEOMS=(box sphere cylinder box_hole touching_boxes)
N_EVENTS=10000

echo "=========================================="
echo " G4CAD paper benchmark suite"
echo " Output: ${OUT_DIR}"
echo "=========================================="

# ---------- Feature 1: 10 000-event physics runs (all geometries, both modes) ----------
echo ""
echo "[1/7] High-statistics physics runs (${N_EVENTS} events)..."
for geom in "${GEOMS[@]}"; do
    echo "  [native] ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode native \
        --events "${N_EVENTS}" \
        --output "${OUT_DIR}/physics_${geom}_native" \
        2>/dev/null

    echo "  [step  ] ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events "${N_EVENTS}" \
        --output "${OUT_DIR}/physics_${geom}_step" \
        2>/dev/null
done

# Merge all physics_summary rows into one file
PHYS_SUMMARY="${OUT_DIR}/all_physics_summary.csv"
header_written=0
for f in "${OUT_DIR}"/physics_*_physics_summary.csv; do
    if [[ -f "$f" ]]; then
        if [[ $header_written -eq 0 ]]; then
            cat "$f" > "${PHYS_SUMMARY}"
            header_written=1
        else
            tail -n +2 "$f" >> "${PHYS_SUMMARY}"
        fi
    fi
done
echo "  -> merged physics summary: ${PHYS_SUMMARY}"

# ---------- Feature 2: Safety distance diagnostic ----------
echo ""
echo "[2/7] Safety diagnostic..."
for geom in "${GEOMS[@]}"; do
    echo "  ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events 0 \
        --safety-diagnostic \
        --safety-points 10000 --safety-path-points 1000 \
        --output "${OUT_DIR}/safety_${geom}" \
        2>/dev/null
done

# Merge safety summaries
SAFE_SUMMARY="${OUT_DIR}/all_safety_summary.csv"
header_written=0
for f in "${OUT_DIR}"/safety_*_safety_summary.csv; do
    if [[ -f "$f" ]]; then
        if [[ $header_written -eq 0 ]]; then
            cat "$f" > "${SAFE_SUMMARY}"
            header_written=1
        else
            tail -n +2 "$f" >> "${SAFE_SUMMARY}"
        fi
    fi
done
echo "  -> merged safety summary: ${SAFE_SUMMARY}"

# ---------- Feature 3: Geantino control runs ----------
echo ""
echo "[3/7] Geantino control runs (1000 events)..."
for geom in "${GEOMS[@]}"; do
    echo "  [native+geantino] ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode native \
        --events 1000 \
        --geantino-test \
        --output "${OUT_DIR}/geantino_native_${geom}" \
        2>/dev/null

    echo "  [step+geantino  ] ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events 1000 \
        --geantino-test \
        --output "${OUT_DIR}/geantino_step_${geom}" \
        2>/dev/null
done

# ---------- Feature 4: Multithreading scaling ----------
echo ""
echo "[4/7] Multithreading scaling (box_hole and touching_boxes, step mode)..."
for geom in box_hole touching_boxes; do
    echo "  [step benchmark] ${geom}"
    "${BENCH}" \
        --geometry "${geom}" --mode step \
        --step-file "${STEP_DIR}/${geom}.step" \
        --events 1000 \
        --benchmark \
        --output "${OUT_DIR}/scaling_${geom}_step" \
        2>/dev/null
done

# Merge scaling summaries (also include the earlier box/native run if present)
SCALE_SUMMARY="${OUT_DIR}/all_scaling_summary.csv"
header_written=0
for f in "${OUT_DIR}"/scaling_*_scaling_summary.csv; do
    if [[ -f "$f" ]]; then
        if [[ $header_written -eq 0 ]]; then
            cat "$f" > "${SCALE_SUMMARY}"
            header_written=1
        else
            tail -n +2 "$f" >> "${SCALE_SUMMARY}"
        fi
    fi
done
echo "  -> merged scaling summary: ${SCALE_SUMMARY}"

# ---------- Feature 5: Cross-section mismatch maps ----------
echo ""
echo "[5/7] Cross-section mismatch maps (box_hole XY, touching_boxes XZ)..."
"${BENCH}" \
    --geometry box_hole --mode step \
    --step-file "${STEP_DIR}/box_hole.step" \
    --events 0 \
    --cross-section-compare \
    --cross-section-plane xy \
    --cross-section-resolution 300 \
    --output "${OUT_DIR}/cs_boxhole" \
    2>/dev/null

"${BENCH}" \
    --geometry touching_boxes --mode step \
    --step-file "${STEP_DIR}/touching_boxes.step" \
    --events 0 \
    --cross-section-compare \
    --cross-section-plane xz \
    --cross-section-resolution 300 \
    --output "${OUT_DIR}/cs_touchingboxes" \
    2>/dev/null

echo "  -> cross-section CSVs written to ${OUT_DIR}/cs_*"

# ---------- Feature 6: Complex STEP case (RP.step) ----------
echo ""
echo "[6/7] Complex STEP case: ${RP_STEP}"
if [[ -f "${RP_STEP}" ]]; then
    "${BENCH}" \
        --complex-case "${RP_STEP}" \
        --events 500 \
        --output "${OUT_DIR}/rp_step" \
        2>/dev/null
    echo "  -> ${OUT_DIR}/rp_step_complex_case.csv"
else
    echo "  WARNING: RP.step not found at ${RP_STEP}, skipping feature 6"
fi

# ---------- Feature 7: Environment record ----------
echo ""
echo "[7/7] Environment record..."
# Written automatically by every g4cad_bench invocation; copy any one as the canonical record
if [[ -f "${OUT_DIR}/physics_box_native_environment.txt" ]]; then
    cp "${OUT_DIR}/physics_box_native_environment.txt" "${OUT_DIR}/environment.txt"
    echo "  -> ${OUT_DIR}/environment.txt"
    cat "${OUT_DIR}/environment.txt"
fi

# ---------- Python analysis ----------
echo ""
echo "Running Python analysis..."
PYTHON="$(command -v python3 || echo python)"

"${PYTHON}" "${SCRIPT_DIR}/python/plot_statistics.py" \
    "${OUT_DIR}/all" 2>/dev/null || \
    echo "  (plot_statistics.py: needs all_physics_summary.csv; merge files first if needed)"

"${PYTHON}" "${SCRIPT_DIR}/python/plot_safety_diagnostic.py" \
    "${OUT_DIR}/safety_box" 2>/dev/null || true

"${PYTHON}" "${SCRIPT_DIR}/python/plot_cross_sections.py" \
    "${OUT_DIR}/cs_boxhole" 2>/dev/null || \
    echo "  (plot_cross_sections.py: needs cross-section CSVs)"

"${PYTHON}" "${SCRIPT_DIR}/python/plot_cross_sections.py" \
    "${OUT_DIR}/cs_touchingboxes" 2>/dev/null || true

echo ""
echo "=========================================="
echo " All done. Results in: ${OUT_DIR}"
echo "=========================================="
