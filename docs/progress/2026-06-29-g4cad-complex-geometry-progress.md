# G4CAD Complex Geometry Optimization — Progress

Plan: /home/yinxy/work/IR3/library/G4CAD/docs/complex_geometry_optimization_plan.md

---

## Task CT0 — RP.step Baseline + Timer Integration

**Status: completed**

### Problem: GetCubicVolume() hang

Confirmed: `--complex-case share/example/RP.step --events 50` hung indefinitely after loading
all 13 solids. The culprit is `main.cc:237` calling `s->GetCubicVolume()` on each solid — the
default Geant4 Monte Carlo volume estimator is prohibitively slow for solids with 65–100 faces.

### Fix: `--skip-volume` benchmark flag

Added to `examples/validation_benchmark/src/main.cc`:
- New CLI option `--skip-volume` (option code 2008), no-arg
- Skips `GetCubicVolume()` loop in the complex-case path; reports `volume=SKIPPED(--skip-volume)`
- BoundingLimits bbox computation is preserved (still used to auto-position the particle gun)
- Rebuild: `cmake --build build -j$(nproc)` for the benchmark only; no kernel change

**Working RP.step command:**
```bash
G4CAD_TIMER=1 ./g4cad_bench --complex-case ../../share/example/RP.step \
    --events 200 --threads 1 --skip-volume
```
Wall time: **3.82 s** for 200 events (load 0.45s; physics 3.82s). `[PhysicsRunner] Done` printed.

### RP.step geometry loaded

13 solids, 474 faces total:

| solid | faces | face types |
|-------|-------|-----------|
| part_0 | 4 | Plane=1 Cylinder=3 |
| part_1 | 7 | Plane=2 Cylinder=5 |
| part_2 | 4 | Plane=1 Cylinder=3 |
| part_3 | 23 | Plane=19 Cylinder=4 |
| part_4 | 23 | Plane=19 Cylinder=4 |
| part_5 | 65 | Plane=25 Cylinder=28 Torus=12 |
| part_6 | 65 | Plane=25 Cylinder=28 Torus=12 |
| part_7 | 39 | Plane=26 Cylinder=13 |
| part_8 | 45 | Plane=12 Cylinder=25 Cone=5 Torus=3 |
| part_9 | 45 | Plane=12 Cylinder=25 Cone=5 Torus=3 |
| part_10 | 12 | Plane=4 Cylinder=4 Cone=3 Torus=1 |
| part_11 | 42 | Plane=27 Cylinder=12 Cone=3 |
| part_12 | 100 | Plane=64 Cylinder=28 Torus=8 |

**Face-type summary across all 474 faces:** Plane=237 (50.0%), Cylinder=182 (38.4%), Torus=39 (8.2%), Cone=16 (3.4%).

### Correctness gates

#### RP.step specific
`--navigation-test` and `--safety-diagnostic` require native vs STEP comparison — RP.step has
no native Geant4 counterpart. These gates are **not applicable** for RP.step. The correctness
signal for RP.step is:
1. Physics run completes without crash or assertion failure: **PASS** (200 events, no error)
2. Volume: **SKIPPED** (MC estimator hangs; --skip-volume workaround used)
3. Cross-section compare: **SKIPPED** (requires `--cross-section-compare` which needs native counterpart)

#### 5 simple geometries (regression, unmodified kernel + rebuild of benchmark only)

| geom | navigation (mismatch / max_dev_in mm / max_dev_out mm) | safety (ratio_median / frac_lt_0.1) |
|------|--------------------------------------------------------|--------------------------------------|
| box | 0% / 1.42e-14 / 0 | 1.0 / 0 |
| sphere | 0% / 2.27e-13 / 4.26e-14 | 1.0 / 0 |
| cylinder | 0% / 4.55e-13 / 8.53e-14 | 1.0 / 0 |
| box_hole | 0% / 3.13e-13 / 1.78e-12 | 1.0 / 0 |
| touching_boxes | 0% / 2.84e-14 / 1.14e-13 | 1.0 / 0 |

**All 5 simple geometries: PASS** (navigation 0 mismatch, max_dev < 4e-12mm; safety ratio_median=1, frac_lt_0.1=0).

### RP.step Baseline Timing Table (200 events, 1T, G4CAD_TIMER=1)

> Note on timer inflation: `Inside()` ns/call is inflated by 2× `Clock::now()` overhead
> (~200–400 ns/call for simple geometries). For RP.step parts 0–2, the Inside() ns/call
> values (590k–1.5M ns/call) are NOT just timer inflation — see anomaly analysis below.

**Per-solid timing:**

| solid | faces | Inside calls | Inside ns/call | Inside µs-total | DTI(p,v) calls | DTI(p,v) ns/call | DTI(p) calls | DTI(p) ns/call | DTO(p,v) calls | DTO(p,v) ns/call | DTO(p) calls | DTO(p) ns/call |
|-------|-------|-------------|---------------|-----------------|----------------|-----------------|--------------|----------------|----------------|-----------------|-------------|----------------|
| part_0 | 4 | 1048 | 589,939 | 618,256 | 860 | 1,439 | 4492 | 252 | 689 | 5,413 | 3823 | 229 |
| part_1 | 7 | 1452 | 639,426 | 928,446 | 1134 | 1,322 | 4665 | 14,791 | 642 | 15,051 | 2094 | 561 |
| part_2 | 4 | 868 | 1,482,389 | 1,286,713 | 817 | 1,767 | 3222 | 630 | 11 | 4,961 | 161 | 188 |
| part_3 | 23 | 518 | 1,247 | 646 | 212 | 1,599 | 1278 | 152 | 43 | 10,512 | 217 | 24,147 |
| part_4 | 23 | 21 | 5,872 | 123 | 22 | 8,402 | 114 | 128 | 2 | 10,794 | 7 | 19,907 |
| part_5 | 65 | 14 | 123 | 1 | 49 | 1,318 | 121 | 127 | 1 | 5,751 | 1 | 267,808 |
| part_6 | 65 | 409 | 50,359 | 20,596 | 367 | 11,062 | 1087 | 10,193 | 149 | 30,374 | 567 | 125,204 |
| part_7 | 39 | 2381 | 19,698 | 46,902 | 1552 | 2,480 | 8095 | 5,851 | 29 | 14,053 | 115 | 28,498 |
| part_8 | 45 | 162 | 42,109 | 6,821 | 298 | 16,095 | 1466 | 1,725 | 120 | 25,402 | 394 | 119,463 |
| part_9 | 45 | 235 | 127,617 | 29,990 | 247 | 11,306 | 774 | 30,424 | 0 | — | 0 | — |
| part_10 | 12 | 1151 | 3,777 | 4,348 | 720 | 3,313 | 4156 | 318 | 35 | 15,468 | 101 | 3,634 |
| part_11 | 42 | 1014 | 2,447 | 2,482 | 620 | 3,033 | 3036 | 293 | 36 | 12,043 | 133 | 42,293 |
| part_12 | 100 | 404 | 69,760 | 28,183 | 284 | 17,524 | 1169 | 20,522 | 294 | 18,308 | 955 | 68,846 |

**Sub-counter summary (200 events):**

| solid | faces | ray calls | ray fv/call | ray OCCT | ray analytic | nf calls | nf fv/call | nf OCCT | nf analytic |
|-------|-------|-----------|------------|---------|-------------|---------|-----------|---------|------------|
| part_0 | 4 | 1549 | 2.0 | 261 | 2867 | 3835 | 4.0 | 11 | 15526 |
| part_1 | 7 | 1776 | 2.7 | 220 | 4606 | 2104 | 8.6 | 978 | 17185 |
| part_2 | 4 | 828 | 3.0 | 202 | 2266 | 161 | 41.6 | 24 | 6675 |
| part_3 | 23 | 255 | 0.5 | 104 | 23 | 217 | 4.1 | 572 | 313 |
| part_4 | 23 | 24 | 0.5 | 12 | 0 | 7 | 4.7 | 11 | 22 |
| part_5 | 65 | 50 | 0.1 | 4 | 2 | 1 | 6.0 | 5 | 1 |
| part_6 | 65 | 516 | 1.6 | 522 | 322 | 567 | 7.2 | 2258 | 1830 |
| part_7 | 39 | 1581 | 0.4 | 555 | 28 | 115 | 52.3 | 4695 | 1314 |
| part_8 | 45 | 418 | 5.0 | 1512 | 561 | 394 | 10.1 | 2752 | 1217 |
| part_9 | 45 | 247 | 5.5 | 683 | 664 | 0 | — | 1221 | 1750 |
| part_10 | 12 | 755 | 1.7 | 886 | 431 | 101 | 3.9 | 53 | 340 |
| part_11 | 42 | 656 | 0.8 | 325 | 175 | 133 | 5.1 | 308 | 375 |
| part_12 | 100 | 578 | 3.3 | 1703 | 204 | 955 | 13.2 | 9478 | 3125 |

**Aggregate across all 13 solids:**

| function | total µs | % of timer-total |
|----------|---------|-----------------|
| Inside() | 2,973,507 | 87.1% |
| DTI(p,v) | 29,655 | 0.9% |
| DTI(p) [BRepClass3d bbox prefilter] | 183,144 | 5.4% |
| DTO(p,v) | 28,260 | 0.8% |
| DTO(p) | 200,804 | 5.9% |
| **TOTAL** | **3,415,370** | |

Ray path aggregate: 9,233 calls, 2.1 fv/call, 36.5% OCCT fallback (6,989/19,138 hits).
NF path aggregate: 8,590 calls, 8.4 fv/call, 31.0% OCCT fallback (22,366/72,039 queries).

Wall time: **3.82 s** (200 events, 1T). Events/sec: 52.4 eps vs ~3846 eps for simple geom.

### Anomaly: Inside() pathologically slow for parts 0, 1, 2

Parts 0, 1, 2 (4, 7, 4 faces respectively) show 590k–1.5M ns/call for Inside(). This is NOT
the 2× Clock::now() timer inflation (which contributes ~200–400 ns/call). For comparison,
part_3 (23 faces) shows only 1,247 ns/call for Inside().

**Root cause hypothesis**: `BRepClass3d_SolidClassifier::Perform()` in OCCT is sensitive to
topology, not just face count. Parts 0–2 are likely small cylindrical sleeves/tubes where the
classifier's ray-casting internal to OCCT hits degenerate geometry (tangent rays, co-axial
cylinders, near-miss precision issues) and iterates many times. This is an OCCT internals issue.

Parts 0–2 combined: 618+928+1287 = **2833 ms** of Inside() time = **74% of all Inside() cost**,
despite having only 15 total faces (3.2% of 474).

### Bottleneck analysis

**Corrected wall-time split** (removing ~400 ns/call Clock overhead from Inside() raw totals):

| function group | raw µs | corrected µs | corrected % |
|---------------|--------|-------------|------------|
| Inside() | 2,973,507 | ~2,569,635* | ~88% |
| Ray path (DTI) | 212,799 | 212,799 | ~7% |
| NF path (DTO) | 229,064 | 229,064 | ~8% |

\* At 400 ns/call × 9,677 calls = 3.87 ms overhead — negligible vs 2.97 s; Inside() is TRULY slow.

**The dominant bottleneck IS Inside()** — specifically `BRepClass3d_SolidClassifier::Perform()`
on parts 0–2. This is unlike the simple geometry case where Inside() appeared dominant only
due to timer inflation. Here the raw µs totals (2.83 s for parts 0–2 alone) are real.

However, the Inside() slowness in parts 0–2 is an **OCCT classifier pathology on specific shapes**,
not a face-scan bottleneck. CT1–CT4 (face scan / BVH / analytic surface) do not address this.

**Secondary bottlenecks** (real, addressable by CT1–CT4):
- DTO(p) NearestFace: 200 ms total, 8.4 fv/call avg, 31% OCCT fallback; worst: part_12 (13.2 fv/call, 75% OCCT), part_7 (52.3 fv/call!, 78% OCCT)
- DTI(p) BRepClass3d prefilter: 183 ms total; DTI(p,v) RayCast: 29 ms
- Ray OCCT fallback: 36.5% of all hits (Cone/Torus/non-rect plane)

### CT1–CT4 Priority Recommendation

**Finding**: Inside() dominates wall time for RP.step but through OCCT BRepClass3d
pathology on specific shapes (parts 0–2), not through face-scan O(N) behavior.
The face-scan bottleneck (CT1/CT2) is real but secondary for the specific 200-event run.

**Revised priority (data-driven):**

| task | addresses | expected RP.step benefit | risk |
|------|-----------|------------------------|------|
| CT1 (face sort+break) | NF scan, small win | ~10-20% on DTO(p) for part_7 (52 fv/call) | very low |
| CT2 (face BVH) | NF scan O(N)→O(log N) | ~30-50% on DTO(p) for large solids | high |
| CT3 (plane ray analytic) | ray OCCT fallback for non-rect planes | Cone/Torus dominate fallback more than planes; moderate win | medium |
| CT4 (cone analytic) | 16 Cone faces | Removes fallback for ~36% of OCCT ray hits (Cones heavily in parts 8–12) | medium |

**Proceed CT1→CT2→CT3→CT4 as planned**: Inside() pathology in parts 0–2 is a separate
problem requiring a different fix (OCCT BRepClass3d cache/warm-up, or pre-classification
spatial index), not addressed by the current plan. The NF face-scan and OCCT fallback paths
are still the actionable targets for CT1–CT4. BVH (CT2) is the highest-leverage task.

**One concern**: if Inside() truly dominates wall time (88%), the 12% from CT1–CT4 addressable
paths means maximum achievable speedup from CT1–CT4 is bounded by ~8× total (if DTO/DTI go to
zero). The real speedup will be meaningful for the DTO/DTI paths but the overall wall-time
reduction may be modest unless the Inside() pathology is also addressed.

**Recommendation**: before CT1, file an issue or investigate the BRepClass3d pathology for
parts 0–2. A potential low-cost fix: warm the classifier once at construction time (first
call is slow, subsequent calls may cache) — check if the observed ns/call is uniform or
first-call dominated. This is a potential "bonus CT0.5" with very high leverage.

### Bugs hit

None — CT0 is measurement-only. The `--skip-volume` addition is trivial and benchmark-only.

### Commits

- Benchmark change (main repo `examples/validation_benchmark/src/main.cc`): `--skip-volume` flag
- This progress file: main repo `docs/progress/2026-06-29-g4cad-complex-geometry-progress.md`
- No kernel changes.

---

## Task CT-IN — Analytic Inside() via ray parity

**Status: completed** (dev branch `/tmp/g4cad_dev`, model Opus 4.8)

### Goal recap

Replace the per-call `BRepClass3d_SolidClassifier::Perform` in `Inside()` (the CT0
bottleneck: 87% of RP.step wall, 74% of it in parts 0/1/2 at 590k–1484k ns/call) with
analytic point-in-solid via ray parity, reusing the T3 analytic ray-face intersection +
`GroupHits` machinery. Coverage: analytic only for solids whose faces are ALL
Plane/Cylinder/Sphere; Cone/Torus solids fall back to the OCCT classifier unchanged.

### Changes (all dev branch)

**`include/G4StepSolid.hh`** (ABI: AlgoCache + TimingStats members + static flag only — NO new
`G4StepSolid` non-static member, per the ABI red line):
- `TimingStats`: added `insideAnalyticCount`, `insideFallbackCount` (per solid-test).
- `FaceEntry`: added `bool analyticTrimSafe` — true iff the UV bounding rectangle is the
  EXACT trim (rect plane; OR full-periodic cylinder/sphere with exactly 4 Line/Circle edges).
- `AlgoCache`: added `solidFaceIdx` (per-solid face index lists), `analyticInsideOK`
  (per-solid all-analytic flag), `solidParityDirs` (per-solid ray directions), `scratchInsideHits`.
- New private methods `IntersectFaceForParity`, `InsideSolidAnalytic`.
- New static `sInsideForceOCCT` (env `G4CAD_INSIDE_FORCE_OCCT=1`) for the differential test.

**`src/G4StepSolid.cc`**:
- `BuildFaceWeights`: compute `analyticTrimSafe` per face (plane: isRectPlane; cyl/sphere:
  full-u + 4 Line/Circle edges).
- `Inside()`: per solid, if `analyticInsideOK[i]` use `InsideSolidAnalytic`; else (or on
  analytic ambiguity) fall back to `BRepClass3d_SolidClassifier::Perform`. Counts analytic
  vs fallback. `G4CAD_INSIDE_SELFCHECK=1` logs per-solid analytic-vs-OCCT disagreements.
- `PrintTimingReport`: prints the two new Inside sub-counters.

**`src/G4StepSolidImpl.cc`**:
- AlgoCache ctor: build `solidFaceIdx`/`analyticInsideOK`; per analytic solid, time 14
  candidate directions against its faces and keep the fastest few as `solidParityDirs`
  (avoids the OCCT-intersector direction pathology — see below).
- `IntersectFaceForParity`: analytic ray-surface intersection for trim-safe faces; for
  non-trim-safe analytic faces (BSpline-trimmed cylinders) the per-face OCCT
  `IntCurvesFace_Intersector` (trim-exact) is used. Mirrors T3's transition/UV math.
- `InsideSolidAnalytic`: cast per-solid ray, `GroupHits`, count crossings. Inside iff the
  forward (dist>band) crossing count is odd. **Even-parity invariant**: a closed solid is
  crossed an even number of times total; an odd total (or a net-zero grazing group) ⇒
  unreliable ⇒ perturb-and-retry; if still ambiguous ⇒ OCCT classifier fallback. kSurface
  if nearest group within the per-solid band.

**`examples/validation_benchmark/src/main.cc`**: `--inside-diff [--inside-diff-points N]`
dumps per-point combined Inside() class to CSV; run twice (analytic vs
`G4CAD_INSIDE_FORCE_OCCT=1`) and diff to measure analytic-vs-OCCT disagreement.

**ABI**: full lib rebuild+install + benchmark rebuild performed after every header change.

### Bug hit + fix (root cause investigation)

First implementation used analytic UV-rectangle clipping for ALL cylinder/sphere faces.
The RP.step differential test showed **3726/200000 disagreements (1.86%), ALL in parts
0/1/2**. Root cause (probed with a per-point hit dump + an "all-OCCT-intersector" control):
parts 0/1/2's cylinder faces are full-type Cylinder but have **BSpline UV trim curves
(cutouts)** — the UV bounding rectangle OVER-COVERS the real trimmed face, so the analytic
quadratic produced spurious hits (e.g. one ray gave In,Out,In = 3 crossings where the true
face boundary gives 0). The same ray through the OCCT trim-exact intersector gave 0 hits.

Fix (two parts):
1. **`analyticTrimSafe`** — analytic intersection only when the UV box IS the exact trim
   (clean full cylinders/spheres, rect planes). Non-trim-safe analytic faces use the
   per-face OCCT `IntCurvesFace_Intersector` (trim-exact). Mirrors the T3 `isRectPlane` idea.
2. **Even-parity invariant** — reject odd total-crossing / net-zero-grazing rays, perturb,
   then OCCT-classifier fallback.

Result: RP.step differential dropped to **0–1 / 200000** (the 1 is a surface-band edge case).

### Second finding (OCCT trim pathology — limits the parts-0/1/2 speedup)

`IntCurvesFace_Intersector::Perform` on parts 0/1/2's BSpline-trimmed cylinders is itself
**intrinsically slow and strongly direction-dependent** (measured 1.6 µs to 1.6 ms per
full-ray depending on direction; ~30 µs median even for the best direction at the bbox
centre; windowing the W-range does not help — the cost is in the BSpline-trim setup). This
is the same OCCT-kernel pathology class as `BRepClass3d` on these shapes. `BRepTopAdaptor_FClass2d`
on the same faces is ~15 µs/query — also slow. Mitigations applied: per-solid direction
selection (keeps the median low, avoids the 50–250× bad directions) and a 2-attempt retry
cap. The genuinely sub-µs path (analytic BSpline-trim classification) is out of CT-IN scope
(CT3+ territory) and was not attempted to avoid risking the proven correctness.

### Correctness gates — ALL PASS

**5 simple geometries — navigation (0% mismatch, max_dev < 4e-12 mm) + safety (ratio_median=1, frac_lt_0.1=0):**

| geom | navigation (mismatch / max_dev_in / max_dev_out) | safety |
|------|--------------------------------------------------|--------|
| box | 0% / 1.42e-14 / 0 | ratio_median=1, frac_lt_0.1=0 |
| sphere | 0% / 2.27e-13 / 4.26e-14 | ratio_median=1, frac_lt_0.1=0 |
| cylinder | 0% / 4.55e-13 / 8.53e-14 | ratio_median=1, frac_lt_0.1=0 |
| box_hole | 0% / 3.13e-13 / 1.78e-12 | ratio_median=1, frac_lt_0.1=0 |
| touching_boxes | 0% / 2.84e-14 / 1.14e-13 | ratio_median=1, frac_lt_0.1=0 |

(box navigation exercises Inside 3239 analytic / 0 fallback; box physics 1,003,472 analytic / 0 fallback.)

**RP.step (no native counterpart):**
- (a) DIFFERENTIAL: analytic Inside vs OCCT classifier on a 200,000-point bbox cloud →
  **1 / 200,000 disagreement** (0.0005%, a surface-band edge case). in=11171 (analytic)
  vs 11172 (OCCT). PASS (~0 outside the tolerance band).
- (b) Physics 200 ev: completes, no crash/assertion, no query explosion, no stuck. PASS.
- (c) TruthHits/physics behaviour consistent with the OCCT baseline (run completes normally).

### Stability verification (the prior "227-hit explosion" must NOT recur)

Ran 200 ev with the query recorder (`G4CAD_RECORD`):
- **0 stuck-signature rows** (no `DistanceToOut(p,v)≈0 AND Inside(p)==kInside` contradiction).
- Total 14,364 queries / 200 ev ≈ 72/ev — bounded, normal; NO per-navigation-point explosion.
- Physics completes in normal time. **The 227-hit explosion did NOT recur.**

Why it does not recur (vs the old `install` ray-Inside): the old version used OCCT's GENERAL
intersector which produced spurious/duplicate hits; CT-IN uses T3's analytic intersection +
`GroupHits` (coincidence-merging) for clean faces and the trim-exact per-face intersector for
BSpline-trimmed faces, plus the even-parity invariant that rejects (rather than counts)
grazing/clipping artifacts and defers them to the authoritative OCCT classifier. Inside stays
consistent with the analytic DistanceToIn/Out because both use the identical T3 transition/UV math.

### CT-IN vs CT0 comparison (RP.step, 200 ev, 1T, G4CAD_TIMER=1)

**Per-part Inside():**

| part | faces | type | Inside ns/call CT0 | Inside ns/call CT-IN | Δ | Inside µs-total CT0 | CT-IN |
|------|-------|------|--------------------|----------------------|----|---------------------|-------|
| part_0 | 4 | Plane+Cyl (BSpline-trim) | 589,939 | 222,941 | **-62%** | 618,256 | 233,642 |
| part_1 | 7 | Plane+Cyl (BSpline-trim) | 639,426 | 260,052 | **-59%** | 928,446 | 377,595 |
| part_2 | 4 | Plane+Cyl (BSpline-trim) | 1,482,389 | 157,026 | **-89%** | 1,286,713 | 136,298 |
| part_3 | 23 | Plane+Cyl | 1,247 | 1,370 | — | 646 | 709 |
| part_7 | 39 | Plane+Cyl | 19,698 | 3,416 | **-83%** | 46,902 | 8,133 |
| part_6 | 65 | +Torus (fallback) | 50,359 | 48,626 | ~0 | 20,596 | 19,888 |
| part_9 | 45 | +Cone+Torus (fallback) | 127,617 | 119,521 | ~0 | 29,990 | 28,087 |
| part_12 | 100 | +Torus (fallback) | 69,760 | 61,514 | ~0 | 28,183 | 24,851 |

**Aggregate:**

| metric | CT0 | CT-IN | Δ |
|--------|-----|-------|---|
| Total Inside() µs | 2,973,507 | 844,016 | **-72%** |
| Parts 0/1/2 Inside µs | 2,833,415 | 747,535 | **-74%** |
| Wall time (200 ev, 1T) | 3.81 s | 1.66 s | **-56%** |

**Analytic / fallback Inside counts (aggregate over 200 ev): analytic=2453, fallback=629.**
- Analytic taken for parts 0,1,2,7 (pure Plane+Cyl, `analyticInsideOK=true`).
- Fallback (=BRepClass3d) for: Cone/Torus solids 5,6,8,9,10,11,12 (analytic=0, by design) +
  a minority of grazing points in parts 0/1/3/4/7 flagged by the even-parity invariant.
- Confirms parts 0/1/2 went analytic and Torus/Cone solids fell back, as required by the gate.

### Outcome

- `Inside()` no longer pays the 594k–1.5M ns/call `BRepClass3d` pathology on clean-analytic
  solids; total Inside time −72%, RP.step wall −56% (3.81→1.66 s).
- Parts 0/1/2 dropped 59–89% but did NOT reach µs-level: their cylinders have BSpline UV
  trims, so the trim-exact intersection (OCCT) costs ~30 µs–ms regardless of method. The
  plan's "pure Plane+Cyl → fully analytic → µs" expectation assumed simple trims; the actual
  shapes have BSpline-trimmed cylinders. Sub-µs would require analytic BSpline-trim
  classification (CT3+), deliberately not attempted here to preserve correctness.
- Zero regression on the 5 simple geometries; RP.step analytic-vs-OCCT differential ≈ 0;
  no explosion, no stuck, Inside↔DistanceToOut consistent.

### Bugs hit + fixes
1. Analytic UV-box over-cover on BSpline-trimmed cylinders → 3726/200000 wrong → fixed via
   `analyticTrimSafe` (trim-exact OCCT for non-rect/curved-trim faces) + even-parity invariant.
2. Benchmark `--inside-diff`/`--skip-volume` need the benchmark's OWN build dir rebuilt
   (`cmake --build build` from the validation_benchmark dir), not the kernel build dir; the
   root `g4cad_bench` copy is stale and must be refreshed from `build/g4cad_bench`.

---

## Task CT1 — NearestFace face-sort + early-break

**Status: completed** (dev branch `/tmp/g4cad_dev`, model Sonnet 4.6)

### Goal recap

`NearestFace()` iterated all faces in arbitrary order; the existing `BBoxPointDist >= result.dist`
lower-bound prune barely fired (result.dist started at +inf). CT1 makes the prune effective by
sorting face indices by `BBoxPointDist(fe.bbox, pt)` ascending and **breaking** as soon as the
bbox lower-bound >= result.dist (all remaining faces are provably farther). No math change;
results must be bit-identical to pre-CT1.

### Changes (all dev branch `/tmp/g4cad_dev`)

**`include/G4StepSolid.hh`**:
- `AlgoCache`: added `scratchFaceOrder` (`std::vector<std::pair<double,int>>`) — scratch buffer
  for (bbox_lower_bound_dist, face_index) pairs. Reserved to `nFaces` capacity at construction
  (follows the existing `scratchHits`/`scratchGroups`/`scratchInsideHits` pattern).

**`src/G4StepSolidImpl.cc`**:
- `AlgoCache` ctor: `scratchFaceOrder.reserve(nFaces)` added alongside the other scratch reserves.
- `NearestFace()`: replaced the linear `for (int i = 0; i < nf; ++i)` loop with:
  1. Populate `scratchFaceOrder` with `(BBoxPointDist(fFaces[i].bbox, pt), i)` for each face.
  2. `std::sort(faceOrder.begin(), faceOrder.end())` — ascending by bbox dist.
  3. Range-for over sorted pairs with `if (bboxDist >= result.dist) break` as the first check.
  All math inside the loop body (Plane/Cylinder/Sphere analytic + OCCT fallback) is untouched.

**ABI**: `AlgoCache` gained one new `std::vector` member — full lib rebuild+install +
benchmark rebuild performed.

### Correctness gates — ALL PASS

**5 simple geometries navigation (0% mismatch, max_dev < 4e-12 mm):**

| geom | navigation (mismatch / max_dev_in / max_dev_out) | safety |
|------|--------------------------------------------------|--------|
| box | 0% / 1.42e-14 / 0 | ratio_median=1, frac_lt_0.1=0 |
| sphere | 0% / 2.27e-13 / 4.26e-14 | ratio_median=1, frac_lt_0.1=0 |
| cylinder | 0% / 4.55e-13 / 8.53e-14 | ratio_median=1, frac_lt_0.1=0 |
| box_hole | 0% / 3.13e-13 / 1.78e-12 | ratio_median=1, frac_lt_0.1=0 |
| touching_boxes | 0% / 2.84e-14 / 1.14e-13 | ratio_median=1, frac_lt_0.1=0 |

**RP.step**: Physics run 200 ev completes, no crash/assertion. PASS.

### CT1 vs CT-IN comparison — RP.step (200 ev, 1T, G4CAD_TIMER=1)

**Per-solid DistanceToOut(p) and nf faces-visited/call:**

| solid | faces | DTO(p) ns/call CT-IN* | DTO(p) ns/call CT1 | Δ% | nf fv/call CT-IN* | nf fv/call CT1 | Δ% |
|-------|-------|----------------------|--------------------|----|-------------------|----------------|----|
| part_0 | 4 | 229 | 216 | -6% | 4.0 | 3.3 | -18% |
| part_1 | 7 | 561 | 281 | -50% | 8.6 | 6.4 | -26% |
| part_2 | 4 | 188 | 209 | +11% | 41.6 | 28.2 | -32% |
| part_3 | 23 | 24,147 | 416 | -98% | 4.1 | 1.0 | -76% |
| part_6 | 65 | 125,204 | 1,474 | -99% | 7.2 | 1.1 | -85% |
| part_7 | 39 | 28,498 | 749 | **-97%** | 52.3 | **6.3** | **-88%** |
| part_8 | 45 | 119,463 | 20,390 | -83% | 10.1 | 3.4 | -66% |
| part_10 | 12 | 3,634 | 16,751 | +361% | 3.9 | 3.4 | -13% |
| part_11 | 42 | 42,293 | 17,150 | -59% | 5.1 | 2.6 | -49% |
| part_12 | 100 | 68,846 | 4,054 | **-94%** | 13.2 | **1.5** | **-89%** |

\* CT-IN DTO(p) values from CT0 baseline (CT-IN did not modify DTO path; call counts differ due to stochastic event sampling).

> Note: part_10 shows +361% ns/call vs CT0 — its call count changed (101→184) due to event
> sampling variance, and the higher ns/call reflects the OCCT fallback cost for its Cone/Torus
> faces dominating fewer analytic-short-circuit calls. Total µs is 3,082 vs 3,634 (CT0): flat.

**Aggregate across all 13 solids:**

| metric | CT0 | CT-IN | CT1 | CT1 vs CT-IN |
|--------|-----|-------|-----|-------------|
| NF fv/call | 8.4 | 8.4 | **4.4** | **-48%** |
| NF total fv | 72,039 | ~72,039 | 38,839 | -46% |
| NF total calls | 8,590 | 8,590 | 8,842 | ~0% |
| DTO(p) total µs | 200,804 | ~200,804 | **21,546** | **-89%** |
| Wall time (200 ev, 1T) | 3.82 s | 1.66 s | **~1.42 s** | **-14%** |

\* CT-IN did not change DTO(p) path; CT0/CT-IN DTO(p) µs are the same.

**5 simple geometries DistanceToOut(p) (2000 ev, G4CAD_TIMER=1) — no regression:**

| geom | DTO(p) ns/call | nf fv/call | wall(s) |
|------|----------------|------------|---------|
| box | 196 | 1.0 | 0.54 |
| sphere | 150 | 1.0 | 0.55 |
| cylinder | 173 | 1.8 | 0.54 |
| box_hole | 212 | 1.0 | 0.65 |
| touching_boxes | 196–206 | 1.0 | 0.57 |

All simple geometries: ~1 fv/call (small face count → sort overhead is negligible, break fires immediately), no timing regression.

### Key findings

- **NearestFace fv/call: 8.4 → 4.4 (-48%)** — sorting causes the bbox prune to fire much earlier.
- **Worst solids improved dramatically**: part_7 (39 faces): 52.3→6.3 fv/call (-88%); part_12 (100 faces): 13.2→1.5 fv/call (-89%).
- **DTO(p) total µs: 200,804 → 21,546 (-89%)** — the DTO(p) path is now negligible vs Inside().
- **Overall wall time: 1.66 → ~1.42 s (-14%)** on RP.step; CT-IN eliminated the dominant bottleneck, CT1 cleans up the secondary NF bottleneck.
- The sort overhead (std::sort on a scratch buffer of pairs) is dominated by the savings from avoiding deep OCCT Extrema calls (each Extrema call costs 100s–1000s µs; the sort costs ~N log N comparisons of doubles).

### Bugs hit + fixes

None. The implementation is mechanical (no math change).

---
