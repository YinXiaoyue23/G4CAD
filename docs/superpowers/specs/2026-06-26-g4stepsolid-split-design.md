# G4StepSolid Source File Split

**Date:** 2026-06-26  
**Goal:** Split the 896-line `G4StepSolid.cc` into an interface layer and a geometry-engine layer for readability and maintainability.

---

## Context

`src/G4StepSolid.cc` mixes four distinct concerns in one file:

| Concern | Lines (approx.) |
|---|---|
| G4VSolid virtual functions (interface) | ~200 |
| Construction, lifecycle, visualisation | ~180 |
| Debug recorder | ~60 |
| Anonymous-namespace helpers (RayHit, HitGroup, GroupHits, BBoxPointDist, rayHitsBox, ShapeNoise) | ~110 |
| AlgoCache constructor / destructor / FaceToSolid | ~80 |
| NearestFace, RayCastToBoundary, GetNormalAtIntersection | ~160 |

Reading `G4StepSolid.cc` to understand the G4VSolid interface requires scrolling past hundreds of lines of ray-casting internals. The split separates these concerns into two focused files.

---

## What Changes

### File layout after refactor

```
include/
  G4StepSolid.hh          ← unchanged
src/
  G4StepSolid.cc          ← interface layer  (~380 lines)
  G4StepSolidImpl.cc      ← geometry engine  (~520 lines)   [new]
  G4StepLoader.cc         ← unchanged
```

`CMakeLists.txt` uses `file(GLOB SOURCES "src/*.cc")` — the new file is picked up automatically, no build-system changes needed.

---

### `src/G4StepSolid.cc` — interface layer

Contains everything a reader needs to understand how G4StepSolid implements the Geant4 solid interface:

- Static member definitions (`sAllCaches`, `sRecordEnabled`, …)
- Anonymous namespace: `kMeshDeflection`, `kProbeMultiplier`, `kMaxSurfRetry`, `polyhedronMutex`, `EInsideName`, coordinate converters (`G4toOCCT`, `OCCTtoG4`)
- Constructor, destructor, copy, assignment
- `CalcBBox`, `BuildFaceWeights`
- All G4VSolid overrides: `Inside`, `DistanceToIn(p,v)`, `DistanceToIn(p)`, `DistanceToOut(p,v)`, `DistanceToOut(p)`, `SurfaceNormal`, `GetPointOnSurface`, `BoundingLimits`, `CalculateExtent`, `StreamInfo`, `DescribeYourselfTo`, `GetPolyhedron`, `CreatePolyhedron`
- Debug recorder: `InitRecorderOnce`, `RecordQuery`

---

### `src/G4StepSolidImpl.cc` — geometry engine

Contains the low-level OCCT geometry computation that backs the interface:

- Heavy OCCT algorithm includes (not in `G4StepSolid.cc`)
- Anonymous namespace: `RayHit`, `HitGroup`, `GroupHits`, `BBoxPointDist`, `rayHitsBox`, `ShapeNoise`, `G4toOCCT`, `OCCTtoG4` (redefined locally — each is a one-liner)
- `AlgoCache::AlgoCache(…)`, `AlgoCache::~AlgoCache()`, `AlgoCache::FaceToSolid`
- `G4StepSolid::GetAlgo()`
- `G4StepSolid::GetNormalAtIntersection(…)`
- `G4StepSolid::NearestFace(…)`
- `G4StepSolid::RayCastToBoundary(…)`

---

### Shared utilities (`G4toOCCT`, `OCCTtoG4`)

These trivial one-line coordinate converters are used in both files. They are defined independently in each file's anonymous namespace rather than extracted to a shared private header. Rationale: the functions are three lines total; a shared header adds more moving parts than it removes.

---

## What Does Not Change

- `include/G4StepSolid.hh` — public API, class layout, `AlgoCache` definition: **unchanged**
- `src/G4StepLoader.cc`, `include/G4StepLoader.hh`: **unchanged**
- `CMakeLists.txt`: **unchanged** (glob picks up the new file)
- All installed files: only `src/G4StepSolidImpl.cc` is added; nothing new is installed

---

## Verification

After the split, the build must succeed and the existing benchmark suite must produce identical results:

1. `cmake --build build -j$(nproc)` — clean compile, zero warnings added
2. `cmake --install build --prefix install` — library installs correctly
3. Safety diagnostic: `ratio_median = 1.000`, `frac_lt_0.1 = 0` for all five geometries
4. Step counts: within prior tolerances (0.99–1.01× native for box/sphere/cylinder/touching_boxes, ~0.92× for box_hole)
