# G4CAD

`G4CAD` is a Geant4 helper library that imports STEP CAD models through OpenCASCADE Technology (OCCT) and exposes them as `G4VSolid` objects.

## Features

- Load STEP assemblies with `G4StepLoader::Load()`.
- `G4StepSolid` wraps a `TopoDS_Shape` as a thread-safe `G4VSolid` implementing the full Geant4 navigation interface (`Inside`, `DistanceToIn`, `DistanceToOut`, `SurfaceNormal`, `BoundingLimits`).
- Per-constituent-solid `BRepClass3d_SolidClassifier` for `Inside()`; avoids misclassification near assembly joints that a single compound classifier produces.
- Automatic surface-band widening: each solid's surface tolerance band is set to `max(requested, 2 × σ)` where `σ` is the solid's maximum BRep tolerance, ensuring geometric self-consistency.
- Optional navigation query recorder (env-var gated, zero overhead when off) for post-hoc debugging via FreeCAD visualisation.
- Polyhedron visualisation through OCCT face triangulation.

## Requirements

- CMake ≥ 3.16
- Geant4
- OpenCASCADE (OCCT)

## Installation via conda

The recommended way to get all dependencies on Linux is through conda-forge.  
Package names are taken from a known-working environment (Geant4 11.0.3, OCCT 7.7.1):

```bash
conda create -n g4cad -c conda-forge geant4 occt cmake cxx-compiler
conda activate g4cad
```

> **Note:** conda's `occt` package links the visualisation module `TKIVtk` against VTK. G4CAD does not use any OCCT visualisation modules, so VTK is **not** required. The CMakeLists files include a stub that silences the resulting cmake error automatically.

## Build and install

```bash
cmake -S . -B build
cmake --build build -j
cmake --install build --prefix /path/to/install
```

## Use in a downstream project

After installation, add G4CAD to any CMake project with:

```cmake
find_package(G4CAD CONFIG REQUIRED)
target_link_libraries(my_app PRIVATE G4CAD::G4CAD)
```

`G4CAD` exports its full dependency chain, so Geant4 and OCCT do not need to be found separately.

Set `G4CAD_DIR` if the install prefix is not on `CMAKE_PREFIX_PATH`:

```bash
cmake -S . -B build -DG4CAD_DIR=/path/to/install/lib/cmake/G4CAD
```

## Quick start

```cpp
#include "G4StepLoader.hh"
#include "G4StepSolid.hh"

// Load each solid from a STEP assembly
auto shapes = G4StepLoader::Load("model.step");

// Wrap as Geant4 solid — place at world origin; each solid carries its
// assembly-frame position inside its TopLoc_Location (OCCT transform)
for (std::size_t i = 0; i < shapes.size(); ++i) {
    auto* solid = new G4StepSolid("part_" + std::to_string(i), shapes[i]);
    // … build G4LogicalVolume, G4PVPlacement as usual
}
```

A complete runnable example (Geant4 simulation + CMakeLists) is in [`share/example/`](share/example/).

## G4StepSolid reference

### Constructor

```cpp
G4StepSolid(const G4String& name, const TopoDS_Shape& shape,
            G4double tolerance = 1e-7);
```

`tolerance` sets the OCCT intersection search window and the Geant4 surface-band half-width in mm.  
The library enforces `band ≥ 2 × shapeTolerance` automatically; the constructor argument is the lower bound only.

### Environment variables

| Variable | Default | Effect |
|----------|---------|--------|
| `G4CAD_RECORD=file.csv` | off | Record every `Inside`/`DistanceToIn`/`DistanceToOut` call to CSV for debugging |
| `G4CAD_RECORD_EVENT=N` | all | Restrict recording to run N only |
| `G4CAD_TOL_VERBOSE=1` | off | Print per-solid tolerance table at first solid construction |
| `G4CAD_PERSOLID_TOL=1` | off | Use per-solid surface band instead of the global single tolerance |

### Query recorder CSV format

```
seq,event,trackid,parentid,type,px,py,pz,vx,vy,vz,result,dist,inside_at_p,solid
```

`event` = `G4Run::GetRunID()`.  
`type` ∈ `{INSIDE, DTI, DTO}`.  
`inside_at_p` is filled for `DTO` calls (result of `Inside(p)` at the query point) and is used for rigorous stuck-track detection: `DTO < tol AND inside_at_p == kInside` signals a confirmed navigation loop.

## Project layout

```
include/          public headers (G4StepLoader.hh, G4StepSolid.hh)
src/              implementation
cmake/            exported CMake package configuration helpers
share/example/    minimal Geant4 simulation example (see README inside)
tests/            unit tests and full simulation benchmarks
tests/step_files/ STEP geometry files (gammaknife, stcyclotron, RP)
```
