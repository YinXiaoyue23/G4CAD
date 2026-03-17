# G4CAD

`G4CAD` is a lightweight Geant4 helper library for importing STEP CAD models through OpenCASCADE and exposing them as Geant4 solids.

The project currently provides:

- `G4StepLoader`: reads a STEP file and extracts solid shapes.
- `G4StepSolid`: wraps an OpenCASCADE `TopoDS_Shape` as a `G4VSolid`.

## Features

- Import STEP geometry with OpenCASCADE.
- Convert CAD solids into Geant4 geometry objects.
- Reuse the same library from downstream Geant4 applications through `find_package(G4CAD CONFIG REQUIRED)`.
- Keep Geant4 and OpenCASCADE link dependencies attached to the exported CMake target.

## Requirements

- CMake 3.16 or newer
- Geant4
- OpenCASCADE
- VTK

The library assumes these dependencies are already available in the active environment.

## Build

```bash
cmake -S . -B build
cmake --build build -j
cmake --install build --prefix install
```

## Use In Another Project

After installation, a downstream project can use:

```cmake
find_package(G4CAD CONFIG REQUIRED)

target_link_libraries(my_app PRIVATE G4CAD::G4CAD)
```

`G4CAD` exports its own dependency chain, so downstream projects do not need to repeat the Geant4, VTK, or OpenCASCADE linkage manually as long as the environment provides them.

## Minimal Usage

```cpp
#include "G4StepLoader.hh"
#include "G4StepSolid.hh"

auto shapes = G4StepLoader::Load("model.step");
auto* solid = new G4StepSolid("cad_solid", shapes.at(0));
```

The returned `G4StepSolid` can then be used to build a `G4LogicalVolume` in the normal Geant4 workflow.

## Project Layout

- `include/`: public headers
- `src/`: implementation
- `cmake/`: exported package configuration helpers
- `share/environment/`: exported local reference environments used during development

## Notes

- `G4StepSolid` implements Geant4 geometry navigation interfaces such as `Inside`, `DistanceToIn`, `DistanceToOut`, `SurfaceNormal`, and `BoundingLimits`.
- Visualization support is provided through a polyhedron generated from OpenCASCADE face triangulations.
- A separate test project can be used to validate geometry correctness, navigation behavior, and performance on simple and complex STEP models.

## Reference Environment

The repository includes a snapshot of the local `IR3` conda environment used for development:

- `share/environment/IR3.environment.yml`: a readable environment export without build strings
- `share/environment/IR3.explicit.txt`: an exact package list for stricter reproduction

Example commands:

```bash
conda env create -f share/environment/IR3.environment.yml
conda create --name IR3-repro --file share/environment/IR3.explicit.txt
```

The explicit export preserves the package source URLs from the local machine, so it may need adjustment if you use a different conda mirror.
