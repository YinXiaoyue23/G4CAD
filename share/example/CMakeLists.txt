cmake_minimum_required(VERSION 3.16)
project(g4cad_example)

# ── Geant4 ──────────────────────────────────────────────────────────────────
find_package(Geant4 REQUIRED)
include(${Geant4_USE_FILE})

# ── OpenCASCADE workaround ───────────────────────────────────────────────────
# conda's occt package builds TKIVtk against VTK; stub the target so cmake
# does not error when VTK is absent (G4CAD does not use OCCT vis modules).
if(NOT TARGET VTK::CommonCore)
    add_library(VTK::CommonCore INTERFACE IMPORTED)
endif()

# ── G4CAD ────────────────────────────────────────────────────────────────────
# Set G4CAD_DIR to the directory containing G4CADConfig.cmake, e.g.:
#   cmake -S . -B build -DG4CAD_DIR=/path/to/g4cad/install/lib/cmake/G4CAD
find_package(G4CAD CONFIG REQUIRED)

add_executable(example
    main.cc
    DetectorConstruction.cc
)
target_link_libraries(example PRIVATE G4CAD::G4CAD)
