cmake_minimum_required(VERSION 3.16)
project(G4CADValidationBench VERSION 1.0.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Geant4 REQUIRED)
find_package(G4CAD CONFIG REQUIRED
    HINTS $ENV{G4CAD_DIR}
          /home/yinxy/work/IR3/library/G4CAD/install/lib/cmake/G4CAD)

message(STATUS "Geant4 version: ${Geant4_VERSION}")
message(STATUS "G4CAD version:  ${G4CAD_VERSION}")
message(STATUS "G4CAD found at: ${G4CAD_DIR}")

option(ENABLE_ROOT "Enable ROOT output (TTree + TH1)" OFF)
if(ENABLE_ROOT)
    find_package(ROOT REQUIRED COMPONENTS Core Hist Tree)
    message(STATUS "ROOT version: ${ROOT_VERSION}")
endif()

# Detect OpenCASCADE version for EnvironmentRecord
find_package(OpenCASCADE CONFIG QUIET)
if(OpenCASCADE_FOUND AND OpenCASCADE_VERSION)
    set(_OCCT_VER "${OpenCASCADE_VERSION}")
    message(STATUS "OpenCASCADE version: ${_OCCT_VER}")
else()
    set(_OCCT_VER "unknown")
endif()

set(SOURCES
    src/main.cc
    src/GeometryRegistry.cc
    src/NavigationValidator.cc
    src/CrossSectionSampler.cc
    src/PhysicsRunner.cc
    src/BenchmarkRunner.cc
    src/OutputWriter.cc
    src/SafetyDiagnostic.cc
    src/EnvironmentRecord.cc
    src/geant4/DetectorConstruction.cc
    src/geant4/ActionInitialization.cc
    src/geant4/RunAction.cc
    src/geant4/EventAction.cc
    src/geant4/SteppingAction.cc
    src/geant4/PrimaryGeneratorAction.cc
)

add_executable(g4cad_bench ${SOURCES})

target_include_directories(g4cad_bench PRIVATE src)
target_link_libraries(g4cad_bench PRIVATE
    G4CAD::G4CAD
    ${Geant4_LIBRARIES}
)

# Version defines for EnvironmentRecord
target_compile_definitions(g4cad_bench PRIVATE
    G4CAD_VERSION_STR="${G4CAD_VERSION}"
    OCCT_VERSION_STR="${_OCCT_VER}"
    BUILD_TYPE_STR="$<CONFIG>"
)

if(ENABLE_ROOT)
    target_link_libraries(g4cad_bench PRIVATE ROOT::Core ROOT::Hist ROOT::Tree)
    target_compile_definitions(g4cad_bench PRIVATE ENABLE_ROOT)
endif()

install(TARGETS g4cad_bench DESTINATION bin)
