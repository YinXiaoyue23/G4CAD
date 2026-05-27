cmake_minimum_required(VERSION 3.16)
project(G4CAD VERSION 0.1.0)

# 1. Find dependencies
# ============================================
find_package(Geant4 REQUIRED)
find_package(OpenCASCADE REQUIRED)

# 2. Define the library
# ============================================
file(GLOB SOURCES "src/*.cc")
file(GLOB HEADERS "include/*.hh")

add_library(G4CAD SHARED ${SOURCES} ${HEADERS})

# 3. Configure dependencies and include paths
# ============================================
# PUBLIC means any target linking against G4CAD will also inherit the Geant4 and OCCT link dependencies.
# This avoids requiring downstream users to repeat find_package(OpenCASCADE) in their own projects.

target_link_libraries(G4CAD PUBLIC
    ${Geant4_LIBRARIES}
    # Only the OCCT modules actually used — avoids pulling in vis modules that require VTK
    TKernel TKMath TKG3d TKBRep TKGeomBase TKGeomAlgo TKTopAlgo TKMesh
    TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209 TKXSBase TKBO
)

target_include_directories(G4CAD PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# 4. Install rules
# ============================================
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install library files (.so / .lib)
install(TARGETS G4CAD
    EXPORT G4CADTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install public headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/G4CADConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/G4CADConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/G4CADConfig.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/G4CAD
)

install(EXPORT G4CADTargets
    FILE G4CADTargets.cmake
    NAMESPACE G4CAD::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/G4CAD
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/G4CADConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/G4CADConfigVersion.cmake"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/G4CAD
)
