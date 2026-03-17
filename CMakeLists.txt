cmake_minimum_required(VERSION 3.16)
project(G4CAD)

# 1. Find dependencies
# ============================================
find_package(Geant4 REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCASCADE REQUIRED) # Assume OpenCASCADE is available in the user environment (e.g. conda).

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
    ${OpenCASCADE_LIBRARIES}
)

target_include_directories(G4CAD PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${Geant4_INCLUDE_DIRS}
    ${OpenCASCADE_INCLUDE_DIR}
)

# 4. Install rules
# ============================================
include(GNUInstallDirs)

# Install library files (.so / .lib)
install(TARGETS G4CAD
    EXPORT G4CADTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

# Install public headers
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Export the CMake package configuration so users can call find_package(G4CAD).
install(EXPORT G4CADTargets
    FILE G4CADConfig.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/G4CAD
)