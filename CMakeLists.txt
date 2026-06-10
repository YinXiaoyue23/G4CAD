cmake_minimum_required(VERSION 3.16)
project(G4CAD VERSION 0.1.0)

find_package(Geant4 REQUIRED)

# conda's OpenCASCADE builds TKIVtk against VTK; stub the target so cmake
# does not error when VTK is absent (G4CAD uses no OCCT visualization modules).
if(NOT TARGET VTK::CommonCore)
    add_library(VTK::CommonCore INTERFACE IMPORTED)
endif()
find_package(OpenCASCADE REQUIRED)

file(GLOB SOURCES "src/*.cc")
file(GLOB HEADERS "include/*.hh")
add_library(G4CAD SHARED ${SOURCES} ${HEADERS})

# Link only the OCCT modules that G4CAD actually uses, avoiding vis modules
# (TKService, TKV3d, TKIVtk …) that would pull in VTK as a hard dependency.
target_link_libraries(G4CAD PUBLIC
    ${Geant4_LIBRARIES}
    TKernel TKMath TKG3d TKBRep TKGeomBase TKGeomAlgo TKTopAlgo TKMesh
    TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209 TKXSBase TKBO
)

target_include_directories(G4CAD PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(TARGETS G4CAD
    EXPORT G4CADTargets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
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

# Export targets for use from the build tree (without installing)
export(EXPORT G4CADTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/G4CADTargets.cmake"
    NAMESPACE G4CAD::
)
