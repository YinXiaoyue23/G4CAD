#pragma once
#include <string>
#include <vector>
#include "G4VSolid.hh"
#include "G4ThreeVector.hh"

enum class GeometryType { Box, Sphere, Cylinder, BoxHole, TouchingBoxes };

struct GeometryInfo {
    std::string   name;
    double        analytical_volume_mm3;
    G4ThreeVector bbox_min;
    G4ThreeVector bbox_max;
    int           n_solids;
    double        tolerance_mm;  // used when constructing G4StepSolid
};

class GeometryRegistry {
public:
    static std::vector<G4VSolid*> CreateNative(GeometryType type);
    static std::vector<G4VSolid*> CreateFromStep(const std::string& step_file,
                                                  double tolerance = 1e-7);
    static GeometryInfo GetInfo(GeometryType type);
    static GeometryType Parse(const std::string& name);
    static std::string  ToString(GeometryType type);
};
