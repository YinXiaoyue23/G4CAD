#pragma once
#include <vector>
#include <string>
#include "G4VSolid.hh"
#include "G4ThreeVector.hh"
#include "OutputWriter.hh"  // for CrossSectionPoint

enum class SlicePlane { XY, XZ, YZ };

struct CrossSectionConfig {
    int    resolution = 500;
    double padding    = 0.05;
    std::vector<SlicePlane> planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
};

class CrossSectionSampler {
public:
    // Sample a single plane for a list of solids (union for multi-solid geometries).
    // Returns CrossSectionPoint vector sorted row-major (x fast, y fast).
    static std::vector<CrossSectionPoint> Sample(
        const std::vector<G4VSolid*>& solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        SlicePlane plane,
        int  resolution,
        double padding);

    // Convenience: run all planes from config, write via OutputWriter
    static void RunAll(
        const std::vector<G4VSolid*>& solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        const CrossSectionConfig& cfg,
        const std::string& geometry_name,
        const std::string& mode,
        OutputWriter& writer);

    static std::string PlaneToString(SlicePlane p);
    static SlicePlane  ParsePlane(const std::string& s);

private:
    // Union Inside for multi-solid (touching_boxes): Inside > Surface > Outside
    static int8_t ClassifyUnion(const std::vector<G4VSolid*>& solids,
                                 const G4ThreeVector& p);
};
