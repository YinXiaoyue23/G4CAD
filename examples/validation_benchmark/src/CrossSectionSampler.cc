#include "CrossSectionSampler.hh"
#include <stdexcept>
#include <iostream>

int8_t CrossSectionSampler::ClassifyUnion(const std::vector<G4VSolid*>& solids,
                                           const G4ThreeVector& p) {
    int8_t best = 0;  // kOutside
    for (auto* s : solids) {
        EInside r = s->Inside(p);
        if (r == kInside)  return 2;
        if (r == kSurface && best < 1) best = 1;
    }
    return best;
}

std::vector<CrossSectionPoint> CrossSectionSampler::Sample(
    const std::vector<G4VSolid*>& solids,
    const G4ThreeVector& bbox_min,
    const G4ThreeVector& bbox_max,
    SlicePlane plane,
    int  resolution,
    double padding)
{
    // Expand bounding box
    G4ThreeVector span = bbox_max - bbox_min;
    G4ThreeVector lo   = bbox_min - padding * span;
    G4ThreeVector hi   = bbox_max + padding * span;

    std::vector<CrossSectionPoint> pts;
    pts.reserve(static_cast<std::size_t>(resolution) * resolution);

    double u0, u1, v0, v1;
    switch (plane) {
        case SlicePlane::XY: u0=lo.x(); u1=hi.x(); v0=lo.y(); v1=hi.y(); break;
        case SlicePlane::XZ: u0=lo.x(); u1=hi.x(); v0=lo.z(); v1=hi.z(); break;
        case SlicePlane::YZ: u0=lo.y(); u1=hi.y(); v0=lo.z(); v1=hi.z(); break;
    }
    double du = (u1 - u0) / (resolution - 1);
    double dv = (v1 - v0) / (resolution - 1);

    for (int i = 0; i < resolution; ++i) {
        double u = u0 + i * du;
        for (int j = 0; j < resolution; ++j) {
            double v = v0 + j * dv;
            G4ThreeVector p;
            switch (plane) {
                case SlicePlane::XY: p = G4ThreeVector(u, v, 0.0); break;
                case SlicePlane::XZ: p = G4ThreeVector(u, 0.0, v); break;
                case SlicePlane::YZ: p = G4ThreeVector(0.0, u, v); break;
            }
            CrossSectionPoint pt;
            pt.x = static_cast<float>(p.x());
            pt.y = static_cast<float>(p.y());
            pt.z = static_cast<float>(p.z());
            pt.classification = ClassifyUnion(solids, p);
            pts.push_back(pt);
        }
    }
    return pts;
}

void CrossSectionSampler::RunAll(
    const std::vector<G4VSolid*>& solids,
    const G4ThreeVector& bbox_min,
    const G4ThreeVector& bbox_max,
    const CrossSectionConfig& cfg,
    const std::string& geometry_name,
    const std::string& mode,
    OutputWriter& writer)
{
    for (SlicePlane plane : cfg.planes) {
        std::string pname = PlaneToString(plane);
        std::cout << "[CrossSectionSampler] Sampling " << geometry_name
                  << " (" << mode << ") plane=" << pname
                  << " res=" << cfg.resolution << "x" << cfg.resolution << " ...\n";
        auto pts = Sample(solids, bbox_min, bbox_max, plane, cfg.resolution, cfg.padding);
        writer.WriteCrossSection(geometry_name, mode, pname, pts);
        std::cout << "[CrossSectionSampler]   -> wrote " << pts.size() << " points\n";
    }
}

std::string CrossSectionSampler::PlaneToString(SlicePlane p) {
    switch (p) {
        case SlicePlane::XY: return "xy";
        case SlicePlane::XZ: return "xz";
        case SlicePlane::YZ: return "yz";
    }
    return "xy";
}

SlicePlane CrossSectionSampler::ParsePlane(const std::string& s) {
    if (s == "xy") return SlicePlane::XY;
    if (s == "xz") return SlicePlane::XZ;
    if (s == "yz") return SlicePlane::YZ;
    throw std::runtime_error("Unknown plane: " + s + ". Valid: xy, xz, yz");
}
