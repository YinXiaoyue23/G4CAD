#include "GeometryRegistry.hh"
#include "G4StepLoader.hh"
#include "G4StepSolid.hh"
#include "G4Box.hh"
#include "G4Sphere.hh"
#include "G4Tubs.hh"
#include "G4SubtractionSolid.hh"
#include "G4DisplacedSolid.hh"
#include "G4SystemOfUnits.hh"
#include "G4PhysicalConstants.hh"
#include <stdexcept>
#include <cmath>

// All dimensions in Geant4 units (mm by default via G4SystemOfUnits.hh)

std::vector<G4VSolid*> GeometryRegistry::CreateNative(GeometryType type) {
    switch (type) {
        case GeometryType::Box:
            return { new G4Box("box", 50*mm, 50*mm, 50*mm) };

        case GeometryType::Sphere:
            return { new G4Sphere("sphere", 0, 50*mm, 0, CLHEP::twopi, 0, CLHEP::pi) };

        case GeometryType::Cylinder:
            return { new G4Tubs("cylinder", 0, 50*mm, 50*mm, 0, CLHEP::twopi) };

        case GeometryType::BoxHole: {
            auto* box  = new G4Box("bh_box", 100*mm, 100*mm, 100*mm);
            auto* hole = new G4Tubs("bh_hole", 0, 20*mm, 101*mm, 0, CLHEP::twopi);
            return { new G4SubtractionSolid("box_hole", box, hole) };
        }

        case GeometryType::TouchingBoxes: {
            // Boxes are placed at ±50mm in DetectorConstruction.
            // Bake the offsets into G4DisplacedSolid so that Inside() uses
            // world-frame coordinates, matching the STEP solids which encode
            // absolute position from FreeCAD.
            auto* raw_L = new G4Box("box_L_raw", 50*mm, 100*mm, 100*mm);
            auto* raw_R = new G4Box("box_R_raw", 50*mm, 100*mm, 100*mm);
            return {
                new G4DisplacedSolid("box_L", raw_L,
                    G4Transform3D(HepGeom::Translate3D(-50*mm, 0, 0))),
                new G4DisplacedSolid("box_R", raw_R,
                    G4Transform3D(HepGeom::Translate3D(+50*mm, 0, 0)))
            };
        }
    }
    throw std::runtime_error("GeometryRegistry::CreateNative: unknown type");
}

std::vector<G4VSolid*> GeometryRegistry::CreateFromStep(const std::string& step_file,
                                                          double tolerance) {
    auto shapes = G4StepLoader::Load(step_file);
    std::vector<G4VSolid*> solids;
    solids.reserve(shapes.size());
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        solids.push_back(
            new G4StepSolid("step_part_" + std::to_string(i), shapes[i], tolerance)
        );
    }
    return solids;
}

GeometryInfo GeometryRegistry::GetInfo(GeometryType type) {
    const double pi = CLHEP::pi;
    switch (type) {
        case GeometryType::Box:
            return { "box",
                     1e6,                        // (2*50)^3 = 1e6 mm^3
                     G4ThreeVector(-50,-50,-50),
                     G4ThreeVector( 50, 50, 50),
                     1, 1e-7 };

        case GeometryType::Sphere:
            return { "sphere",
                     (4.0/3.0)*pi*125000.0,      // 4/3 * pi * 50^3
                     G4ThreeVector(-50,-50,-50),
                     G4ThreeVector( 50, 50, 50),
                     1, 1e-7 };

        case GeometryType::Cylinder:
            return { "cylinder",
                     pi * 2500.0 * 100.0,        // pi * 50^2 * 100 mm
                     G4ThreeVector(-50,-50,-50),
                     G4ThreeVector( 50, 50, 50),
                     1, 1e-7 };

        case GeometryType::BoxHole:
            return { "box_hole",
                     8000000.0 - pi * 400.0 * 202.0,  // (2*100)^3 - pi*20^2*(2*101)
                     G4ThreeVector(-100,-100,-100),
                     G4ThreeVector( 100, 100, 100),
                     1, 1e-7 };

        case GeometryType::TouchingBoxes:
            return { "touching_boxes",
                     2.0 * 100.0 * 200.0 * 200.0,  // 2 * (100*200*200)
                     G4ThreeVector(-100,-100,-100),
                     G4ThreeVector( 100, 100, 100),
                     2, 1e-7 };
    }
    throw std::runtime_error("GeometryRegistry::GetInfo: unknown type");
}

GeometryType GeometryRegistry::Parse(const std::string& name) {
    if (name == "box")            return GeometryType::Box;
    if (name == "sphere")         return GeometryType::Sphere;
    if (name == "cylinder")       return GeometryType::Cylinder;
    if (name == "box_hole")       return GeometryType::BoxHole;
    if (name == "touching_boxes") return GeometryType::TouchingBoxes;
    throw std::runtime_error("Unknown geometry: " + name +
        ". Valid: box, sphere, cylinder, box_hole, touching_boxes");
}

std::string GeometryRegistry::ToString(GeometryType type) {
    switch (type) {
        case GeometryType::Box:           return "box";
        case GeometryType::Sphere:        return "sphere";
        case GeometryType::Cylinder:      return "cylinder";
        case GeometryType::BoxHole:       return "box_hole";
        case GeometryType::TouchingBoxes: return "touching_boxes";
    }
    return "unknown";
}
