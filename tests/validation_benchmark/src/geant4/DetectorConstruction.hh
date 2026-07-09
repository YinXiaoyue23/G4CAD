#pragma once
#include "G4VUserDetectorConstruction.hh"
#include "G4VSolid.hh"
#include <vector>
#include <string>

class DetectorConstruction : public G4VUserDetectorConstruction {
public:
    DetectorConstruction(std::vector<G4VSolid*> solids,
                         std::string material     = "G4_Si",
                         double      worldHalfMm  = 300.0);

    G4VPhysicalVolume* Construct() override;

private:
    std::vector<G4VSolid*> fSolids;
    std::string            fMaterial;
    double                 fWorldHalf;
};
