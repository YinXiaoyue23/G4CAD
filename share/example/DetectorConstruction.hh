#pragma once

#include "G4VUserDetectorConstruction.hh"
#include <string>

class DetectorConstruction : public G4VUserDetectorConstruction {
public:
    explicit DetectorConstruction(const std::string& stepFile);
    G4VPhysicalVolume* Construct() override;
private:
    std::string fStepFile;
};
