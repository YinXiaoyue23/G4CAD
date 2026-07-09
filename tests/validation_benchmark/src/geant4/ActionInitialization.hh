#pragma once
#include "G4VUserActionInitialization.hh"
#include "G4ThreeVector.hh"
#include "OutputWriter.hh"
#include <string>

struct PrimaryConfig {
    std::string   particle  = "e-";
    double        energyMeV = 10.0;
    G4ThreeVector direction = G4ThreeVector(0, 0, 1);
    // Offset from Z axis so the particle avoids sphere poles (degenerate OCCT
    // BREP points) and the box_hole centre hole (radius 20 mm).
    G4ThreeVector position  = G4ThreeVector(30, 0, -200);
};

class ActionInitialization : public G4VUserActionInitialization {
public:
    ActionInitialization(OutputWriter*       writer,
                         const PrimaryConfig& primaryCfg,
                         std::string          geometryName,
                         std::string          mode);

    void BuildForMaster() const override;
    void Build() const override;

private:
    OutputWriter*  fWriter;
    PrimaryConfig  fPrimary;
    std::string    fGeometryName;
    std::string    fMode;
};
