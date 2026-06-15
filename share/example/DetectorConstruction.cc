#include "DetectorConstruction.hh"
#include "G4StepLoader.hh"
#include "G4StepSolid.hh"

#include "G4Box.hh"
#include "G4LogicalVolume.hh"
#include "G4PVPlacement.hh"
#include "G4NistManager.hh"
#include "G4SystemOfUnits.hh"

DetectorConstruction::DetectorConstruction(const std::string& stepFile)
    : fStepFile(stepFile) {}

G4VPhysicalVolume* DetectorConstruction::Construct() {
    auto* nist = G4NistManager::Instance();

    // World volume — 600 × 400 × 600 mm air box
    auto* worldSolid = new G4Box("World", 300*mm, 200*mm, 300*mm);
    auto* worldLV    = new G4LogicalVolume(worldSolid,
                          nist->FindOrBuildMaterial("G4_AIR"), "World");
    auto* worldPV    = new G4PVPlacement(nullptr, {}, worldLV,
                          "World", nullptr, false, 0, false);

    // Load STEP file.
    // G4StepLoader::Load returns the individual constituent solids of the
    // assembly.  Each TopoDS_Shape carries its position in the assembly
    // coordinate frame via its TopLoc_Location, so placing every solid at the
    // world origin (identity transform) correctly reconstructs the assembly.
    auto shapes = G4StepLoader::Load(fStepFile);
    if (shapes.empty()) {
        G4cerr << "DetectorConstruction: no solids loaded from "
               << fStepFile << G4endl;
        return worldPV;
    }

    // Choose the material that matches your model; silicon is used here for RP.step.
    auto* material = nist->FindOrBuildMaterial("G4_Si");

    for (std::size_t i = 0; i < shapes.size(); ++i) {
        auto  name  = "Solid_" + std::to_string(i);
        auto* solid = new G4StepSolid(name, shapes[i]);
        auto* lv    = new G4LogicalVolume(solid, material, name);
        // No overlap check: STEP-assembly tolerances are handled by G4StepSolid
        // internally; Geant4's overlap checker is not appropriate here.
        new G4PVPlacement(nullptr, {}, lv, name, worldLV, false, (G4int)i, false);
    }

    return worldPV;
}
