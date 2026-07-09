#include "DetectorConstruction.hh"
#include "G4Box.hh"
#include "G4LogicalVolume.hh"
#include "G4PVPlacement.hh"
#include "G4NistManager.hh"
#include "G4SystemOfUnits.hh"

DetectorConstruction::DetectorConstruction(std::vector<G4VSolid*> solids,
                                            std::string material,
                                            double worldHalfMm)
    : fSolids(std::move(solids)), fMaterial(std::move(material)),
      fWorldHalf(worldHalfMm) {}

G4VPhysicalVolume* DetectorConstruction::Construct() {
    auto* nist = G4NistManager::Instance();
    auto* worldMat = nist->FindOrBuildMaterial("G4_AIR");
    auto* detMat   = nist->FindOrBuildMaterial(fMaterial);

    auto* worldBox = new G4Box("world", fWorldHalf*mm, fWorldHalf*mm, fWorldHalf*mm);
    auto* worldLV  = new G4LogicalVolume(worldBox, worldMat, "world");
    auto* worldPV  = new G4PVPlacement(nullptr, {}, worldLV, "world", nullptr, false, 0);

    if (fSolids.size() == 1) {
        auto* lv = new G4LogicalVolume(fSolids[0], detMat, fSolids[0]->GetName());
        new G4PVPlacement(nullptr, G4ThreeVector(0,0,0), lv,
                          fSolids[0]->GetName(), worldLV, false, 0);
    } else {
        // touching_boxes: offset is baked into G4DisplacedSolid; place at origin
        for (std::size_t i = 0; i < fSolids.size(); ++i) {
            auto* lv = new G4LogicalVolume(fSolids[i], detMat, fSolids[i]->GetName());
            new G4PVPlacement(nullptr, G4ThreeVector(0, 0, 0), lv,
                              fSolids[i]->GetName(), worldLV, false, (int)i);
        }
    }

    return worldPV;
}
