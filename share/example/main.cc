// G4CAD minimal example
// Loads a STEP file and runs a short Geant4 simulation.
//
// Build:
//   cmake -S . -B build -DG4CAD_DIR=/path/to/g4cad/install/lib/cmake/G4CAD
//   cmake --build build -j
//
// Run:
//   cd build && ./example ../RP.step
//
// Optional env vars (see G4StepSolid documentation):
//   G4CAD_RECORD=queries.csv         record every navigation query to CSV
//   G4CAD_RECORD_EVENT=N             record only run N
//   G4CAD_TOL_VERBOSE=1              print per-solid tolerance table at startup
//   G4CAD_PERSOLID_TOL=1             use per-solid surface band (default: off)

#include "DetectorConstruction.hh"

#include "G4RunManager.hh"
#include "FTFP_BERT.hh"
#include "G4ParticleGun.hh"
#include "G4ParticleTable.hh"
#include "G4SystemOfUnits.hh"
#include "G4VUserActionInitialization.hh"
#include "G4VUserPrimaryGeneratorAction.hh"
#include "G4Event.hh"

namespace {

// 120 GeV proton along +Z, starting 280 mm upstream of the detector centre
class PrimaryGenerator : public G4VUserPrimaryGeneratorAction {
public:
    PrimaryGenerator() : fGun(new G4ParticleGun(1)) {
        auto* p = G4ParticleTable::GetParticleTable()->FindParticle("proton");
        fGun->SetParticleDefinition(p);
        fGun->SetParticleMomentumDirection(G4ThreeVector(0, 0, 1));
        fGun->SetParticleEnergy(120*GeV);
        fGun->SetParticlePosition(G4ThreeVector(0, 0, -280*mm));
    }
    ~PrimaryGenerator() override { delete fGun; }
    void GeneratePrimaries(G4Event* evt) override {
        fGun->GeneratePrimaryVertex(evt);
    }
private:
    G4ParticleGun* fGun;
};

class ActionInit : public G4VUserActionInitialization {
public:
    void BuildForMaster() const override {}
    void Build() const override { SetUserAction(new PrimaryGenerator); }
};

} // namespace

int main(int argc, char* argv[]) {
    const char* stepFile = (argc > 1) ? argv[1] : "RP.step";

    auto* rm = new G4RunManager;
    rm->SetUserInitialization(new DetectorConstruction(stepFile));
    rm->SetUserInitialization(new FTFP_BERT);
    rm->SetUserInitialization(new ActionInit);
    rm->Initialize();

    G4cout << "\n=== Running 10 events ===\n" << G4endl;
    rm->BeamOn(10);
    G4cout << "\n=== Done ===\n" << G4endl;

    delete rm;
    return 0;
}
